"""Raw chest-LiDAR -> 2D LaserScan WITHOUT FAST-LIO (A/B experiment, P16 20261029-raw-scan-ab).

Same output geometry as level_bridge.py's /qazbot_lio/scan, but built straight from the HAL cloud:
  * input  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud (frame lidar_chest_front, ~10 Hz,
    fields x y z intensity ring timestamp; timestamp = float64 ns after header.stamp, 0..~95 ms);
  * extrinsic lidar_chest_front -> lidar_imu_chest_front: x2.yaml extrinsic_R/extrinsic_T (= inverse of
    the robot's /tf_static lidar_chest_front -> lidar_imu_chest_front, re-checked live 2026-10-29);
  * attitude + short-term motion: /aima/mc/leg_odometry (MC estimator, ~150 Hz, pose of
    lidar_imu_chest_front in the gravity-levelled leg_odom). Used ONLY for (a) roll/pitch levelling and
    (b) per-point deskew inside one sweep (relative pose over <=0.1 s); its absolute x/y/yaw are not used;
  * base_footprint = level_bridge's definition: the LiDAR IMU projected on the floor, yaw = heading of the
    LiDAR forward axis, so a later switch keeps AMCL laser pose / Trust frames valid;
  * slice = level_bridge's Z_MIN/Z_MAX/FLOOR_MARGIN_PER_M relative to the IMU, 1440 bins, range_max param.
Publishes only under /qazbot_raw/ (scan, scan_obstacles, scan_clearing, optional scan_low). No TF, no odom.

CPU: the cloud and pose subscriptions are raw (no deserialisation of skipped clouds); the 150 Hz pose stream
is never woken by the executor -- it sits in a KEEP_LAST queue of a second, unspun node and is drained in a
batch on every processed cloud (rclpy wake-ups at 150-200 Hz cost ~25 % of a core on the X2).
"""
import json
import math
import os
import struct
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import level_bridge as lb  # noqa: E402  (pure functions + constants; no node is created on import)

CLOUD_TOPIC = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
POSE_TOPIC = '/aima/mc/leg_odometry'
IMU_TOPIC = '/aima/hal/sensor/lidar_chest_front/imu'   # one-shot gravity cross-check only
#: x2.yaml mapping.extrinsic_R (row-major R_imu<-lidar) and extrinsic_T: p_imu = R p_lidar + T.
R_IL = np.array([-0.003843965609489519, -0.9999545839883389, 0.008720888091315271,
                 0.0008810920694298785, 0.008717562353234953, 0.9999616131549165,
                 -0.9999922237723444, 0.0038515019571126463, 0.0008475420438577297]).reshape(3, 3)
T_IL = np.array([0.004274399951100347, -0.0157517995685339, -0.011211999692022804])
N_SLICES = 10            # deskew: piecewise-constant pose per 1/10 of the sweep (~10 ms)


def quat_xyzw_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def parse_odom_cdr(buf):
    """(stamp s, frame_id, child_frame_id, px,py,pz, qx,qy,qz,qw) from a serialized nav_msgs/Odometry."""
    sec, nsec = struct.unpack_from('<iI', buf, 4)
    off = 8
    n = struct.unpack_from('<I', buf, 4 + off)[0]
    frame = bytes(buf[4 + off + 4:4 + off + 3 + n]).decode()
    off += 4 + n
    off = (off + 3) & ~3
    n = struct.unpack_from('<I', buf, 4 + off)[0]
    child = bytes(buf[4 + off + 4:4 + off + 3 + n]).decode()
    off += 4 + n
    off = (off + 7) & ~7
    return (sec + nsec * 1e-9, frame, child) + struct.unpack_from('<7d', buf, 4 + off)


def cloud_arrays(m):
    """xyz (N,3) float64 and per-point offset (s) of the HAL cloud, finite points only."""
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z', 't'], 'formats': ['<f4', '<f4', '<f4', '<f8'],
                   'offsets': [off['x'], off['y'], off['z'], off['timestamp']], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    P = np.column_stack((a['x'], a['y'], a['z'])).astype(np.float64)
    t = a['t'] * 1e-9
    ok = np.isfinite(P).all(axis=1) & np.isfinite(t)
    return P[ok], t[ok]


class PoseHistory:
    def __init__(self, n=600):
        self.t = np.zeros(0)
        self.p = np.zeros((0, 3))
        self.q = np.zeros((0, 4))
        self.n = n

    def add(self, rows):
        if not rows:
            return 0
        a = np.array(rows)
        if len(self.t) and a[0, 0] < self.t[-1] - 0.5:      # sensor clock stepped back: restart
            self.t, self.p, self.q = np.zeros(0), np.zeros((0, 3)), np.zeros((0, 4))
        self.t = np.concatenate((self.t, a[:, 0]))[-self.n:]
        self.p = np.concatenate((self.p, a[:, 1:4]))[-self.n:]
        self.q = np.concatenate((self.q, a[:, 4:8]))[-self.n:]
        o = np.argsort(self.t, kind='stable')
        self.t, self.p, self.q = self.t[o], self.p[o], self.q[o]
        return len(rows)

    def at(self, ts):
        """Interpolated (R (K,3,3), p (K,3)) at times ts; also the max clamp distance (s)."""
        t = self.t
        i = np.clip(np.searchsorted(t, ts), 1, len(t) - 1)
        t0, t1 = t[i - 1], t[i]
        w = np.clip((ts - t0) / np.maximum(t1 - t0, 1e-9), 0.0, 1.0)[:, None]
        p = self.p[i - 1] * (1 - w) + self.p[i] * w
        q0, q1 = self.q[i - 1], self.q[i].copy()
        q1[(q0 * q1).sum(1) < 0] *= -1
        q = q0 * (1 - w) + q1 * w
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        R = np.stack([quat_xyzw_to_rot(x) for x in q])
        clamp = float(np.max(np.maximum(t[0] - ts, ts - t[-1]).clip(0)))
        return R, p, clamp


def slice_scan(Q, z_min, z_max, margin, range_min, range_max):
    """level_bridge.on_cloud's slice/binning on points already in base_footprint (z rel. IMU)."""
    flat = np.hypot(Q[:, 0], Q[:, 1])
    S = Q[(Q[:, 2] > z_min + margin * flat) & (Q[:, 2] < z_max)]
    r = np.hypot(S[:, 0], S[:, 1])
    a = np.arctan2(S[:, 1], S[:, 0])
    ok = (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    ranges = np.full(lb.N_BINS, np.inf, dtype=np.float32)
    idx = ((a + math.pi) / (2 * math.pi) * lb.N_BINS).astype(int) % lb.N_BINS
    np.minimum.at(ranges, idx, r.astype(np.float32))
    order = np.lexsort((r, idx))
    _, first = np.unique(idx[order], return_index=True)
    sel = order[first]
    obstacles = np.column_stack((r[sel] * np.cos(a[sel]), r[sel] * np.sin(a[sel]), np.zeros(len(sel))))
    return ranges, obstacles


class RawScanBridge(Node):
    def __init__(self):
        super().__init__('qazbot_raw_scan_bridge')
        P = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.every = int(P('scan_every', 2))
        self.range_max = float(P('range_max', 8.0))
        self.range_min = float(P('range_min', lb.RANGE_MIN))
        self.z_min, self.z_max = float(P('z_min', lb.Z_MIN)), float(P('z_max', lb.Z_MAX))
        self.margin = float(P('floor_margin_per_m', lb.FLOOR_MARGIN_PER_M))
        self.deskew = bool(P('deskew', True))
        #: false (default): stamp/deskew reference = sweep start (= cloud header.stamp);
        #: true: sweep end (cloud stamp + last point offset) -- FAST-LIO's convention
        self.stamp_at_end = bool(P('stamp_at_end', False))
        self.clearing = bool(P('publish_clearing', True))
        #: optional second, low slice 'z_min,z_max,margin_per_m' (rel. IMU; floor is ~-0.91) to imitate the
        #: D1 map's single ~0.2-0.3 m lidar plane; '' = off
        low = str(P('low_slice', ''))
        self.low = tuple(float(x) for x in low.split(',')) if low else None
        prefix = str(P('prefix', '/qazbot_raw'))
        if not prefix.startswith('/qazbot_raw'):
            raise SystemExit('refusing to publish outside /qazbot_raw')
        self.pub_scan = self.create_publisher(LaserScan, prefix + '/scan', 10)
        self.pub_obs = self.create_publisher(PointCloud2, prefix + '/scan_obstacles', 10)
        self.pub_clr = self.create_publisher(PointCloud2, prefix + '/scan_clearing', 10)
        self.pub_low = self.create_publisher(LaserScan, prefix + '/scan_low', 10) if self.low else None
        be = lambda d: QoSProfile(depth=d, reliability=ReliabilityPolicy.BEST_EFFORT,  # noqa: E731
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, be(2), raw=True)
        # Passive node: never added to the executor; queues drained by take_message().
        self.passive = rclpy.create_node('qazbot_raw_scan_bridge_passive')
        self.sub_pose = self.passive.create_subscription(Odometry, POSE_TOPIC, lambda m: None, be(400), raw=True)
        self.sub_imu = self.passive.create_subscription(Imu, IMU_TOPIC, lambda m: None, be(400))
        self.hist = PoseHistory()
        self.n_cloud = 0
        self.stats = dict(clouds=0, scans=0, skipped_nopose=0, pose_msgs=0, max_clamp_ms=0.0, proc_ms=[], floor=[])
        self.t_stat = time.monotonic()
        self.checked = False
        self.get_logger().info(
            f'raw_scan_bridge: {CLOUD_TOPIC} + {POSE_TOPIC} -> {prefix}/scan(+_obstacles,_clearing'
            f'{",_low" if self.low else ""}) every={self.every} range_max={self.range_max} '
            f'slice=({self.z_min},{self.z_max}) margin={self.margin} deskew={self.deskew} stamp_at_end={self.stamp_at_end} low={self.low}')

    def drain(self, sub, limit=2000):
        out = []
        for _ in range(limit):
            with sub.handle:
                r = sub.handle.take_message(sub.msg_type, sub.raw)
            if r is None:
                break
            out.append(r[0])
        return out

    def drain_pose(self):
        rows = []
        for buf in self.drain(self.sub_pose):
            t, frame, child, *v = parse_odom_cdr(buf)
            if child == 'lidar_imu_chest_front':
                rows.append([t] + v)
        self.stats['pose_msgs'] += self.hist.add(rows)

    def gravity_check(self):
        """One-shot: angle between leg_odom 'up' and the LiDAR-IMU mean accel (validates the levelling)."""
        imus = self.drain(self.sub_imu)
        if len(imus) < 20 or len(self.hist.t) < 2:
            return
        acc = np.mean([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imus], 0)
        ts = np.array([imus[-1].header.stamp.sec + imus[-1].header.stamp.nanosec * 1e-9])
        R, _, _ = self.hist.at(ts)
        up_imu = R[0].T @ np.array([0, 0, 1.0])
        ang = math.degrees(math.acos(np.clip(up_imu @ acc / np.linalg.norm(acc), -1, 1)))
        self.get_logger().info(f'gravity check: leg_odom up vs LiDAR-IMU accel ({len(imus)} msgs) = {ang:.2f} deg')
        self.passive.destroy_subscription(self.sub_imu)
        self.checked = True

    def on_cloud(self, raw):
        self.n_cloud += 1
        self.stats['clouds'] += 1
        if self.n_cloud % self.every:
            return
        t_wall = time.perf_counter()
        self.drain_pose()
        if not self.checked:
            self.gravity_check()
        if len(self.hist.t) < 2:
            self.stats['skipped_nopose'] += 1
            return
        m = deserialize_message(raw, PointCloud2)
        P, dt = cloud_arrays(m)
        if not len(P):
            return
        t0 = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        Pi = P @ R_IL.T + T_IL                                   # lidar -> LiDAR-IMU frame
        span = max(float(dt.max()), 1e-3)
        k = np.minimum((dt / span * N_SLICES).astype(int), N_SLICES - 1) if self.deskew else np.zeros(len(dt), int)
        t_ref = t0 + span if self.stamp_at_end else t0
        ts = t0 + (np.arange(N_SLICES) + 0.5) * span / N_SLICES if self.deskew else np.array([t0])
        Rk, pk, clamp = self.hist.at(np.concatenate(([t_ref], ts)))
        R_ref, p_ref = Rk[0], pk[0]
        Rk, pk = Rk[1:], pk[1:]
        W = np.empty_like(Pi)
        for j in range(len(ts)):                                 # into leg_odom (levelled), origin = IMU at t_ref
            s = k == j
            W[s] = Pi[s] @ Rk[j].T + (pk[j] - p_ref)
        f = R_ref @ lb.LIDAR_FWD_IN_IMU
        yaw = math.atan2(f[1], f[0])
        c, sn = math.cos(-yaw), math.sin(-yaw)
        Q = np.column_stack((c * W[:, 0] - sn * W[:, 1], sn * W[:, 0] + c * W[:, 1], W[:, 2]))
        ranges, obstacles = slice_scan(Q, self.z_min, self.z_max, self.margin, self.range_min, self.range_max)
        scan = LaserScan()
        scan.header.stamp = m.header.stamp
        if self.stamp_at_end:
            ns = int(round(t_ref * 1e9))
            scan.header.stamp.sec, scan.header.stamp.nanosec = ns // 10**9, ns % 10**9
        scan.header.frame_id = 'base_footprint'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / lb.N_BINS
        scan.angle_max = scan.angle_min + scan.angle_increment * (lb.N_BINS - 1)
        scan.range_min, scan.range_max = self.range_min, self.range_max
        scan.ranges = ranges.tolist()
        self.pub_scan.publish(scan)
        self.pub_obs.publish(pc2.create_cloud_xyz32(scan.header, obstacles.tolist()))
        if self.clearing:
            # clearing_endpoints uses level_bridge's module constants Z_MIN/Z_MAX (same defaults as here).
            clr = lb.clearing_endpoints(Q, 0.0, self.range_max)
            self.pub_clr.publish(pc2.create_cloud_xyz32(scan.header, clr.tolist()))
        if self.low:
            lo, _ = slice_scan(Q, self.low[0], self.low[1], self.low[2], self.range_min, self.range_max)
            s2 = LaserScan()
            s2.header, s2.angle_min, s2.angle_max = scan.header, scan.angle_min, scan.angle_max
            s2.angle_increment, s2.range_min, s2.range_max = scan.angle_increment, scan.range_min, scan.range_max
            s2.ranges = lo.tolist()
            self.pub_low.publish(s2)
        st = self.stats
        st['scans'] += 1
        st['max_clamp_ms'] = max(st['max_clamp_ms'], clamp * 1e3)
        st['proc_ms'].append((time.perf_counter() - t_wall) * 1e3)
        fl = Q[(np.hypot(Q[:, 0], Q[:, 1]) > 1.0) & (np.hypot(Q[:, 0], Q[:, 1]) < 3.0) & (Q[:, 2] < -0.7), 2]
        if len(fl) > 50:
            st['floor'].append(float(np.median(fl)))
        if time.monotonic() - self.t_stat > 30:
            pm = np.array(st['proc_ms'])
            self.get_logger().info('stats ' + json.dumps(dict(
                clouds=st['clouds'], scans=st['scans'], skipped_nopose=st['skipped_nopose'], pose_msgs=st['pose_msgs'],
                max_clamp_ms=round(st['max_clamp_ms'], 1), proc_ms_p50=round(float(np.median(pm)), 1) if len(pm) else None,
                proc_ms_max=round(float(pm.max()), 1) if len(pm) else None,
                floor_z_rel_imu=round(float(np.median(st['floor'])), 3) if st['floor'] else None,
                finite_bins=int(np.isfinite(ranges).sum()),
                lidar_down_deg=round(-math.degrees(math.asin(np.clip(f[2], -1, 1))), 2))))
            self.t_stat = time.monotonic()
            st.update(clouds=0, scans=0, skipped_nopose=0, pose_msgs=0, max_clamp_ms=0.0, proc_ms=[], floor=[])


def main():
    rclpy.init()
    node = RawScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.passive.destroy_node()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
