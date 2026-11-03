"""FAST-LIO -> 2D for slam_toolbox: a gravity-levelled planar pose and a
height slice of the deskewed cloud as a LaserScan.

FAST-LIO's world is its first IMU frame (the LiDAR's own IMU on the X2, whose
x axis is roughly up), so nothing it publishes is level. Here:
  * the level rotation comes from the LiDAR-IMU acceleration rotated into
    FAST-LIO's world by its current orientation and averaged over LEVEL_S
    seconds -- gravity in that world whenever the bridge starts (walking
    accelerations average out), so the start order against FAST-LIO does
    not matter;
  * base_footprint = the IMU position, yaw = heading of the LiDAR's forward
    axis (= the torso's x, /tf_static) projected on the floor;
  * TF odom -> base_footprint and /odom_level carry that pose (TF only with
    publish_tf:=true, the default; localization.launch.py odom_source:=native_imu
    sets it false so native_odom_bridge.py is the ONE odom->base_footprint owner,
    while /odom_level and /scan stay exactly as they are -- the scan needs only the
    LIO pose at the cloud stamp, which brings cloud_registered back to the body);
  * /scan = points of /cloud_registered (world frame, deskewed; publish with
    dense_publish_en for the full cloud) between Z_MIN and Z_MAX relative to
    the IMU, binned by bearing in base_footprint.

Inputs:  /Odometry, /cloud_registered, the LiDAR IMU.  use_sim_time on replay.
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import TransformBroadcaster

IMU_TOPIC = '/aima/hal/sensor/lidar_chest_front/imu'
LEVEL_S = 1.0
#: LiDAR forward (x) in lidar_imu_chest_front: column 0 of R_imu<-lidar, the
#: inverse of /tf_static lidar_chest_front -> lidar_imu_chest_front.
LIDAR_FWD_IN_IMU = np.array([-0.003843965609489519, 0.0008810920694298785, -0.9999922237723444])
#: Slice heights relative to the LiDAR IMU (floor is ~1.0 m below it when
#: standing): 0.35 .. 1.8 m above the floor. The LiDAR looks ~5 deg down and
#: the body sways 1-2 deg while walking, so far away the floor rises into a
#: fixed slice (+-0.5 m at 15 m) and draws fans of false walls: the floor
#: margin grows with range, and the scan stops at RANGE_MAX.
Z_MIN, Z_MAX = -0.65, 0.80
FLOOR_MARGIN_PER_M = 0.035      # ~2 deg of sway
#: Twist baseline: differencing 10 Hz poses ~3 mm apart is noise; 0.3 s keeps
#: it low and lags the true velocity by half the baseline.
TWIST_BASELINE_S = 0.3
N_BINS = 1440
RANGE_MIN, RANGE_MAX = 0.45, 25.0


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def level_rotation(up):
    """Rotation taking `up` (unit, FAST-LIO world) onto +z."""
    z = up / np.linalg.norm(up)
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = a - (a @ z) * z
    x /= np.linalg.norm(x)
    return np.vstack([x, np.cross(z, x), z])


class LevelBridge(Node):
    def __init__(self, **kwargs):          # kwargs: rclpy Node options (tests: parameter_overrides)
        super().__init__('level_bridge', **kwargs)
        self.acc, self.t_acc0, self.R_lv, self.R_wb = [], None, None, None
        self.poses = {}            # stamp ns -> (x, y, yaw, p_lv)
        self.track = deque()       # (t s, x, y, yaw) for the twist baseline
        self.last_key = None       # newest odometry stamp seen, ns
        #: publish_tf false: no TF publisher at all (single odom->base_footprint owner,
        #: checked on the graph); /odom_level and /scan are unchanged.
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        be = QoSProfile(depth=400, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.sub_imu = self.create_subscription(Imu, IMU_TOPIC, self.on_imu, be)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 100)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, 20)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 20)
        self.pub_odom = self.create_publisher(Odometry, '/odom_level', 50)
        global RANGE_MAX
        RANGE_MAX = float(self.declare_parameter('range_max', RANGE_MAX).value)
        #: Slice every Nth cloud: 5 Hz is plenty for the map, and on the robot
        #: the numpy slice of the dense cloud is ~30 % of a core at 10 Hz.
        self.every = int(self.declare_parameter('scan_every', 1).value)
        self.n_cloud = 0

    def on_imu(self, m):
        if self.R_lv is not None or self.R_wb is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.t_acc0 = self.t_acc0 or t
        a = m.linear_acceleration
        self.acc.append(self.R_wb @ np.array([a.x, a.y, a.z]))
        if t - self.t_acc0 >= LEVEL_S:
            up = np.mean(self.acc, axis=0)
            self.R_lv = level_rotation(up)
            self.get_logger().info(f'levelled: up in FAST-LIO world {np.round(up / np.linalg.norm(up), 4)}')
            # Done with the IMU: 200 msg/s through rclpy were ~27 % of a core
            # on the robot even when every one of them was ignored.
            self.destroy_subscription(self.sub_imu)

    def on_odom(self, m):
        self.R_wb = quat_to_rot(m.pose.pose.orientation)
        if self.R_lv is None:
            return
        p = m.pose.pose.position
        R = self.R_lv @ self.R_wb
        p_lv = self.R_lv @ np.array([p.x, p.y, p.z])
        f = R @ LIDAR_FWD_IN_IMU
        yaw = math.atan2(f[1], f[0])
        key = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if self.last_key is not None and key < self.last_key:
            # The sensor clock stepped back (PTP resync, observed with FAST-LIO
            # "lidar loop back"). Poses and the twist baseline keyed by the old,
            # now-future stamps would block the scan lookup and the twist forever.
            self.get_logger().warn(f'odometry stamp went back {(self.last_key - key) * 1e-9:.3f} s: '
                                   'clearing pose cache and twist baseline')
            self.poses.clear()
            self.track.clear()
        self.last_key = key
        self.poses[key] = (p_lv[0], p_lv[1], yaw, p_lv)
        if len(self.poses) > 50:
            for k in sorted(self.poses)[:-50]:
                del self.poses[k]
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = float(p_lv[0]), float(p_lv[1])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.tf is not None:
            self.tf.sendTransform(t)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.track.append((stamp, p_lv[0], p_lv[1], yaw))
        while len(self.track) > 1 and stamp - self.track[1][0] >= TWIST_BASELINE_S:
            self.track.popleft()
        t0, x0, y0, yaw0 = self.track[0]
        if stamp - t0 < TWIST_BASELINE_S:
            return      # no velocity yet: publish no Odometry rather than a zero twist
        dt = stamp - t0
        dx, dy = p_lv[0] - x0, p_lv[1] - y0
        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = float(p_lv[0]), float(p_lv[1])
        o.pose.pose.position.z = float(p_lv[2])
        o.pose.pose.orientation = t.transform.rotation
        # Velocity in base_footprint (REP-105: twist is in the child frame).
        c, sn = math.cos(-yaw), math.sin(-yaw)
        o.twist.twist.linear.x = (c * dx - sn * dy) / dt
        o.twist.twist.linear.y = (sn * dx + c * dy) / dt
        o.twist.twist.angular.z = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)) / dt
        # FAST-LIO's covariance is [pos, rot] in its own (tilted) world; rotate
        # it into the level frame. Yaw variance is first order (it ignores the
        # forward-axis projection).
        B = np.kron(np.eye(2), self.R_lv)
        o.pose.covariance = (B @ np.array(m.pose.covariance).reshape(6, 6) @ B.T).ravel().tolist()
        self.pub_odom.publish(o)

    def on_cloud(self, m):
        self.n_cloud += 1
        if self.n_cloud % self.every:
            return
        key = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        pose = self.poses.get(key)
        if pose is None or self.R_lv is None:
            return
        x, y, yaw, p_lv = pose
        P = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if P.size == 0:
            return
        Q = P.astype(np.float64) @ self.R_lv.T - p_lv
        flat = np.hypot(Q[:, 0], Q[:, 1])
        Q = Q[(Q[:, 2] > Z_MIN + FLOOR_MARGIN_PER_M * flat) & (Q[:, 2] < Z_MAX)]
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = c * Q[:, 0] - s * Q[:, 1]
        by = s * Q[:, 0] + c * Q[:, 1]
        r = np.hypot(bx, by)
        a = np.arctan2(by, bx)
        ok = (r > RANGE_MIN) & (r < RANGE_MAX)
        r, a = r[ok], a[ok]
        ranges = np.full(N_BINS, np.inf, dtype=np.float32)
        idx = ((a + math.pi) / (2 * math.pi) * N_BINS).astype(int) % N_BINS
        np.minimum.at(ranges, idx, r.astype(np.float32))
        scan = LaserScan()
        scan.header.stamp = m.header.stamp
        scan.header.frame_id = 'base_footprint'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / N_BINS
        scan.angle_max = scan.angle_min + scan.angle_increment * (N_BINS - 1)
        scan.range_min, scan.range_max = RANGE_MIN, RANGE_MAX
        scan.ranges = ranges.tolist()
        self.pub_scan.publish(scan)


def main():
    rclpy.init()
    node = LevelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
