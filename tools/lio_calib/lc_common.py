"""lio_calib common: closed-bag reading, frames, stationary windows.

Frames (from ws/src/lio/x2.yaml + level_bridge.py):
  L = lidar_chest_front (raw cloud frame), B = lidar_imu_chest_front = FAST-LIO body,
  W = FAST-LIO world (camera_init, first IMU frame; its x is ~up, NOT level).
  p_B = R_BL p_L + t_BL   (x2.yaml mapping.extrinsic_R / extrinsic_T = T_imu_lidar).
  odom_level/base_footprint = IMU position levelled, yaw = heading of L's x axis on the floor.
Time: FAST-LIO odometry stamp = cloud header stamp + last per-point offset (lidar_end_time);
  verified on manual_amcl_walk_02 (odom - header = 0.095 s = max point 'timestamp' field).
Read-only. Refuses bags without metadata.yaml or whose .db3 is open by any process
(never read a recorder's open SQLite).
"""
import math
import os
import glob

import numpy as np
import yaml

CLOUD_TOPIC = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
IMU_TOPIC = '/aima/hal/sensor/lidar_chest_front/imu'
ODOM_TOPIC = '/qazbot_lio/odometry'
LEVEL_TOPIC = '/qazbot_lio/odom_level'
X2_YAML = '/home/agi/control_backend/ws/src/lio/x2.yaml'


def extrinsic(path=X2_YAML):
    d = yaml.safe_load(open(path))['/**']['ros__parameters']['mapping']
    R = np.array(d['extrinsic_R'], float).reshape(3, 3)
    t = np.array(d['extrinsic_T'], float)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T                                   # T_BL


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a > math.pi - 1e-6:                     # near pi: use the symmetric part
        w, V = np.linalg.eigh((R + R.T) / 2)
        ax = V[:, np.argmax(w)]
        return ax * a * (1 if ax @ v >= 0 else -1)
    return v / (2 * math.sin(a)) * a


def exp_so3(w):
    a = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if a < 1e-12:
        return np.eye(3) + K
    return np.eye(3) + math.sin(a) / a * K + (1 - math.cos(a)) / a ** 2 * K @ K


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------ bag access
def assert_closed(bag):
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        raise SystemExit(f'{bag}: no metadata.yaml (recording still open or crashed) -> refuse')
    dbs = {os.path.realpath(p) for p in glob.glob(os.path.join(bag, '*.db3')) + glob.glob(os.path.join(bag, '*.mcap'))}
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.path.realpath(fd) in dbs or os.readlink(fd).split(' (')[0] in dbs:
                raise SystemExit(f'{bag}: storage file open by pid {fd.split("/")[2]} -> refuse (never read an open recorder db)')
        except OSError:
            pass


def reader(bag, topics):
    import rosbag2_py
    assert_closed(bag)
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta.get('storage_identifier', 'sqlite3')),
           rosbag2_py.ConverterOptions('', ''))
    have = {t['topic_metadata']['name'] for t in meta['topics_with_message_count']}
    topics = [t for t in topics if t in have]
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r, have


def load_odom(bag, topic=ODOM_TOPIC):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r, have = reader(bag, [topic])
    if topic not in have:
        return None
    T, P, Q = [], [], []
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, Odometry)
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        P.append((p.x, p.y, p.z))
        Q.append((q.x, q.y, q.z, q.w))
    T, P, Q = np.array(T), np.array(P), np.array(Q)
    o = np.argsort(T, kind='stable')
    return dict(t=T[o], p=P[o], q=Q[o])


def load_imu(bag):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    r, have = reader(bag, [IMU_TOPIC])
    if IMU_TOPIC not in have:
        return None
    T, A = [], []
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, Imu)
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        a = m.linear_acceleration
        A.append((a.x, a.y, a.z))
    return dict(t=np.array(T), a=np.array(A))


def cloud_xyz(m, rmin=1.0, rmax=40.0):
    """x,y,z float32 at offsets 0/4/8 (verified: AIMA cloud fields x,y,z,intensity,ring,timestamp(f64 ns rel))."""
    names = [f.name for f in m.fields]
    off = {f.name: f.offset for f in m.fields}
    assert names[:3] == ['x', 'y', 'z'] and off['x'] == 0 and off['y'] == 4 and off['z'] == 8, names
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
    P = a[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
    tmax = 0.0
    if 'timestamp' in off:
        o = off['timestamp']
        tt = a[:, o:o + 8].copy().view(np.float64).ravel()
        tt = tt[np.isfinite(tt)]
        tmax = float(tt.max()) * 1e-9 if tt.size else 0.0
    P = P[np.isfinite(P).all(1)]
    r = np.linalg.norm(P, axis=1)
    return P[(r > rmin) & (r < rmax)], tmax


def load_clouds(bag, want_stamps, tol=0.006):
    """Clouds whose END stamp (header + max point offset, = FAST-LIO odometry stamp) is in want_stamps."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    r, have = reader(bag, [CLOUD_TOPIC])
    if CLOUD_TOPIC not in have:
        raise SystemExit(f'{bag}: no {CLOUD_TOPIC}')
    want = np.sort(np.asarray(want_stamps, float))
    out = {}
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, PointCloud2)
        th = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # cheap pre-check before decoding (end stamp is header + <=0.11 s)
        i = np.searchsorted(want, th - tol)
        if i >= len(want) or want[i] > th + 0.12 + tol:
            continue
        P, tmax = cloud_xyz(m)
        te = th + tmax
        j = np.argmin(np.abs(want - te))
        if abs(want[j] - te) <= tol:
            out[float(want[j])] = P
    return out


# --------------------------------------------------------------- odometry tools
def poses(od):
    """4x4 T_WB per odometry sample."""
    T = np.repeat(np.eye(4)[None], len(od['t']), 0)
    for k, (q, p) in enumerate(zip(od['q'], od['p'])):
        T[k, :3, :3] = quat_to_R(*q)
        T[k, :3, 3] = p
    return T


def stationary_windows(od, v_max=0.04, w_max=0.04, base=0.5, min_dur=2.0, trim=0.3):
    """Windows where the LIO pose moves < v_max m/s and < w_max rad/s over a `base` s baseline.
    Thresholds are above measured standing sway (1-2.5 cm per 1.5 s, locA-data seg.py) -- EXPERIMENTAL."""
    t, Tw = od['t'], poses(od)
    n = len(t)
    still = np.zeros(n, bool)
    j = 0
    for i in range(n):
        while j < n - 1 and t[j] < t[i] + base:
            j += 1
        dt = t[j] - t[i]
        if dt < 0.8 * base:
            still[i] = still[i - 1] if i else False
            continue
        v = np.linalg.norm(Tw[j, :3, 3] - Tw[i, :3, 3]) / dt
        w = np.linalg.norm(rotvec(Tw[i, :3, :3].T @ Tw[j, :3, :3])) / dt
        still[i] = v < v_max and w < w_max
    # the baseline looks ahead: a sample is still only if the base window starting there is still
    wins, i = [], 0
    while i < n:
        if still[i]:
            k = i
            while k + 1 < n and still[k + 1] and t[k + 1] - t[k] < 0.5:
                k += 1
            a, b = t[i] + trim, min(t[k] + base, t[-1]) - trim   # sample k's baseline ends at t[k]+base
            if b - a >= min_dur:
                wins.append((a, b))
            i = k + 1
        else:
            i += 1
    return wins


def up_in_B(imu, a, b):
    """Gravity 'up' in the body (LiDAR-IMU) frame from the mean specific force over [a,b] (robot standing)."""
    if imu is None:
        return None
    m = (imu['t'] >= a) & (imu['t'] <= b)
    if m.sum() < 50:
        return None
    g = imu['a'][m].mean(0)
    return g / np.linalg.norm(g)


def level_yaw_track(lv):
    """odom_level -> t, x, y, unwrapped yaw."""
    q = lv['q']
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    return lv['t'], lv['p'][:, 0], lv['p'][:, 1], np.unwrap(yaw)


def amcl_update_replay(t, x, y, yaw, a, b, d_thresh=0.25, a_thresh=0.2):
    """Replay nav2_amcl AmclNode::shouldUpdateFilter (amcl_node.cpp:768-777, per-axis |dx|,|dy| > d, |dyaw| > a,
    odom-frame deltas) over [a,b]. Returns per-update (delta_trans, delta_rot) and the un-applied remainder."""
    m = (t >= a) & (t <= b)
    xs, ys, ws = x[m], y[m], yaw[m]
    if xs.size < 2:
        return [], (0.0, 0.0)
    ref = (xs[0], ys[0], ws[0])
    ups = []
    for k in range(1, xs.size):
        dx, dy, dw = xs[k] - ref[0], ys[k] - ref[1], wrap(ws[k] - ref[2])
        if abs(dx) > d_thresh or abs(dy) > d_thresh or abs(dw) > a_thresh:
            ups.append((math.hypot(dx, dy), dw))
            ref = (xs[k], ys[k], ws[k])
    rem = (math.hypot(xs[-1] - ref[0], ys[-1] - ref[1]), wrap(ws[-1] - ref[2]))
    return ups, rem
