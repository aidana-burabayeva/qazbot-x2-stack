#!/usr/bin/env python3
"""D-04 offline A/B/C/D replay harness (offline tooling, never touches the live graph).

One command (documented in README.md next to this file):
    source /opt/ros/humble/setup.bash
    nice -n 19 python3 <repo>/tests/d04/replay_d04.py --level fit --out <repo>/tests/d04/results
    (bags: env QAZBOT_D04_BAGS, see BAGS.md; repo root is resolved from __file__)

What it does
* Reads recorded /cloud_registered + LIO odometry (+ lidar IMU) straight from the
  sqlite bag files (no `ros2 bag play`, no publishers, no DDS traffic).
* A and C run the REAL node code: LevelBridge.on_imu/on_odom/on_cloud of the
  running file (A) and of the candidate (C) are called on the recorded messages
  with a stub `self` that captures the published clouds.
* B builds one PointCloud2-style observation (marking + clearing) from the same
  levelled points; D builds 3D band/clearing clouds for an approximate VoxelLayer.
* Every variant feeds the same Humble ObstacleLayer emulation (costmap_emul.py)
  with the same footprint, ranges and rolling 4x4 m @ 0.05 window.
* SYNTHETIC scenarios re-range REAL beams that would hit an injected box
  (no beam is invented: if no recorded beam reaches the box it stays unseen).
"""
import argparse
import importlib.util
import json
import math
import os
import sqlite3
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import costmap_emul as ce  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))      # repo root, resolved from this file
RUNNING = os.path.join(ROOT, 'ws/src/lio/level_bridge.py')  # baseline A == running file (sha c817a5ed...)
CANDIDATE = os.path.join(ROOT, 'docs/engineering/d04_d05_20261102/candidate/level_bridge.py')
if not os.path.exists(CANDIDATE):                           # legacy layout of the robot's control_backend
    CANDIDATE = os.path.join(ROOT, 'candidates/d04/level_bridge.py')
EXTRINSICS = os.path.join(ROOT, 'docs/evidence/x2_extrinsics.json')
# Bag locations (never inside the repo: 3.7 GB each). See BAGS.md for names, sizes and sha256.
BAGDIR = os.environ.get('QAZBOT_D04_BAGS', '/home/agi/control_backend/docs/evidence/nav_20261102_live')
LEGACY = os.environ.get('QAZBOT_D04_LEGACY', '/home/agi/control_backend')  # old (pre-live) bags only
# Effective params (P1 report: live get_parameters == /home/agi/.ros/log/qazbot_ctl_t9htg14z.yaml)
LAUNCH_RANGE_MAX = 8.0          # level_bridge range_max:=8.0
SCAN_EVERY = 2                  # level_bridge scan_every:=2
OBST_MAX, RAY_MAX = 3.0, 3.5    # obstacle_max_range, raytrace_max_range (both sources)
FOOTPRINT = np.array([(0.17, 0.229), (0.008, 0.34), (-0.182, 0.34), (-0.305, 0.229),
                      (-0.305, -0.261), (-0.182, -0.372), (0.008, -0.372), (0.17, -0.261)])
FP_PAD = 0.01
# RPP: desired_linear_vel 0.3, rotate_to_heading w 0.6, carrot 0.5, 1.5 s (D1 report)
MOTIONS = [(0.3, 0.0), (0.3, 0.3), (0.3, -0.3), (0.0, 0.6), (0.0, -0.6)]
OLD_FOV = (-46.0, 44.5)

BAGS = {
    'udp_20261028': dict(db=f'{LEGACY}/docs/evidence/i1_cloud_registered_udp_20261028_092700Z/'
                            'i1_cloud_registered_udp_20261028_092700Z_0.db3',
                         cloud='/qazbot_lio/cloud_registered', odom='/qazbot_lio/odometry',
                         imu='/aima/hal/sensor/lidar_chest_front/imu'),
    # live bags (mcap, ~3.5 GB each, recorded WITH the live scan_obstacles -> live-matched levelling)
    'static_204153': dict(db=os.path.join(BAGDIR, 'd04_static_bag_204153', 'd04_static_bag_204153_0.mcap'),
                          cloud='/qazbot_lio/cloud_registered', odom='/qazbot_lio/odometry',
                          imu='/aima/hal/sensor/lidar_chest_front/imu',
                          obstacles='/qazbot_lio/scan_obstacles'),
    'static_211835': dict(db=os.path.join(BAGDIR, 'd04_static_bag_211835', 'd04_static_bag_211835_0.mcap'),
                          cloud='/qazbot_lio/cloud_registered', odom='/qazbot_lio/odometry',
                          imu='/aima/hal/sensor/lidar_chest_front/imu',
                          obstacles='/qazbot_lio/scan_obstacles'),
    'replay_dense_01': dict(db=f'{LEGACY}/logs/replay_dense_01/bag/bag_0.db3',
                            cloud='/cloud_registered', odom='/Odometry', imu=None),
    'replay_portable_01': dict(db=f'{LEGACY}/logs/replay_portable_01/bag/bag_0.db3',
                               cloud='/cloud_registered', odom='/Odometry', imu=None),
    'replay_dense_controlled_01': dict(db=f'{LEGACY}/logs/replay_dense_controlled_01/bag/bag_0.db3',
                                       cloud='/cloud_registered', odom='/Odometry', imu=None),
}

LIVE_BAGS = ['static_204153', 'static_211835']


def bag_missing_reason(bagname):
    db = BAGS[bagname]['db']
    return None if os.path.exists(db) else f'bag {bagname} not found at {db} (set QAZBOT_D04_BAGS / QAZBOT_D04_LEGACY, see tests/d04/BAGS.md)'


def load_bridge(path, name):
    sys.dont_write_bytecode = True       # never write __pycache__ next to the running file
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.RANGE_MAX = LAUNCH_RANGE_MAX     # what the node sets from range_max:=8.0
    return mod


class _Cap:
    def __init__(self):
        self.msg = None

    def publish(self, m):
        self.msg = m


class _Log:
    def info(self, *a, **k):
        pass

    warn = warning = info


def stub_node(mod):
    """`self` for LevelBridge methods: same attributes as __init__, no rclpy."""
    s = types.SimpleNamespace(acc=[], t_acc0=None, R_lv=None, R_wb=None, poses={},
                              track=__import__('collections').deque(), last_key=None,
                              publish_tf=False, tf=None, every=1, n_cloud=0,
                              pub_scan=_Cap(), pub_clearing=_Cap(), pub_obstacles=_Cap(),
                              pub_odom=_Cap(), sub_imu=None)
    s.get_logger = lambda: _Log()
    s.destroy_subscription = lambda sub: None
    return s


def read_bag(db, topics):
    """Yield (topic, stamp_ns, msg) in bag order; sqlite3 (.db3) or mcap via rosbag2_py (read only)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    topics = set(topics)
    if db.endswith('.mcap'):
        import rosbag2_py
        rd = rosbag2_py.SequentialReader()
        rd.open(rosbag2_py.StorageOptions(uri=db, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
        cls = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types() if t.name in topics}
        while rd.has_next():
            name, data, ts = rd.read_next()
            if name in cls:
                yield name, ts, deserialize_message(data, cls[name])
        return
    c = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    tt = {r[0]: (r[1], r[2]) for r in c.execute('select id,name,type from topics')}
    ids = {i: v for i, v in tt.items() if v[0] in topics}
    cls = {i: get_message(v[1]) for i, v in ids.items()}
    q = 'select topic_id,timestamp,data from messages where topic_id in (%s) order by timestamp' % \
        ','.join(str(i) for i in ids)
    for tid, ts, data in c.execute(q):
        yield ids[tid][0], ts, deserialize_message(data, cls[tid])


# --- live-matched levelling (F1 of V1) --------------------------------------------------
def tilt_rotation(a, b):
    """Small tilt correction Rx(a) @ Ry(b) (radians); yaw-invariant."""
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    return Rx @ Ry


def bridge_obstacles(mod, R_lv, odom_msg, cloud_msg):
    """scan_obstacles xyz the bridge (module `mod`) would publish for this odom+cloud pair, or None."""
    s = stub_node(mod)
    s.R_lv = R_lv
    mod.LevelBridge.on_odom(s, odom_msg)
    s.pub_obstacles.msg = None
    mod.LevelBridge.on_cloud(s, cloud_msg)
    return None if s.pub_obstacles.msg is None else cloud_xyz(s.pub_obstacles.msg)


def _match_cost(mod, R_lv, pairs, cap=0.05):
    """Mean over pairs of mean(min(nn distance recorded->replayed in xy, cap)); cap if nothing replayed."""
    from scipy.spatial import cKDTree
    tot = 0.0
    for odom_msg, cloud_msg, rec in pairs:
        o = bridge_obstacles(mod, R_lv, odom_msg, cloud_msg)
        if o is None or len(o) == 0 or len(rec) == 0:
            tot += cap
            continue
        d, _ = cKDTree(o[:, :2]).query(rec[:, :2])
        tot += float(np.minimum(d, cap).mean())
    return tot / max(len(pairs), 1)


def fit_levelling(mod, R0, pairs, step_mrad=15.0, maxiter=150):
    """Fit the live node's R_lv = Rtilt(a,b) @ R0 so the replayed bridge reproduces RECORDED scan_obstacles.

    pairs = [(odom_msg, cloud_msg, recorded_obstacles_xyz_body)]. 2 DOF (yaw does not change
    the levelled geometry the costmap sees). Returns (R_lv, info dict).
    """
    from scipy.optimize import minimize

    def cost(x):
        return _match_cost(mod, tilt_rotation(x[0] * 1e-3, x[1] * 1e-3) @ R0, pairs)

    c0 = cost([0.0, 0.0])
    res = minimize(cost, [0.0, 0.0], method='Nelder-Mead',
                   options=dict(initial_simplex=np.array([[0, 0], [step_mrad, 0], [0, step_mrad]]),
                                xatol=0.05, fatol=1e-7, maxiter=maxiter))
    R = tilt_rotation(res.x[0] * 1e-3, res.x[1] * 1e-3) @ R0
    return R, dict(tilt_mrad=[float(res.x[0]), float(res.x[1])], cost_before_m=float(c0),
                   cost_after_m=float(res.fun), n_pairs=len(pairs),
                   tilt_deg=float(np.degrees(np.hypot(res.x[0], res.x[1]) * 1e-3)))


def load_rlv(path):
    """Explicit levelling: JSON file with a 3x3 matrix (list of lists) or {"R_lv": [[...]]}."""
    d = json.load(open(path))
    R = np.array(d['R_lv'] if isinstance(d, dict) else d, dtype=float)
    assert R.shape == (3, 3) and np.allclose(R @ R.T, np.eye(3), atol=1e-6), 'R_lv must be a rotation'
    return R


def floor_fit_up(P):
    """Gravity 'up' from the floor plane (bags without IMU). Iterative SVD fit."""
    up = np.array([1.0, 0.0, 0.0])              # FAST-LIO world x ~ up on the X2 (LiDAR IMU)
    for _ in range(3):
        h = P @ up
        fl = P[h < np.percentile(h, 1) + 0.15]
        fl = fl[np.abs(fl @ up - np.median(fl @ up)) < 0.05]
        _, _, vt = np.linalg.svd(fl - fl.mean(0), full_matrices=False)
        n = vt[2]
        up = n if n @ up > 0 else -n
    return up


def limit_clouds(n):
    """Make read_bag stop after n cloud messages of the bag being read (early stop of the mcap scan)."""
    global read_bag
    orig = read_bag

    def rb(db, topics):
        cl = [b['cloud'] for b in BAGS.values() if b['db'] == db][0]
        k = 0
        for item in orig(db, topics):
            yield item
            if item[0] == cl:
                k += 1
                if k >= n:
                    return
    read_bag = rb


def prepare(bagname, level='auto', n_fit=20):
    """Run the RUNNING node's on_imu/on_odom on the bag; return per-scan frames.

    level: 'auto' (default) = fit the live R_lv against recorded scan_obstacles when the bag has
           them, else the legacy bag levelling (IMU first 1 s / floor fit);
           'fit' = require the fit; 'bag' = legacy levelling; <path.json> = explicit 3x3 R_lv.
    NOTE: bag levelling differs from the live node's R_lv (live: levelled at its own start) by up to
    ~0.6 deg on real X2 data, so A is only 'live-matched' with 'fit' or an explicit matrix.
    """
    from sensor_msgs_py import point_cloud2 as pc2
    b = BAGS[bagname]
    A = load_bridge(RUNNING, 'lb_running_prep')
    s = stub_node(A)
    obs_topic = b.get('obstacles')
    topics = [t for t in (b['cloud'], b['odom'], b['imu'], obs_topic) if t]
    msgs = list(read_bag(b['db'], topics))
    have_obs = bool(obs_topic) and any(t == obs_topic for t, _, _ in msgs)
    level_src, info = 'imu', None
    R_fixed = None
    if level not in ('auto', 'fit', 'bag'):
        R_fixed, level_src = load_rlv(level), 'explicit'
    elif level == 'fit' and not have_obs:
        raise ValueError(f'{bagname}: no recorded scan_obstacles, cannot fit the live R_lv')
    elif level != 'bag' and have_obs:
        level_src = 'fit'
    if b['imu'] is None and R_fixed is None:
        level_src = 'fit' if level_src == 'fit' else 'floor_fit'
        first = next(m for t, _, m in msgs if t == b['cloud'])
        P0 = pc2.read_points_numpy(first, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)
        s.R_lv = A.level_rotation(floor_fit_up(P0))
    if R_fixed is not None:
        s.R_lv = R_fixed
    elif level_src == 'fit':
        # start point = legacy bag levelling (IMU), then fit tilt on matched samples
        probe = stub_node(A)
        probe.R_lv = s.R_lv
        for t, ts, m in msgs:
            if probe.R_lv is not None:
                break
            if t == b['odom']:
                A.LevelBridge.on_odom(probe, m)
            elif t == b['imu']:
                A.LevelBridge.on_imu(probe, m)
        R0 = probe.R_lv
        rec = {m.header.stamp.sec * 10**9 + m.header.stamp.nanosec: cloud_xyz(m)
               for t, _, m in msgs if t == obs_topic}
        odo, cl = {}, []
        for t, _, m in msgs:
            k = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if t == b['odom']:
                odo[k] = m
            elif t == b['cloud'] and k in rec and k in odo:
                cl.append((k, m))
        sel = cl[::max(1, len(cl) // n_fit)][:n_fit]
        pairs = [(odo[k], m, rec[k]) for k, m in sel]
        s.R_lv, info = fit_levelling(A, R0, pairs)
        level_src = 'fit'
    frames, n_cloud = [], 0
    for t, ts, m in msgs:
        if t == b['imu']:
            A.LevelBridge.on_imu(s, m)
        elif t == b['odom']:
            A.LevelBridge.on_odom(s, m)
        elif t == b['cloud']:
            n_cloud += 1
            if n_cloud % SCAN_EVERY:
                continue
            key = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            pose = s.poses.get(key)
            if pose is None or s.R_lv is None:
                continue
            frames.append(dict(msg=m, pose=pose, R_lv=s.R_lv.copy(), key=key, R_wb=s.R_wb.copy()))
    PREP_INFO[bagname] = dict(level_source=level_src, fit=info)
    return frames, level_src, A


PREP_INFO = {}


def level_points(fr):
    from sensor_msgs_py import point_cloud2 as pc2
    P = pc2.read_points_numpy(fr['msg'], field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    return P @ fr['R_lv'].T - fr['pose'][3]


def to_body(Q, yaw):
    c, s = math.cos(-yaw), math.sin(-yaw)
    return np.column_stack((c * Q[:, 0] - s * Q[:, 1], s * Q[:, 0] + c * Q[:, 1], Q[:, 2]))


def from_body(B, yaw):
    return to_body(B, -yaw)


def floor_z(frames):
    Q = level_points(frames[0])
    r = np.hypot(Q[:, 0], Q[:, 1])
    fl = Q[(r > 0.6) & (r < 3.0) & (Q[:, 2] < -0.8)]
    return float(np.median(fl[:, 2]))


# --- synthetic objects (re-ranged real beams) -------------------------------------------
def box_hit(Bp, box, fz):
    """Slab intersection of rays origin->point with an axis-aligned body-frame box.

    box = (cx, cy, hx, hy, z0, z1) with z floor-relative. Returns t (0..1) or inf.
    """
    cx, cy, hx, hy, z0, z1 = box
    lo = np.array([cx - hx, cy - hy, fz + z0])
    hi = np.array([cx + hx, cy + hy, fz + z1])
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = lo / Bp
        t2 = hi / Bp
    tmin = np.nanmax(np.where(np.isnan(np.minimum(t1, t2)), -np.inf, np.minimum(t1, t2)), axis=1)
    tmax = np.nanmin(np.where(np.isnan(np.maximum(t1, t2)), np.inf, np.maximum(t1, t2)), axis=1)
    # rays parallel to a slab and outside it never hit
    par = (Bp == 0) & ((0 < lo) | (0 > hi))
    hit = (tmax >= tmin) & (tmin > 0) & (tmin < 1.0) & ~par.any(axis=1)
    return np.where(hit, tmin, np.inf)


def inject(Q, yaw, boxes, fz):
    if not boxes:
        return Q, 0
    Bp = to_body(Q, yaw)
    t = np.full(len(Bp), np.inf)
    for bx in boxes:
        t = np.minimum(t, box_hit(Bp, bx, fz))
    m = np.isfinite(t)
    Bp[m] *= t[m, None]
    return from_body(Bp, yaw), int(m.sum())


def to_msg(fr, Q):
    from sensor_msgs_py import point_cloud2 as pc2
    P = (Q + fr['pose'][3]) @ fr['R_lv']            # inverse of Q = P R^T - p
    return pc2.create_cloud_xyz32(fr['msg'].header, P.astype(np.float32).tolist())


# --- variants ---------------------------------------------------------------------------
def cloud_xyz(m):
    from sensor_msgs_py import point_cloud2 as pc2
    if m is None or m.width * m.height == 0:
        return np.empty((0, 3))
    return pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)


def global_pose(fr):
    x, y, yaw, _ = fr['pose']
    return (float(x), float(y), float(yaw))


def body_to_global(Pb, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * Pb[:, 0] - s * Pb[:, 1], y + s * Pb[:, 0] + c * Pb[:, 1], Pb[:, 2]))


class SplitVariant:
    """A (running file) / C (candidate file): two sources, real on_cloud."""

    def __init__(self, mod, R_lv):
        self.mod = mod
        self.s = stub_node(mod)
        self.s.R_lv = R_lv

    def observations(self, fr, msg):
        s = self.s
        s.poses = {fr['key']: fr['pose']}
        s.pub_obstacles.msg = s.pub_clearing.msg = None
        t0 = time.perf_counter()
        self.mod.LevelBridge.on_cloud(s, msg)
        dt = time.perf_counter() - t0
        pose = global_pose(fr)
        org = (pose[0], pose[1], 0.0)
        ob = body_to_global(cloud_xyz(s.pub_obstacles.msg), pose)
        cl = body_to_global(cloud_xyz(s.pub_clearing.msg), pose)
        nbytes = len(s.pub_obstacles.msg.data) + len(s.pub_clearing.msg.data)
        mark = ce.Observation(ob, org, OBST_MAX, 0.0, RAY_MAX, 0.0, 0.0, 0.0)
        clear = ce.Observation(cl, org, 2.5, 0.0, RAY_MAX, 0.0, 0.0, 0.0)
        return [mark], [clear], dt, nbytes, dict(n_mark=len(ob), n_clear=len(cl))


class SingleCloudVariant:
    """B: one PointCloud2 source, marking+clearing (AgiBot-style), real 3D heights.

    Frame: base_footprint on the floor (z = Q_z - floor_z), sensor origin at the
    LiDAR IMU height. The range-dependent floor margin of the bridge cannot be
    expressed by ObstacleLayer's fixed min/max_obstacle_height, so the band is the
    bridge band evaluated at obstacle_max_range (most floor-safe fixed value
    inside the marking range): min = (Z_MIN - floor) + 0.035*3.0, max = Z_MAX - floor.
    """

    def __init__(self, mod, fz):
        self.mod, self.fz = mod, fz
        self.hmin = (mod.Z_MIN - fz) + mod.FLOOR_MARGIN_PER_M * OBST_MAX
        self.hmax = mod.Z_MAX - fz

    def observations(self, fr, Q):
        t0 = time.perf_counter()
        pose = global_pose(fr)
        Bp = to_body(Q, pose[2])
        r = np.hypot(Bp[:, 0], Bp[:, 1])
        Bp = Bp[(r > self.mod.RANGE_MIN) & (r < LAUNCH_RANGE_MAX)]
        Bp[:, 2] -= self.fz
        G = body_to_global(Bp, pose)
        o = ce.Observation(G, (pose[0], pose[1], -self.fz), OBST_MAX, 0.0, RAY_MAX, 0.0,
                           self.hmin, self.hmax)
        dt = time.perf_counter() - t0
        # wire size: AgiBot-style publishes the whole in-range cloud (xyz float32); the
        # height filter runs inside the costmap's ObservationBuffer.
        return [o], [o], dt, len(Bp) * 12, dict(n_mark=len(o.points), n_clear=len(o.points))


class VoxelApprox:
    """D: approximate nav2 VoxelLayer (mark_threshold 0): 3D band marks, 3D ray clears.

    APPROX: rays are sampled every min(res, z_res)/4 instead of VoxelGrid's
    integer 3D Bresenham; the 2D cell is FREE when its column has no marked voxel.
    """

    def __init__(self, mod, fz, n=80, res=0.05, z_res=0.12, nz=16):
        self.mod, self.fz, self.n, self.res, self.zr, self.nz = mod, fz, n, res, z_res, nz
        self.vox = np.zeros((n, n, nz), bool)
        self.layer = ce.ObstacleLayer(4.0, res, 2.0, FOOTPRINT, FP_PAD)

    def step(self, fr, Q):
        L = self.layer
        pose = global_pose(fr)
        t0 = time.perf_counter()
        old = (L.ox, L.oy)
        L.update_origin(pose[0], pose[1])
        if old[0] is not None and (old[0] != L.ox or old[1] != L.oy):
            cx, cy = int(round((L.ox - old[0]) / self.res)), int(round((L.oy - old[1]) / self.res))
            v = np.zeros_like(self.vox)
            n = self.n
            sx0, sx1, sy0, sy1 = max(0, cx), min(n, n + cx), max(0, cy), min(n, n + cy)
            if sx0 < sx1 and sy0 < sy1:
                v[sy0 - cy:sy1 - cy, sx0 - cx:sx1 - cx] = self.vox[sy0:sy1, sx0:sx1]
            self.vox = v
        Bp = to_body(Q, pose[2])
        r = np.hypot(Bp[:, 0], Bp[:, 1])
        Bp = Bp[(r > self.mod.RANGE_MIN) & (r < LAUNCH_RANGE_MAX)]
        r = np.hypot(Bp[:, 0], Bp[:, 1])
        band = (Bp[:, 2] > self.mod.Z_MIN + self.mod.FLOOR_MARGIN_PER_M * r) & (Bp[:, 2] < self.mod.Z_MAX)
        Bp[:, 2] -= self.fz
        G = body_to_global(Bp, pose)
        o = np.array([pose[0], pose[1], -self.fz])
        # clearing: all points, 3D, clipped to the grid's z range and RAY_MAX
        d = G - o
        L3 = np.linalg.norm(d, axis=1)
        tmax = np.minimum(1.0, RAY_MAX / L3)
        with np.errstate(divide='ignore', invalid='ignore'):
            tz = np.where(d[:, 2] < 0, (0.0 - o[2]) / d[:, 2], (self.nz * self.zr - 0.01 - o[2]) / d[:, 2])
        tmax = np.minimum(tmax, np.where(np.isfinite(tz) & (tz > 0), tz, 1.0))
        step = min(self.res, self.zr) / 4
        ns = int(math.ceil(RAY_MAX / step)) + 1
        ts = np.linspace(0, 1, ns)[None, :] * (tmax * L3)[:, None] / L3[:, None]
        sx = o[0] + ts * d[:, 0:1]
        sy = o[1] + ts * d[:, 1:2]
        sz = o[2] + ts * d[:, 2:3]
        mx = np.floor((sx - L.ox) / self.res).astype(np.int64)
        my = np.floor((sy - L.oy) / self.res).astype(np.int64)
        mz = np.floor(sz / self.zr).astype(np.int64)
        ok = (mx >= 0) & (my >= 0) & (mx < self.n) & (my < self.n) & (mz >= 0) & (mz < self.nz)
        self.vox[my[ok], mx[ok], mz[ok]] = False
        touched = np.zeros((self.n, self.n), bool)
        touched[my[ok], mx[ok]] = True
        L.g[touched & ~self.vox.any(axis=2)] = ce.FREE
        # marking: band points within obstacle range (3D distance)
        M = G[band]
        dd = np.linalg.norm(M - o, axis=1)
        M = M[dd < OBST_MAX]
        mx = np.floor((M[:, 0] - L.ox) / self.res).astype(np.int64)
        my = np.floor((M[:, 1] - L.oy) / self.res).astype(np.int64)
        mz = np.floor(M[:, 2] / self.zr).astype(np.int64)
        ok = (mx >= 0) & (my >= 0) & (mx < self.n) & (my < self.n) & (mz >= 0) & (mz < self.nz)
        self.vox[my[ok], mx[ok], mz[ok]] = True
        L.g[my[ok], mx[ok]] = ce.LETHAL
        fx, fy = L.footprint_cells(pose)
        L.g[fy, fx] = ce.FREE
        self.vox[fy, fx, :] = False
        dt = time.perf_counter() - t0
        return dt, int(band.sum()) * 12 + len(G) * 12


# --- scenarios --------------------------------------------------------------------------
def box_at(bearing_deg, rng, w=0.3, z0=0.0, z1=1.2):
    a = math.radians(bearing_deg)
    return (rng * math.cos(a), rng * math.sin(a), w / 2, w / 2, z0, z1)


def scenarios(nf):
    """name -> (list of (first, last_excl, box)), seed_cells, label."""
    on, off = 5, 15                      # 2 s present (10 updates), then gone
    sc = {
        'recorded_only': ([], None, 'RECORDED'),
        'S1_front_0deg_0.75m': ([(on, off, box_at(0, 0.75))], None, 'SYNTHETIC'),
        'S2_left_+49deg_0.8m': ([(on, off, box_at(49, 0.8))], None, 'SYNTHETIC'),
        'S3_right_-55deg_0.65m': ([(on, off, box_at(-55, 0.65))], None, 'SYNTHETIC'),
        'S4_low_0deg_0.8m_h0.25': ([(on, nf, box_at(0, 0.8, 0.4, 0.0, 0.25))], None, 'SYNTHETIC'),
        'S5_tall_overhang_0deg_0.6m_z1.0-1.7': ([(on, nf, box_at(0, 0.6, 0.4, 1.0, 1.7))], None, 'SYNTHETIC'),
        'S6_side_chair_+49deg_0.8m_static': ([(on, nf, box_at(49, 0.8, 0.45, 0.0, 0.9))], None, 'SYNTHETIC'),
        'S7_person_leaves_into_blind': (
            [(on + i, on + i + 1, box_at(30 + 5 * i, 0.7, 0.35, 0.0, 1.7)) for i in range(10)], None,
            'SYNTHETIC'),
        'S9_D04_right_-52deg_0.5m_transient': ([(on, off, box_at(-52, 0.62, 0.25, 0.0, 1.7))], None,
                                               'SYNTHETIC'),
        'S10_D04_left_+48deg_0.55m_transient': ([(on, off, box_at(48, 0.67, 0.25, 0.0, 1.7))], None,
                                                'SYNTHETIC'),
        'S8_blind_seed_absence_not_free': ([], [(65, 0.8), (75, 0.7), (-68, 0.8), (-80, 1.0), (180, 0.8),
                                                (120, 1.2)], 'SYNTHETIC'),
    }
    return sc


def bearing_range(xy, pose):
    x, y, th = pose
    dx, dy = xy[:, 0] - x, xy[:, 1] - y
    b = np.degrees(np.arctan2(dy, dx) - th)
    b = (b + 180) % 360 - 180
    return b, np.hypot(dx, dy)


def region_cells(layer, pose, box, dil=0.05):
    """Lethal count inside the box footprint (global), dilated by one cell."""
    cx, cy, hx, hy, _, _ = box
    P = layer.lethal_world()
    if len(P) == 0:
        return 0
    x, y, th = pose
    c, s = math.cos(-th), math.sin(-th)
    bx = c * (P[:, 0] - x) - s * (P[:, 1] - y)
    by = s * (P[:, 0] - x) + c * (P[:, 1] - y)
    return int(((np.abs(bx - cx) <= hx + dil) & (np.abs(by - cy) <= hy + dil)).sum())


def run_scenario(frames, fz, name, spec, A, C, variants=('A', 'B', 'C', 'D'), collide=True):
    evs, seed, label = spec
    nf = len(frames)
    R_lv = frames[0]['R_lv']
    split = {'A': SplitVariant(A, R_lv), 'C': SplitVariant(C, R_lv)}
    single = SingleCloudVariant(A, fz)
    layers = {v: ce.ObstacleLayer(4.0, 0.05, 2.0, FOOTPRINT, FP_PAD) for v in ('A', 'B', 'C')}
    vox = VoxelApprox(A, fz)
    layers['D'] = vox.layer
    fp = ce.pad_footprint(FOOTPRINT, FP_PAD)
    out = {v: dict(t_bridge=[], t_layer=[], bytes=[], lethal=[], edge_lethal=[], coll=[], region=[],
                   seed_kept=None, n_mark=[], n_clear=[]) for v in variants}
    p0 = global_pose(frames[0])
    seed_xy = None
    if seed:
        seed_xy = np.array([[p0[0] + r * math.cos(p0[2] + math.radians(b)),
                             p0[1] + r * math.sin(p0[2] + math.radians(b))] for b, r in seed])
    n_inj = []
    for k, fr in enumerate(frames):
        boxes = [b for (f0, f1, b) in evs if f0 <= k < f1]
        Q = level_points(fr)
        Qi, ni = inject(Q, fr['pose'][2], boxes, fz)
        n_inj.append(ni)
        msg = fr['msg'] if not boxes else to_msg(fr, Qi)
        pose = global_pose(fr)
        for v in variants:
            L = layers[v]
            if k == 0 and seed_xy is not None:
                L.update_origin(pose[0], pose[1])
                mx, my, ok = L.world_to_map(seed_xy[:, 0], seed_xy[:, 1])
                L.g[my[ok], mx[ok]] = ce.LETHAL
                if v == 'D':
                    vox.vox[my[ok], mx[ok], 5] = True
            if v in ('A', 'C'):
                mk, cl, dtb, nb, info = split[v].observations(fr, msg)
            elif v == 'B':
                mk, cl, dtb, nb, info = single.observations(fr, Qi)
            if v == 'D':
                dtl, nb = vox.step(fr, Qi)
                dtb, info = 0.0, dict(n_mark=0, n_clear=0)
            else:
                t0 = time.perf_counter()
                L.update(pose, mk, cl)
                dtl = time.perf_counter() - t0
            o = out[v]
            o['t_bridge'].append(dtb); o['t_layer'].append(dtl); o['bytes'].append(nb)
            o['n_mark'].append(info['n_mark']); o['n_clear'].append(info['n_clear'])
            lw = L.lethal_world()
            b, r = bearing_range(lw, pose) if len(lw) else (np.empty(0), np.empty(0))
            o['lethal'].append(int(len(lw)))
            o['edge_lethal'].append(int((((b > OLD_FOV[1]) | (b < OLD_FOV[0])) & (np.abs(b) <= 90) &
                                         (r < 1.5)).sum()))
            if collide:
                o['coll'].append([bool(ce.rpp_collision_imminent(L, fp, pose, vv, ww)) for vv, ww in MOTIONS])
            if evs:
                o['region'].append([region_cells(L, pose, bx) for (_, _, bx) in evs])
            if k == nf - 1:
                o['final_cells'] = [[round(float(a), 3), round(float(b_), 3)] for a, b_ in lw]
            if seed_xy is not None and k == nf - 1:
                mx, my, ok = L.world_to_map(seed_xy[:, 0], seed_xy[:, 1])
                o['seed_kept'] = [bool(L.g[my[i], mx[i]] == ce.LETHAL) if ok[i] else None
                                  for i in range(len(seed_xy))]
    return dict(label=label, n_frames=nf, n_injected=n_inj, events=[(a, b_, list(bx)) for a, b_, bx in evs],
                seed=seed, out=out)


# --- coverage / FOV analysis -------------------------------------------------------------
def coverage(frames, fz, A):
    """Observed bearing span of all points and of in-band points per flat-range band,
    plus the LiDAR attitude in the level frame (why the edges are asymmetric)."""
    ext = json.load(open(EXTRINSICS))
    R_il = np.array(ext['R_imu_lidar'])
    res = {}
    allb, bandb = {}, {}
    bands = [(0.45, 1.0), (1.0, 2.0), (2.0, 3.5), (3.5, 8.0)]
    for fr in frames:
        Q = level_points(fr)
        Bp = to_body(Q, fr['pose'][2])
        r = np.hypot(Bp[:, 0], Bp[:, 1])
        a = np.degrees(np.arctan2(Bp[:, 1], Bp[:, 0]))
        inb = (Bp[:, 2] > A.Z_MIN + A.FLOOR_MARGIN_PER_M * r) & (Bp[:, 2] < A.Z_MAX)
        for lo, hi in bands:
            m = (r > lo) & (r <= hi)
            allb.setdefault((lo, hi), []).append(a[m])
            bandb.setdefault((lo, hi), []).append(a[m & inb])
    for (lo, hi) in bands:
        aa = np.concatenate(allb[(lo, hi)])
        bb = np.concatenate(bandb[(lo, hi)])
        q = lambda x: [round(float(np.min(x)), 2), round(float(np.max(x)), 2),
                       round(float(np.percentile(x, 0.5)), 2), round(float(np.percentile(x, 99.5)), 2)] \
            if len(x) else None
        res[f'{lo}-{hi}m'] = dict(all_min_max_p05_p995=q(aa), band_min_max_p05_p995=q(bb),
                                  n_all=int(len(aa)), n_band=int(len(bb)),
                                  band_outside_old_fov=int(((bb < OLD_FOV[0]) | (bb > OLD_FOV[1])).sum()))
    # LiDAR axes in the level, yaw-removed body frame
    fr = frames[0]
    # R_wb is not stored; recompute from yaw-free definitions via the points instead:
    res['lidar_imu_offset_m'] = float(np.linalg.norm(ext['T_imu_lidar']))
    res['floor_z_rel_imu'] = fz
    return res


def lidar_attitude(frames, A):
    """Pitch/roll of the LiDAR in the gravity-levelled frame (odometry attitude at the first scan)."""
    ext = json.load(open(EXTRINSICS))
    R_il = np.array(ext['R_imu_lidar'])
    R_wb = frames[0]['R_wb']
    R = frames[0]['R_lv'] @ R_wb @ R_il            # lidar axes in level frame
    yaw = frames[0]['pose'][2]
    c, s_ = math.cos(-yaw), math.sin(-yaw)
    Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
    Rb = Rz @ R                                    # in yaw-removed body frame
    x, y, z = Rb[:, 0], Rb[:, 1], Rb[:, 2]
    pitch = math.degrees(math.asin(-x[2]))
    roll = math.degrees(math.atan2(y[2], z[2]))
    # The X2 chest LiDAR is mounted rolled ~-90 deg (AgiBot pcl_rectifier rotates roll -90);
    # the residual about -90 is what tilts its FOV boundary sideways.
    return dict(lidar_pitch_down_deg=round(pitch, 2), lidar_roll_deg=round(roll, 2),
                lidar_roll_residual_from_minus90_deg=round(roll + 90.0, 2))


def summarize(res):
    """Compact per-scenario/variant metrics."""
    S = {}
    for bag, scs in res.items():
        for name, r in scs.items():
            if not isinstance(r, dict) or 'out' not in r:
                continue
            ev = r['events']
            last_off = max((e[1] for e in ev), default=None)
            for v, o in r['out'].items():
                coll = np.array(o['coll']) if o['coll'] else None
                d = dict(lethal_final=o['lethal'][-1], edge_lethal_final=o['edge_lethal'][-1],
                         edge_lethal_max=max(o['edge_lethal']),
                         t_bridge_ms_p50=1e3 * float(np.median(o['t_bridge'])),
                         t_bridge_ms_p95=1e3 * float(np.percentile(o['t_bridge'], 95)),
                         t_bridge_ms_p99=1e3 * float(np.percentile(o['t_bridge'], 99)),
                         t_layer_ms_p50=1e3 * float(np.median(o['t_layer'])),
                         bytes_per_update=float(np.mean(o['bytes'])),
                         n_clear_mean=float(np.mean(o['n_clear'])), n_mark_mean=float(np.mean(o['n_mark'])))
                if coll is not None:
                    d['coll_any_frames'] = int(coll.any(axis=1).sum())
                    d['coll_straight_frames'] = int(coll[:, 0].sum())
                if o['region']:
                    reg = np.array(o['region'])
                    nf = len(reg)
                    if last_off is not None and last_off < nf:
                        after = reg[last_off:].sum(axis=1)
                        d['region_lethal_after_leave_final'] = int(after[-1])
                        d['region_lethal_after_leave_first'] = int(after[0])
                        clear_idx = np.nonzero(after == 0)[0]
                        d['updates_to_clear_after_leave'] = int(clear_idx[0]) if len(clear_idx) else None
                        if coll is not None:
                            d['coll_any_after_leave_final'] = bool(coll[-1].any())
                            d['coll_any_frames_after_leave'] = int(coll[last_off:].any(axis=1).sum())
                    pres = [reg[k, i] for i, e in enumerate(ev) for k in range(e[0], min(e[1], nf))]
                    d['region_lethal_while_present_min'] = int(min(pres)) if pres else None
                    d['region_present_updates_with_mark'] = int(sum(1 for p in pres if p > 0))
                    d['region_present_updates'] = len(pres)
                    if coll is not None:
                        pc = [bool(coll[k].any()) for e in ev for k in range(e[0], min(e[1], nf))]
                        d['coll_any_while_present'] = int(sum(pc))
                if v != 'A' and 'A' in r['out']:
                    fa = {tuple(c) for c in r['out']['A']['final_cells']}
                    fv = {tuple(c) for c in o['final_cells']}
                    d['final_cells_only_in_A'] = len(fa - fv)
                    d['final_cells_only_in_variant'] = len(fv - fa)
                if o['seed_kept'] is not None:
                    d['seed_kept'] = o['seed_kept']
                S.setdefault(bag, {}).setdefault(name, {})[v] = d
    return S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'results'))
    ap.add_argument('--bags', nargs='*', default=LIVE_BAGS,
                    help='default: the two registered live static bags; legacy bags: %s' % ', '.join(b for b in BAGS if b not in LIVE_BAGS))
    ap.add_argument('--scenarios', nargs='*', default=None)
    ap.add_argument('--max-clouds', type=int, default=None,
                    help='read only the first N cloud messages of each bag (10 Hz; 400 = 40 s). Default: whole bag')
    ap.add_argument('--no-collision', action='store_true')
    ap.add_argument('--level', default='auto',
                    help="R_lv source: auto (fit vs recorded scan_obstacles if in bag, default), fit, bag, or a JSON 3x3 file")
    args = ap.parse_args()
    if args.max_clouds:
        limit_clouds(args.max_clouds)
    for b in args.bags:
        why = bag_missing_reason(b)
        if why:
            sys.exit('ERROR: ' + why)
    os.makedirs(args.out, exist_ok=True)
    A = load_bridge(RUNNING, 'lb_running')
    C = load_bridge(CANDIDATE, 'lb_candidate')
    import hashlib
    meta = dict(max_clouds=args.max_clouds, level=args.level, running_sha256=hashlib.sha256(open(RUNNING, 'rb').read()).hexdigest(),
                candidate_sha256=hashlib.sha256(open(CANDIDATE, 'rb').read()).hexdigest(),
                bags={}, coverage={}, lidar_attitude={})
    res = {}
    t_all = time.time()
    for bag in args.bags:
        frames, lsrc, _ = prepare(bag, args.level)
        fz = floor_z(frames)
        meta['bags'][bag] = dict(db=BAGS[bag]['db'], frames=len(frames), level_source=lsrc,
                                 levelling=PREP_INFO.get(bag), floor_z=fz)
        meta['coverage'][bag] = coverage(frames, fz, A)
        try:
            meta['lidar_attitude'][bag] = lidar_attitude(frames, A)
        except Exception as e:                   # noqa: BLE001 (report, never hide)
            meta['lidar_attitude'][bag] = f'ERROR {e!r}'
        sc = scenarios(len(frames))
        names = args.scenarios or list(sc)
        res[bag] = {}
        for nm in names:
            t0 = time.time()
            res[bag][nm] = run_scenario(frames, fz, nm, sc[nm], A, C, collide=not args.no_collision)
            print(f'{bag} {nm} {time.time() - t0:.1f}s', flush=True)
    meta['wall_s'] = time.time() - t_all
    S = summarize(res)
    json.dump(dict(meta=meta, summary=S), open(os.path.join(args.out, 'summary.json'), 'w'), indent=1)
    json.dump(res, open(os.path.join(args.out, 'raw.json'), 'w'))
    print(json.dumps(dict(meta=meta), indent=1)[:4000])


if __name__ == '__main__':
    main()
