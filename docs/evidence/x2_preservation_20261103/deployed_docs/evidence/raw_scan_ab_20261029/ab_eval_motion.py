"""A/B in motion: aggregates only scans received while the robot moves (native odom |v|>0.05 m/s or |wz|>0.1 rad/s,
or motion_adapter counters.published increased in the last 1 s). Online/streaming (bounded memory), read-only.
Map pose at scan stamp = AMCL pose (at its stamp) composed with native-odom delta to the scan stamp.
Usage: ab_eval_motion.py DURATION_S OUT_JSON PID_FASTLIO PID_LEVELBRIDGE PID_RAW [PID_RAW_NODESKEW]"""
import json, math, struct, sys, time, os
from collections import deque
import numpy as np, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from scipy.ndimage import distance_transform_edt
DUR, OUT = float(sys.argv[1]), sys.argv[2]
PIDS = dict(zip(['fastlio', 'level_bridge', 'raw_scan_bridge', 'raw_nodeskew'], [int(x) for x in sys.argv[3:]]))
TOP = {'lio': '/qazbot_lio/scan', 'raw': '/qazbot_raw/scan', 'raw_low': '/qazbot_raw/scan_low', 'raw_nodeskew': '/qazbot_raw/nodeskew/scan'}
ANG = -math.pi + np.arange(1440) * 2 * math.pi / 1440
rclpy.init(); n = rclpy.create_node('qazbot_raw_ab_eval_motion')
odom = deque(maxlen=400); st = dict(dist=None, amcl=None, pub=None, pub_t=0.0)
agg = {c: {k: dict(lat=[], fin=[], m1=[], m2=[]) for k in TOP} for c in ('moving', 'standing')}
pairs = {c: {k: dict(d=[], both=[], only_l=[], only_r=[], off=[]) for k in list(TOP)[1:] + ['nodeskew_vs_raw']} for c in ('moving', 'standing')}
ns_buf = deque(maxlen=30)
lio_buf = deque(maxlen=30); cloud_lat = []; speeds = []
def tstamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def on_odom(m):
    q = m.pose.pose.orientation; v = m.twist.twist
    odom.append((tstamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w), math.hypot(v.linear.x, v.linear.y), abs(v.angular.z), time.time()))
def odom_at(t):
    if not odom: return None
    a = min(odom, key=lambda o: abs(o[0] - t))
    return a if abs(a[0] - t) < 0.1 else None
def on_status(m):
    try: p = json.loads(m.data)['counters']['published']
    except Exception: return
    if st['pub'] is not None and p > st['pub']: st['pub_t'] = time.time()
    st['pub'] = p
def moving():
    if time.time() - st['pub_t'] < 1.0: return True
    if odom and time.time() - odom[-1][6] < 0.5: return odom[-1][4] > 0.05 or odom[-1][5] > 0.1
    return False
def on_map(m):
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    st['dist'] = (distance_transform_edt(~(g >= 65)) * m.info.resolution, m.info)
def on_amcl(m):
    q = m.pose.pose.orientation
    ta = tstamp(m.header); oa = odom_at(ta)
    if oa is None and odom and odom[-1][4] < 0.02 and odom[-1][5] < 0.05: oa = odom[-1]   # standing: stale AMCL stamp is fine
    if oa is not None: st['amcl'] = (ta, m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w), oa)
def map_pose(ts):
    if st['amcl'] is None: return None
    ta, x, y, yaw, oa = st['amcl']; ob = odom_at(ts)
    if oa is None or ob is None: return None
    dx, dy = ob[1] - oa[1], ob[2] - oa[2]; c, s = math.cos(-oa[3]), math.sin(-oa[3])
    lx, ly = c * dx - s * dy, s * dx + c * dy; c2, s2 = math.cos(yaw), math.sin(yaw)
    return x + c2 * lx - s2 * ly, y + s2 * lx + c2 * ly, yaw + (ob[3] - oa[3])
def score(r, pose):
    dist, info = st['dist']; f = np.isfinite(r)
    if f.sum() < 10: return None
    x, y, yaw = pose; mx = x + r[f] * np.cos(ANG[f] + yaw); my = y + r[f] * np.sin(ANG[f] + yaw)
    ix = ((mx - info.origin.position.x) / info.resolution).astype(int); iy = ((my - info.origin.position.y) / info.resolution).astype(int)
    ins = (ix >= 0) & (ix < info.width) & (iy >= 0) & (iy < info.height)
    dd = np.full(len(mx), np.inf); dd[ins] = dist[iy[ins], ix[ins]]
    return float((dd <= 0.1).mean()), float((dd <= 0.2).mean())
def mk(k):
    def cb(m):
        now = time.time(); t = tstamp(m.header); c = 'moving' if moving() else 'standing'
        r = np.array(m.ranges, dtype=np.float32); r[(r < m.range_min) | (r > m.range_max)] = np.inf
        a = agg[c][k]; a['lat'].append((now - t) * 1e3); a['fin'].append(int(np.isfinite(r).sum()))
        if c == 'moving' and odom: speeds.append((odom[-1][4], odom[-1][5]))
        if st['dist'] is not None:
            p = map_pose(t); s = score(r, p) if p else None
            if s: a['m1'].append(s[0]); a['m2'].append(s[1])
        if k == 'lio': lio_buf.append((t, r)); return
        if k == 'raw_nodeskew': ns_buf.append((t, r))
        if k == 'raw':   # same cloud, deskew on vs off (direct deskew effect); 'lio' side = no-deskew here
            nb = min(ns_buf, key=lambda x: abs(x[0] - t), default=None)
            if nb is not None and abs(nb[0] - t) < 0.002:
                P = pairs[c]['nodeskew_vs_raw']; P['off'].append(0.0); rl = nb[1]; fl, fr = np.isfinite(rl), np.isfinite(r); mm = fl & fr
                P['d'].extend(np.abs(rl[mm] - r[mm]).round(3).tolist()); P['both'].append(int(mm.sum()))
                P['only_l'].append(int((fl & ~fr).sum())); P['only_r'].append(int((fr & ~fl).sum()))
        best = min(lio_buf, key=lambda x: abs(x[0] - t), default=None)
        if best is None or abs(best[0] - t) > 0.05: return
        P = pairs[c][k]; P['off'].append((best[0] - t) * 1e3)
        rl = best[1]; fl, fr = np.isfinite(rl), np.isfinite(r); mm = fl & fr
        P['d'].extend(np.abs(rl[mm] - r[mm]).round(3).tolist()); P['both'].append(int(mm.sum()))
        P['only_l'].append(int((fl & ~fr).sum())); P['only_r'].append(int((fr & ~fl).sum()))
    return cb
for k, t in TOP.items(): n.create_subscription(LaserScan, t, mk(k), 20)
n.create_subscription(PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    lambda b: cloud_lat.append(time.time() - sum(a * b_ for a, b_ in zip(struct.unpack_from('<iI', b, 4), (1, 1e-9)))), qos_profile_sensor_data, raw=True)
n.create_subscription(Odometry, '/qazbot_native/odom', on_odom, 50)
n.create_subscription(String, '/qazbot_d1_map/motion_adapter/status', on_status, 10)
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/qazbot_d1_map/map', on_map, tl)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', on_amcl, tl)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', on_amcl, 10)
def cpu(pid):
    try:
        with open(f'/proc/{pid}/stat') as f: v = f.read().rsplit(')', 1)[1].split()
        return int(v[11]) + int(v[12])
    except Exception: return 0
HZ = os.sysconf('SC_CLK_TCK'); c0 = {k: cpu(p) for k, p in PIDS.items()}; w0 = time.time(); last_dump = w0
Q = lambda a, q: round(float(np.percentile(a, q)), 3) if len(a) else None
def dump():
    wall = time.time() - w0
    res = dict(t_wall_start=w0, duration_s=round(wall, 1), cpu_pct_of_core={k: round(100 * (cpu(p) - c0[k]) / HZ / wall, 1) for k, p in PIDS.items()},
               cloud_latency_ms=dict(p50=Q(np.array(cloud_lat) * 1e3, 50), p95=Q(np.array(cloud_lat) * 1e3, 95)),
               moving_speed=dict(n=len(speeds), v_p50=Q([s[0] for s in speeds], 50), v_p95=Q([s[0] for s in speeds], 95), wz_p95=Q([s[1] for s in speeds], 95)))
    for c in agg:
        R = res[c] = {}
        for k, a in agg[c].items():
            R[k] = dict(n=len(a['lat']), latency_ms=dict(p50=Q(a['lat'], 50), p95=Q(a['lat'], 95), max=Q(a['lat'], 100)),
                        finite_bins_median=Q(a['fin'], 50), map_n=len(a['m1']), map_0p1=Q(a['m1'], 50), map_0p2=Q(a['m2'], 50),
                        map_0p1_mean=round(float(np.mean(a['m1'])), 3) if a['m1'] else None)
        for k, P in pairs[c].items():
            R[k if k.startswith('nodeskew') else 'lio_vs_' + k] = dict(pairs=len(P['off']), stamp_off_ms_p50=Q(P['off'], 50), absdiff_median=Q(P['d'], 50), absdiff_p90=Q(P['d'], 90),
                                    frac_lt_5cm=round(float((np.array(P['d']) < 0.05).mean()), 3) if P['d'] else None,
                                    bins_both=Q(P['both'], 50), only_lio=Q(P['only_l'], 50), only_raw=Q(P['only_r'], 50))
    json.dump(res, open(OUT, 'w'), indent=1)
    return res
while time.time() - w0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    if len(cloud_lat) > 5000: del cloud_lat[:2500]
    if time.time() - last_dump > 60: dump(); last_dump = time.time()
res = dump(); print(json.dumps(res, indent=1))
