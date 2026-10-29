"""A/B: /qazbot_lio/scan (FAST-LIO+level_bridge) vs /qazbot_raw/scan (+_low). Read-only subscriber.
Usage: ab_eval.py DURATION_S OUT_JSON PID_FASTLIO PID_LEVELBRIDGE PID_RAW"""
import json, math, sys, time, os
import numpy as np, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.ndimage import distance_transform_edt
DUR, OUT = float(sys.argv[1]), sys.argv[2]
PIDS = dict(fastlio=int(sys.argv[3]), level_bridge=int(sys.argv[4]), raw_scan_bridge=int(sys.argv[5]))
TOP = {'lio': '/qazbot_lio/scan', 'raw': '/qazbot_raw/scan', 'raw_low': '/qazbot_raw/scan_low'}
rclpy.init(); n = rclpy.create_node('qazbot_raw_ab_eval')
rec = {k: [] for k in TOP}; cloud_lat = []; st = {'map': None, 'pose': None}
def mk(k):
    def cb(m):
        now = time.time(); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        r = np.array(m.ranges, dtype=np.float32); r[(r < m.range_min) | (r > m.range_max)] = np.inf
        rec[k].append((t, now, r, st['pose']))
    return cb
for k, t in TOP.items(): n.create_subscription(LaserScan, t, mk(k), 20)
n.create_subscription(PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    lambda m: cloud_lat.append(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), qos_profile_sensor_data, raw=False)
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/qazbot_d1_map/map', lambda m: st.__setitem__('map', m), tl)
def onpose(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    st['pose'] = (p.x, p.y, 2 * math.atan2(q.z, q.w), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', onpose, tl)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', onpose, 10)
def cpu(pid):
    with open(f'/proc/{pid}/stat') as f: v = f.read().rsplit(')', 1)[1].split()
    return int(v[11]) + int(v[12])
HZ = os.sysconf('SC_CLK_TCK'); c0 = {k: cpu(p) for k, p in PIDS.items()}; w0 = time.time()
while time.time() - w0 < DUR: rclpy.spin_once(n, timeout_sec=0.05)
wall = time.time() - w0; cpu_pct = {k: round(100 * (cpu(p) - c0[k]) / HZ / wall, 1) for k, p in PIDS.items()}
res = {'duration_s': round(wall, 1), 'cpu_pct_of_core': cpu_pct, 'cloud_latency_ms': None}
P = lambda a, q: round(float(np.percentile(a, q)), 1)
if cloud_lat: res['cloud_latency_ms'] = dict(p50=P(np.array(cloud_lat) * 1e3, 50), p95=P(np.array(cloud_lat) * 1e3, 95), n=len(cloud_lat))
for k, v in rec.items():
    if len(v) < 3: res[k] = {'n': len(v)}; continue
    t = np.array([x[0] for x in v]); lat = np.array([x[1] - x[0] for x in v]) * 1e3
    fin = np.array([np.isfinite(x[2]).sum() for x in v])
    res[k] = dict(n=len(v), rate_hz=round((len(v) - 1) / (t[-1] - t[0]), 2), latency_ms=dict(p50=P(lat, 50), p95=P(lat, 95), max=round(float(lat.max()), 1)),
                  finite_bins_median=int(np.median(fin)), finite_pct=round(100 * float(np.median(fin)) / 1440, 1))
# pairing lio <-> raw by nearest stamp
def pair(a, b):
    tb = np.array([x[0] for x in rec[b]]); out = []
    for x in rec[a]:
        j = int(np.argmin(np.abs(tb - x[0]))); out.append((x[0] - tb[j], x[2], rec[b][j][2]))
    return out
for b in ('raw', 'raw_low'):
    if len(rec['lio']) < 3 or len(rec[b]) < 3: continue
    pr = pair('lio', b); off = np.array([p[0] for p in pr]) * 1e3
    # use the modal offset (stamp convention), then per-bin agreement on pairs within 15 ms of it
    mo = float(np.median(off)); good = [p for p in pr if abs(p[0] * 1e3 - mo) < 15]
    d = []; both = []; only_l = []; only_r = []
    for _, rl, rr in good:
        fl, fr = np.isfinite(rl), np.isfinite(rr); m = fl & fr
        d.append(np.abs(rl[m] - rr[m])); both.append(m.sum()); only_l.append((fl & ~fr).sum()); only_r.append((fr & ~fl).sum())
    d = np.concatenate(d) if d else np.zeros(0)
    res[f'lio_vs_{b}'] = dict(stamp_offset_ms_median=round(mo, 1), pairs=len(good), absdiff_m=dict(median=round(float(np.median(d)), 3), p90=round(float(np.percentile(d, 90)), 3),
        frac_lt_5cm=round(float((d < 0.05).mean()), 3)) if len(d) else None, bins_both=int(np.median(both)), bins_only_lio=int(np.median(only_l)), bins_only_raw=int(np.median(only_r)))
# map match at the AMCL pose (latest pose at scan receive time)
m = st['map']
if m is not None:
    W, H, res_m = m.info.width, m.info.height, m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
    g = np.array(m.data, dtype=np.int16).reshape(H, W); occ = g >= 65
    dist = distance_transform_edt(~occ) * res_m
    for k, v in rec.items():
        sc = []; sc2 = []
        for t, now, r, pose in v:
            if pose is None: continue
            x, y, yaw, _ = pose; a = -math.pi + np.arange(1440) * 2 * math.pi / 1440; f = np.isfinite(r)
            if f.sum() < 10: continue
            mx = x + r[f] * np.cos(a[f] + yaw); my = y + r[f] * np.sin(a[f] + yaw)
            ix = ((mx - ox) / res_m).astype(int); iy = ((my - oy) / res_m).astype(int)
            ins = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
            dd = np.full(len(mx), np.inf); dd[ins] = dist[iy[ins], ix[ins]]
            sc.append(float((dd <= 0.1).mean())); sc2.append(float((dd <= 0.2).mean()))
        if sc: res.setdefault('map_match', {})[k] = dict(n=len(sc), frac_within_0p1m=round(float(np.median(sc)), 3), frac_within_0p2m=round(float(np.median(sc2)), 3))
    res['amcl_pose_last'] = st['pose']
json.dump(res, open(OUT, 'w'), indent=1); print(json.dumps(res, indent=1))
