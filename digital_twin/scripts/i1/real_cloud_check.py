"""I1 on REAL recorded clouds of the physical X2 (read-only): legacy binning vs candidate build_ranges.

Pipeline = level_bridge's slice (gravity-levelled from the LiDAR IMU, Z_MIN/Z_MAX, floor margin, RANGE_MIN/MAX) applied to the raw
chest-LiDAR clouds (not deskewed: the robot stands/walks slowly). Checks per cloud:
  C1 every legacy finite bin keeps exactly the same range in the candidate (no real return is lost or changed),
  C2 candidate +inf only inside the FOV mask, never within the guard of a return, never on a bin with a return,
  C3 outside the FOV there is no +inf (unknown stays unknown),
  C4 every +inf bin has a cloud point within the coverage bridge (the bearing really was observed, not just 'no point in the slice'),
  C5 a half-blocked sensor (all points of the right half removed from the cloud) produces no +inf there,
and prints the FOV actually covered by returns, the per-cloud count of returning bins in the FOV (is 'few returns' ever normal?) and the
share of hit-free FOV bins that stay unknown for lack of coverage (does the coverage rule over-restrict?).
usage: real_cloud_check.py BAG_DB3 [MAX_CLOUDS] [RANGE_MAX]
"""
import importlib.util
import json
import math
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

spec = importlib.util.spec_from_file_location('lb', '/work/candidates/i1_rpp_collision/level_bridge.py')
lb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lb)

db, max_clouds = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 60
range_max = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
ex = json.load(open('/backend_logs/x2_extrinsics.json'))
R = np.array(ex['R_imu_lidar'])
T = np.array(ex['T_imu_lidar'])
IMU, CLOUD = '/aima/hal/sensor/lidar_chest_front/imu', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
c = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
tid = {n: (i, get_message(t)) for i, n, t in c.execute('select id,name,type from topics')}
acc = [[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in
       (deserialize_message(d, tid[IMU][1]) for (d,) in c.execute('select data from messages where topic_id=?', (tid[IMU][0],)))]
g = np.mean(acc, axis=0)
up = g / np.linalg.norm(g)                                      # up in the IMU frame (accelerometer reads +g up at rest)
fwd = lb.LIDAR_FWD_IN_IMU - (lb.LIDAR_FWD_IN_IMU @ up) * up
fwd /= np.linalg.norm(fwd)
left = np.cross(up, fwd)

rows = c.execute('select data from messages where topic_id=? order by timestamp', (tid[CLOUD][0],)).fetchall()
step = max(1, len(rows) // max_clouds)
FOV = None
agg = dict(clouds=0, c1_bad=0, c2_bad=0, c3_bad=0, c4_bad=0, c5_bad=0, fov_hits=[], uncovered_free_share=[], legacy_inf=0, new_inf=0, new_nan=0, finite=0, fov_lo=[], fov_hi=[], no_inf_clouds=0)
for (d,) in rows[::step]:
    m = deserialize_message(d, tid[CLOUD][1])
    S = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)       # mixed-type fields (ring, ...): structured read
    P = np.stack([S['x'], S['y'], S['z']], axis=1).astype(np.float64)
    if P.size == 0:
        continue
    X = P @ R.T + T
    z = X @ up
    bx, by = X @ fwd, X @ left
    flat = np.hypot(bx, by)
    k = (z > lb.Z_MIN + lb.FLOOR_MARGIN_PER_M * flat) & (z < lb.Z_MAX)
    r = flat[k]
    a = np.arctan2(by[k], bx[k])
    ok = (r > lb.RANGE_MIN) & (r < range_max)
    r, a = r[ok], a[ok]
    n = lb.N_BINS
    far = flat > lb.RANGE_MIN                                            # coverage: ANY point (floor, far wall) before the slice
    covered = lb.coverage_bins(np.arctan2(by[far], bx[far]), n)
    legacy = np.full(n, np.inf, dtype=np.float32)
    np.minimum.at(legacy, ((a + math.pi) / (2 * math.pi) * n).astype(int) % n, r.astype(np.float32))
    new = lb.build_ranges(r, a, covered)
    inc = 2 * math.pi / n
    centre = -math.pi + (np.arange(n) + 0.5) * inc
    in_fov = (centre >= math.radians(lb.FOV_DEG[0])) & (centre <= math.radians(lb.FOV_DEG[1]))
    fin_l = np.isfinite(legacy)
    hit = np.isfinite(new)
    near = hit.copy()
    for s_ in range(1, lb.INF_GUARD_BINS + 1):
        near |= np.roll(hit, s_) | np.roll(hit, -s_)
    agg['clouds'] += 1
    agg['c1_bad'] += int(np.count_nonzero(~np.isclose(new[fin_l], legacy[fin_l])))
    inf_new = np.isposinf(new)
    agg['c2_bad'] += int(np.count_nonzero(inf_new & (~in_fov | near)))
    agg['c3_bad'] += int(np.count_nonzero(inf_new & ~in_fov))
    seen = covered.copy()
    for s_ in range(1, lb.COVER_DILATE_BINS + 1):
        seen |= np.roll(covered, s_) | np.roll(covered, -s_)
    agg['c4_bad'] += int(np.count_nonzero(inf_new & ~seen))
    agg['fov_hits'].append(int(np.count_nonzero(hit & in_fov)))
    free_candidates = in_fov & ~near
    agg['uncovered_free_share'].append(float(np.count_nonzero(free_candidates & ~seen)) / max(1, np.count_nonzero(free_candidates)))
    # C5: the left half delivers nothing; the coverage bridge may leak into it by at most COVER_DILATE_BINS beyond the boundary
    blocked = (centre >= (lb.COVER_DILATE_BINS + 1) * 2 * math.pi / n) & (centre <= math.radians(lb.FOV_DEG[1]))
    cov_half = lb.coverage_bins(np.arctan2(by[far], bx[far])[~((np.arctan2(by[far], bx[far]) >= 0.0)
                                                              & (np.arctan2(by[far], bx[far]) <= math.radians(lb.FOV_DEG[1])))], n)
    half = lb.build_ranges(r, a, cov_half)
    agg['c5_bad'] += int(np.count_nonzero(np.isposinf(half) & blocked))
    agg['legacy_inf'] += int(np.count_nonzero(np.isposinf(legacy)))
    agg['new_inf'] += int(np.count_nonzero(inf_new))
    agg['new_nan'] += int(np.count_nonzero(np.isnan(new)))
    agg['finite'] += int(np.count_nonzero(hit))
    if hit.any():
        deg = np.degrees(centre[hit])
        agg['fov_lo'].append(float(np.percentile(deg, 1)))
        agg['fov_hi'].append(float(np.percentile(deg, 99)))
    if not np.any(inf_new):
        agg['no_inf_clouds'] += 1
fh = np.array(agg.pop('fov_hits'))
agg['fov_return_bins_per_cloud'] = dict(min=int(fh.min()), p5=int(np.percentile(fh, 5)), median=int(np.median(fh)), max=int(fh.max()))
agg['unknown_for_lack_of_coverage_share'] = dict(median=round(float(np.median(agg['uncovered_free_share'])), 3),
                                                 max=round(float(np.max(agg['uncovered_free_share'])), 3))
agg.pop('uncovered_free_share')
agg['fov_lo'] = round(float(np.median(agg['fov_lo'])), 1) if agg['fov_lo'] else None
agg['fov_hi'] = round(float(np.median(agg['fov_hi'])), 1) if agg['fov_hi'] else None
print(json.dumps(agg))
raise SystemExit(0 if agg['c1_bad'] == agg['c2_bad'] == agg['c3_bad'] == agg['c4_bad'] == agg['c5_bad'] == 0 and agg['clouds'] else 1)
