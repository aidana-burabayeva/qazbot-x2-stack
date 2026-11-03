#!/usr/bin/env python3
"""Read-only: average Trust soft_score of 5 live scans around a pose; coarse-to-fine grid search. argv: x y yaw radius yawrange"""
import sys, math, time, numpy as np, rclpy
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
import loc_trust as L
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
x0, y0, t0, R, TR = map(float, sys.argv[1:6])
grid = L.OccGrid.from_pgm('/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml')
cfg = L.TrustConfig()
rclpy.init(); n = rclpy.create_node('seed_fit_1102'); scans = []
n.create_subscription(LaserScan, '/qazbot_lio/scan', lambda m: scans.append(L.scan_endpoints(m.ranges, m.angle_min, m.angle_increment, cfg.rmin, cfg.rmax)), qos_profile_sensor_data)
end = time.time() + 10
while len(scans) < 5 and time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
pts = scans[-5:]
def sc(p): return float(np.mean([L.soft_score(q, grid, p, cfg.score_sigma) for q in pts]))
print('n_scans', len(pts), 'pts', [len(q) for q in pts], 'seed score %.3f' % sc((x0, y0, t0)))
best = (sc((x0, y0, t0)), (x0, y0, t0)); top = []
for th in np.arange(-TR, TR + 1e-9, 0.05):
    for dx in np.arange(-R, R + 1e-9, 0.1):
        for dy in np.arange(-R, R + 1e-9, 0.1):
            p = (x0 + dx, y0 + dy, t0 + th); s = sc(p); top.append((s, p))
top.sort(reverse=True)
c = top[0][1]
for th in np.arange(-0.05, 0.051, 0.01):
    for dx in np.arange(-0.1, 0.101, 0.025):
        for dy in np.arange(-0.1, 0.101, 0.025):
            p = (c[0] + dx, c[1] + dy, c[2] + th); s = sc(p)
            if s > best[0]: best = (s, p)
print('best %.3f at x %.3f y %.3f yaw %.3f rad (%.1f deg); offset dxy %.2f m dyaw %.1f deg' % (best[0], *best[1], math.degrees(best[1][2]), math.hypot(best[1][0]-x0, best[1][1]-y0), math.degrees(L.wrap(best[1][2]-t0))))
seen = []
for s, p in top:
    if all(math.hypot(p[0]-q[0], p[1]-q[1]) > 0.4 or abs(L.wrap(p[2]-q[2])) > 0.3 for q in seen):
        seen.append(p); print(' mode %.3f x %.2f y %.2f yaw %.1f deg' % (s, p[0], p[1], math.degrees(p[2])))
    if len(seen) >= 4: break
