#!/usr/bin/env python3
"""Read-only D-01 probe: per-message receive (CLOCK_REALTIME) vs header stamp for scan/odom, Trust T1 reasons, CPU.
Usage: freshness_probe.py DURATION_S OUT_JSON"""
import sys, time, json, rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
dur, out = float(sys.argv[1]), sys.argv[2]
rclpy.init(); n = rclpy.create_node('freshness_probe_1102')
rec = {'scan': [], 'odom': []}; t1 = []; cpu = []
def cb(k):
    def f(m):
        rec[k].append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    return f
def st(m):
    r = [x for x in json.loads(m.data).get('reasons', []) if x.startswith('T1_') and x != 'T1_tf_stale']
    if r: t1.append((time.time(), r))
n.create_subscription(LaserScan, '/qazbot_lio/scan', cb('scan'), qos_profile_sensor_data)
n.create_subscription(Odometry, '/qazbot_native/odom', cb('odom'), 10)
n.create_subscription(String, '/qazbot_d1_map/localization_trust/status', st, 10)
def cpustat():
    v = list(map(int, open('/proc/stat').readline().split()[1:])); return sum(v), v[3] + v[4]
c0 = cpustat(); t0 = time.time(); nxt = t0 + 1
while time.time() < t0 + dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt:
        c1 = cpustat(); cpu.append(round(100 * (1 - (c1[1] - c0[1]) / max(1, c1[0] - c0[0])), 1)); c0 = c1; nxt += 1
def pct(a, p):
    a = sorted(a); return round(a[min(len(a) - 1, int(p * len(a)))], 4) if a else None
res = {'t0': t0, 'dur': dur, 'loadavg': open('/proc/loadavg').read().split()[:3],
       'cpu_busy_pct': {'p50': pct(cpu, .5), 'max': max(cpu) if cpu else None}}
for k, v in rec.items():
    age = [r - h for r, h in v]; gap = [b[0] - a[0] for a, b in zip(v, v[1:])]
    res[k] = {'n': len(v), 'rate_hz': round(len(v) / dur, 2),
              'age_p50': pct(age, .5), 'age_p95': pct(age, .95), 'age_p99': pct(age, .99), 'age_max': round(max(age), 4) if age else None,
              'gap_p99': pct(gap, .99), 'gap_max': round(max(gap), 4) if gap else None,
              'gaps_gt_0_3': [round(v[i + 1][0], 2) for i, g in enumerate(gap) if g > 0.3]}
res['t1_events'] = [(round(t, 2), r) for t, r in t1]
json.dump(res, open(out, 'w'), indent=1)
print(json.dumps({k: (res[k] if k not in ('scan', 'odom') else {x: y for x, y in res[k].items() if x != 'gaps_gt_0_3'} | {'n_gaps': len(res[k]['gaps_gt_0_3'])}) for k in res if k != 't1_events'} | {'n_t1': len(t1)}))
