#!/usr/bin/env python3
"""Passive stamp-age probe: subscribes only (no publishers, no services).
Usage: age_probe.py DURATION_S TOPIC:TYPE[:qos] ...   TYPE in odom|scan|pc2|imu|tf|str
age = CLOCK_REALTIME at callback - header.stamp (s). Also callback mono gap. Prints JSON."""
import json, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

T = {'odom': Odometry, 'scan': LaserScan, 'pc2': PointCloud2, 'imu': Imu, 'tf': TFMessage, 'str': String}
dur = float(sys.argv[1])
rclpy.init()
node = rclpy.create_node('qazbot_age_probe_ro')
data = {}
statuses = []

def pct(v, p):
    if not v:
        return None
    s = sorted(v)
    return round(s[min(len(s) - 1, int(p * (len(s) - 1) + 0.5))], 4)

def cb(key, kind, msg):
    rt = time.time(); mono = time.monotonic()
    d = data.setdefault(key, {'ages': [], 'mono': [], 'back': 0, 'last': None})
    if kind == 'str':
        statuses.append((rt, msg.data)); d['mono'].append(mono); return
    if kind == 'tf':
        for t in msg.transforms:
            k2 = key + ' ' + t.header.frame_id + '->' + t.child_frame_id
            e = data.setdefault(k2, {'ages': [], 'mono': [], 'back': 0, 'last': None})
            s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            e['ages'].append(rt - s); e['mono'].append(mono)
            if e['last'] is not None and s <= e['last']:
                e['back'] += 1
            e['last'] = s
        return
    s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    d['ages'].append(rt - s); d['mono'].append(mono)
    if d['last'] is not None and s <= d['last']:
        d['back'] += 1
    d['last'] = s

subs = []
for arg in sys.argv[2:]:
    parts = arg.split(':')
    topic, kind = parts[0], parts[1]
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    if len(parts) > 2 and parts[2] == 'rel':
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    subs.append(node.create_subscription(T[kind], topic, lambda m, t=topic, k=kind: cb(t, k, m), qos))
t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    rclpy.spin_once(node, timeout_sec=0.05)
out = {}
for k, d in data.items():
    g = [b - a for a, b in zip(d['mono'], d['mono'][1:])]
    span = (d['mono'][-1] - d['mono'][0]) if len(d['mono']) > 1 else 0
    out[k] = dict(n=len(d['mono']), rate_hz=round((len(d['mono']) - 1) / span, 2) if span else None,
                  age_min=pct(d['ages'], 0), age_p50=pct(d['ages'], .5), age_p95=pct(d['ages'], .95),
                  age_p99=pct(d['ages'], .99), age_max=pct(d['ages'], 1), n_future=sum(1 for a in d['ages'] if a < 0),
                  stamp_nonincreasing=d['back'], gap_p99=pct(g, .99), gap_max=pct(g, 1))
if statuses and __import__('os').environ.get('AGE_PROBE_SAMPLES'):
    out['_status_samples'] = [s for _, s in statuses[:: max(1, len(statuses) // 5)]][:6]
if statuses:
    out['_status_last'] = statuses[-1][1]
    reasons = {}
    for _, s in statuses:
        try:
            j = json.loads(s)
        except Exception:
            continue
        for r in j.get('reasons', []) or []:
            reasons[r] = reasons.get(r, 0) + 1
    out['_status_reason_counts'] = dict(n=len(statuses), **reasons)
print(json.dumps(out, indent=1))
node.destroy_node()
rclpy.try_shutdown()
