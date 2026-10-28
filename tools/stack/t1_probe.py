#!/usr/bin/env python3
"""Passive T1 correlation probe (subscribe only). For every trust status containing a T1 reason, record what an
independent subscriber (same QoS as trust: BEST_EFFORT depth 1 for odom_level/scan) had at that moment:
time since last receipt and the source-stamp age of the last message; plus status inter-arrival gap.
Usage: t1_probe.py DURATION_S"""
import json, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

dur = float(sys.argv[1])
rclpy.init()
n = rclpy.create_node('qazbot_t1_probe_ro')
last = {}
st = dict(n=0, t1=0, events=[], prev=None, gaps=[])
def src(key):
    def cb(m):
        last[key] = (time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return cb
def status(m):
    mono, rt = time.monotonic(), time.time()
    if st['prev'] is not None:
        st['gaps'].append(mono - st['prev'])
    gap = None if st['prev'] is None else mono - st['prev']
    st['prev'] = mono
    st['n'] += 1
    try:
        j = json.loads(m.data)
    except Exception:
        return
    t1 = [r for r in j.get('reasons', []) if r.startswith('T1')]
    if t1:
        st['t1'] += 1
        ev = dict(t=time.strftime('%H:%M:%S', time.localtime(rt)) + ('%.2f' % (rt % 1))[1:], reasons=t1,
                  status_gap=round(gap, 3) if gap else None)
        for k, (lm, s) in last.items():
            ev[k + '_since_recv'] = round(mono - lm, 3)
            ev[k + '_stamp_age'] = round(rt - s, 3)
        st['events'].append(ev)
be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Odometry, '/qazbot_lio/odom_level', src('odom'), be)
n.create_subscription(LaserScan, '/qazbot_lio/scan', src('scan'), be)
n.create_subscription(String, '/qazbot_d1_map/localization_trust/status', status, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
g = sorted(st['gaps'])
print(json.dumps(dict(status_n=st['n'], t1_ticks=st['t1'],
                      status_gap_p50=round(g[len(g)//2], 3) if g else None,
                      status_gap_p99=round(g[int(.99*(len(g)-1))], 3) if g else None,
                      status_gap_max=round(g[-1], 3) if g else None,
                      status_gaps_over_0p3=sum(1 for x in g if x > 0.3),
                      events=st['events']), indent=1))
