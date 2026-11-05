#!/usr/bin/env python3
"""Extract the D-05 numbers from a recorded bag (E2E-02 style: MC velocity by source, odom_level).

usage: extract_stop_events.py <bag_dir> <T0_epoch_s of the goal start>
Prints (a) steady response cmd->speed, (b) stop events (v0, travel until stopped), (c) speed noise windows,
(d) the stop_time fit k = sum(v d)/sum(v^2). Needs aimdk_msgs on the path (source aimdk_env.sh).
"""
import bisect
import math
import statistics as st
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, T0 = sys.argv[1], float(sys.argv[2])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
ty = {t.name: t.type for t in r.get_all_topics_and_types()}
mc, ol = [], []
cls = {k: get_message(ty[k]) for k in ('/aima/mc/locomotion/velocity', '/qazbot_lio/odom_level')}
while r.has_next():
    tp, data, ts = r.read_next()
    if tp not in cls:
        continue
    m = deserialize_message(data, cls[tp])
    t = ts * 1e-9
    if tp.endswith('velocity') and m.source != 'rc' and T0 <= t <= T0 + 90:
        mc.append((t, m.forward_velocity, m.angular_velocity))
    elif tp.endswith('odom_level'):
        q = m.pose.pose.orientation
        ol.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
t = [x[0] for x in mc]
vx = [x[1] for x in mc]
wz = [x[2] for x in mc]
olt = [x[0] for x in ol]


def idx(tt):
    return min(max(bisect.bisect(olt, tt) - 1, 0), len(ol) - 1)


def speed(tt, w):
    i, j = idx(tt - w / 2), idx(tt + w / 2)
    dt = ol[j][0] - ol[i][0]
    return math.hypot(ol[j][1] - ol[i][1], ol[j][2] - ol[i][2]) / dt if dt > 0 else float('nan')


ev, last = [], -9
for k in range(1, len(t)):
    if vx[k] < 0.02 and max(vx[max(0, k - 25):k]) >= 0.1 and t[k] - last > 3 and abs(wz[k]) < 0.7:
        ev.append(k)
        last = t[k]
rows = []
for k in ev:
    tc = t[k]
    v0 = speed(tc, 0.4)
    i0 = idx(tc)
    tend = tc
    for s in [x * 0.1 for x in range(1, 26)]:
        tend = tc + s
        if speed(tc + s, 0.4) < 0.12:
            break
    j = idx(tend)
    rows.append((tc - T0, v0, math.hypot(ol[j][1] - ol[i0][1], ol[j][2] - ol[i0][2]), tend - tc))
    print('stop t %.1f v0 %.2f travel %.2f m in %.1f s' % rows[-1])
clean = [r_ for r_ in rows if r_[3] < 1.5]
k = sum(r_[1] * r_[2] for r_ in clean) / sum(r_[1] ** 2 for r_ in clean)
res = [r_[2] - 0.4 * r_[1] for r_ in clean]
print('clean events %d, fit d = %.3f s * v, residual of 0.40 s*v: sd %.3f max %.3f' % (
    len(clean), k, st.pstdev(res), max(abs(x) for x in res)))
