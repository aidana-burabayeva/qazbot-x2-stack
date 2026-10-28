"""Offline replay: bag map->odom sequence through old (1 s cumulative window) vs new (consecutive step) T5.
Usage: source /opt/ros/humble/setup.bash; ROS_LOCALHOST_ONLY=1 python3 replay_t5_bag.py [db3]  (no nodes, no DDS traffic)"""
import sys, math, sqlite3
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
db = sys.argv[1] if len(sys.argv) > 1 else '/home/agi/control_backend/logs/bags/short_nav_01/short_nav_01_0.db3'
XY, YAW, WIN = 0.10, 0.10, 1.0
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
c = sqlite3.connect(db)
tid = c.execute("select id from topics where name='/qazbot_d1_map/tf'").fetchone()[0]
seq, last = [], None
for ts, data in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,)):
    for t in deserialize_message(bytes(data), TFMessage).transforms:
        if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
            q = t.transform.rotation; tr = t.transform.translation
            p = (tr.x, tr.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            if p != last:
                seq.append((ts/1e9, p)); last = p      # one sample per distinct AMCL update
print('distinct map->odom samples:', len(seq))
# Evaluator ticks (sample-and-hold of the latest map->odom, TICK_HZ) -- the old window sees held values too.
TICK = 0.1
hist, old, new, maxstep, prev, i, cur = [], [], [], 0.0, None, 0, None
t = seq[0][0]
while t <= seq[-1][0] + 2.0:
    while i < len(seq) and seq[i][0] <= t: cur = seq[i][1]; i += 1
    p = cur
    hist = [(a, b) for a, b in hist if t - a <= WIN] + [(t, p)]
    p0 = hist[0][1]
    jx = max(math.hypot(b[0]-p0[0], b[1]-p0[1]) for _, b in hist); jy = max(abs(wrap(b[2]-p0[2])) for _, b in hist)
    if jx > XY or jy > YAW: old.append((t, jx, jy))
    if prev:
        sx = math.hypot(p[0]-prev[0], p[1]-prev[1]); sy = abs(wrap(p[2]-prev[2])); maxstep = max(maxstep, sx)
        if sx > XY or sy > YAW: new.append((t, sx, sy))
    prev = p; t += TICK
print('max per-update step xy: %.3f m' % maxstep)
print('OLD trips (tick-level): %d, first/last at %s, max window xy %.3f' % (len(old), (round(old[0][0],1), round(old[-1][0],1)) if old else None, max([o[1] for o in old], default=0)))
print('NEW trips: %d' % len(new))
