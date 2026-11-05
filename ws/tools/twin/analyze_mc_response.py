#!/usr/bin/env python3
"""Real-robot MC response from a bag: per constant-command segment, commanded vs measured speed.
Measured = finite difference of LIO odom_level pose (and MC leg_odometry) over the steady part of the segment.
Usage: analyze_mc_response.py BAG_DIR [--min-seg S]"""
import bisect, json, math, pathlib, sqlite3, sys
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity
bag = pathlib.Path(sys.argv[1]); db = sqlite3.connect(f'file:{next(bag.glob("*.db3"))}?mode=ro', uri=True)
tid = {n: i for i, n in db.execute('select id,name from topics')}
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
def odom(topic):
    out = []
    for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid[topic],)):
        m = deserialize_message(bytes(d), Odometry); q = m.pose.pose.orientation
        out.append((ts / 1e9, m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w)))
    return out
cmds = []
for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid['/aima/mc/locomotion/velocity'],)):
    m = deserialize_message(bytes(d), McLocomotionVelocity); cmds.append((ts / 1e9, m.source, m.forward_velocity, m.lateral_velocity, m.angular_velocity))
lio, leg = odom('/qazbot-lio/odom_level'.replace('qazbot-lio', 'qazbot_lio')), odom('/aima/mc/leg_odometry')
# segments: consecutive messages with the same (source, fwd, lat, ang), non-zero, gaps < 0.3 s
segs = []; cur = None
for t, s, f, l, a in cmds:
    nz = abs(f) + abs(l) + abs(a) > 1e-9
    if nz and cur and cur['key'] == (s, f, l, a) and t - cur['t1'] < 0.3: cur['t1'] = t; cur['n'] += 1
    elif nz: cur = dict(key=(s, f, l, a), t0=t, t1=t, n=1); segs.append(cur)
    else: cur = None
def rate(seq, t0, t1):
    ts = [x[0] for x in seq]; i, j = bisect.bisect_left(ts, t0), bisect.bisect_right(ts, t1) - 1
    if j - i < 3: return None
    a, b = seq[i], seq[j]; dt = b[0] - a[0]
    dx, dy = b[1] - a[1], b[2] - a[2]; fwd = (dx * math.cos(a[3]) + dy * math.sin(a[3])) / dt   # along initial heading (straight walks)
    return math.hypot(dx, dy) / dt, wrap(b[3] - a[3]) / dt, dt
rows = []
for s in segs:
    dur = s['t1'] - s['t0']
    if dur < float(sys.argv[3]) if len(sys.argv) > 3 else dur < 1.5: continue
    t0, t1 = s['t0'] + 1.0, s['t1']            # skip the first second (start-up)
    L, G = rate(lio, t0, t1), rate(leg, t0, t1)
    _, f, l, a = s['key']
    rows.append(dict(src=s['key'][0], t=round(s['t0'] - cmds[0][0], 1), dur=round(dur, 1), cmd_fwd=f, cmd_lat=l, cmd_ang=a,
                     lio_speed=L and round(L[0], 3), lio_yawrate=L and round(L[1], 3), leg_speed=G and round(G[0], 3), leg_yawrate=G and round(G[1], 3)))
print(json.dumps(rows, indent=0)); 
