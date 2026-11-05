#!/usr/bin/env python3
"""Gain of the real MC for the adapter's (sdk_*) commands: commanded vs achieved distance / rotation per command burst.
commanded distance = sum(fwd*dt), commanded turn = sum(ang*dt); achieved from LIO odom_level between the burst start
and 2 s after its last command (coast included). Usage: analyze_sdk_gain.py BAG_DIR..."""
import bisect, math, pathlib, sqlite3, sys
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
for b in sys.argv[1:]:
    bag = pathlib.Path(b); db = sqlite3.connect(f'file:{next(bag.glob("*.db3"))}?mode=ro', uri=True)
    tid = {n: i for i, n in db.execute('select id,name from topics')}
    lio = []
    for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid['/qazbot_lio/odom_level'],)):
        m = deserialize_message(bytes(d), Odometry); q = m.pose.pose.orientation
        lio.append((ts / 1e9, m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w)))
    lt = [x[0] for x in lio]
    cmds = []
    for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid['/aima/mc/locomotion/velocity'],)):
        m = deserialize_message(bytes(d), McLocomotionVelocity)
        if m.source.startswith('sdk') or m.source.startswith('node'):
            cmds.append((ts / 1e9, m.forward_velocity, m.lateral_velocity, m.angular_velocity))
    print('==', bag.name, 'sdk commands:', len(cmds), 'sources:', sorted({m for m in [0]}) and '')
    bursts, cur = [], []
    for c in cmds:
        if cur and c[0] - cur[-1][0] > 3.0: bursts.append(cur); cur = []
        cur.append(c)
    if cur: bursts.append(cur)
    for bu in bursts:
        dist = turn = 0.0; wmax = 0.0
        for a, c in zip(bu, bu[1:]):
            dt = min(c[0] - a[0], 0.2)       # MC zeroes a command after 0.2 s without update
            dist += a[1] * dt; turn += a[3] * dt; wmax = max(wmax, abs(a[3]))
        t0, t1 = bu[0][0], bu[-1][0] + 2.0
        i, j = bisect.bisect_left(lt, t0), min(bisect.bisect_right(lt, t1) - 1, len(lio) - 1)
        if j - i < 5: continue
        path = sum(math.hypot(lio[k + 1][1] - lio[k][1], lio[k + 1][2] - lio[k][2]) for k in range(i, j))
        dyaw = sum(wrap(lio[k + 1][3] - lio[k][3]) for k in range(i, j))
        print('burst t=%7.1f dur=%5.1fs  cmd: dist %6.2f m turn %+6.1f deg (|w|max %.2f) | achieved: path %6.2f m turn %+6.1f deg | gain dist %s turn %s' % (
            t0 - cmds[0][0], bu[-1][0] - t0, dist, math.degrees(turn), wmax, path, math.degrees(dyaw),
            '%.2f' % (path / dist) if abs(dist) > 0.05 else '-', '%.2f' % (dyaw / turn) if abs(turn) > 0.05 else '-'))
