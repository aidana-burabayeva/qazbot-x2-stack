#!/usr/bin/env python3
"""MC response measured two independent ways: native /aima/mc/leg_odometry vs FAST-LIO odom_level.

Per command burst (consecutive non-zero /aima/mc/locomotion/velocity messages, gap <= 1.0 s, any source):
  commanded distance = sum(|fwd| * dt), commanded turn = sum(ang * dt)   (dt capped at 0.2 s: MC zeroes a stale command)
  achieved           = from the burst start to 2.5 s after its last command (coast included), by each odometry:
    distance = norm of the 3-D displacement; turn = rotation angle of R0^T R1 (axis-angle magnitude).
Both are frame-independent: leg_odometry is expressed for the chest-LiDAR IMU (tilted), so raw planar x/y or Euler yaw of
that frame must not be used (see docs/digital_twin/friday_data_review.md). Sign of the turn is taken from the command.
Output: JSON lines to stdout and a table. Usage: analyze_leg_vs_lio.py BAG_DIR... [--json OUT.json]
"""
import bisect, json, math, pathlib, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity

GAP_S, COAST_S, DT_CAP = 1.0, 2.5, 0.2


def quat_to_R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def rot_angle(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))


def load_odom(db, tid, topic):
    if topic not in tid:
        return None
    t, p, q = [], [], []
    for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid[topic],)):
        m = deserialize_message(bytes(d), Odometry)
        t.append(ts / 1e9)
        p.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
        o = m.pose.pose.orientation
        q.append((o.x, o.y, o.z, o.w))
    return np.array(t), np.array(p), np.array(q)


def achieved(odo, t0, t1):
    if odo is None:
        return None
    t, p, q = odo
    i, j = bisect.bisect_left(t, t0), bisect.bisect_right(t, t1) - 1
    if j - i < 3:
        return None
    path = float(np.sum(np.linalg.norm(np.diff(p[i:j + 1], axis=0), axis=1)))
    disp = float(np.linalg.norm(p[j] - p[i]))
    turn = rot_angle(quat_to_R(q[i]).T @ quat_to_R(q[j]))
    return dict(disp=disp, path=path, turn_deg=math.degrees(turn))


def analyze(bag):
    db = sqlite3.connect(f'file:{next(bag.glob("*.db3"))}?mode=ro', uri=True)
    tid = {n: i for i, n in db.execute('select id,name from topics')}
    if '/aima/mc/locomotion/velocity' not in tid:
        return []
    cmds = []
    for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid['/aima/mc/locomotion/velocity'],)):
        m = deserialize_message(bytes(d), McLocomotionVelocity)
        cmds.append((ts / 1e9, m.source, m.forward_velocity, m.lateral_velocity, m.angular_velocity))
    leg, lio = load_odom(db, tid, '/aima/mc/leg_odometry'), load_odom(db, tid, '/qazbot_lio/odom_level')
    bursts, cur = [], []
    for c in cmds:
        nz = abs(c[2]) + abs(c[3]) + abs(c[4]) > 1e-9
        if not nz:
            continue
        if cur and c[0] - cur[-1][0] > GAP_S:
            bursts.append(cur); cur = []
        cur.append(c)
    if cur:
        bursts.append(cur)
    rows = []
    for b in bursts:
        dist = turn = 0.0
        for a, c in zip(b, b[1:]):
            dt = min(c[0] - a[0], DT_CAP)
            dist += abs(a[2]) * dt
            turn += a[4] * dt
        dur = b[-1][0] - b[0][0]
        if dur < 1.0 or (dist < 0.05 and abs(turn) < math.radians(5)):
            continue
        t0, t1 = b[0][0], b[-1][0] + COAST_S
        rows.append(dict(bag=bag.name, t=round(t0 - cmds[0][0], 1), dur=round(dur, 1), sources=sorted({x[1] for x in b}),
                         fwd_med=float(np.median([x[2] for x in b])), ang_med=float(np.median([x[4] for x in b])),
                         cmd_dist=dist, cmd_turn_deg=math.degrees(turn), leg=achieved(leg, t0, t1), lio=achieved(lio, t0, t1)))
    return rows


def ratio(a, b):
    return '%.2f' % (a / b) if a is not None and b and abs(b) > 0.05 else '-'


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
    if out:
        args.remove(out)
    allrows = []
    for b in args:
        allrows += analyze(pathlib.Path(b))
    print('%-22s %6s %5s %-22s %5s %5s | %6s %7s | %6s %6s %5s | %6s %6s %5s' % (
        'bag', 't', 'dur', 'sources', 'fwd', 'ang', 'cmdD', 'cmdT', 'legD', 'legT', 'x/cmd', 'lioD', 'lioT', 'x/cmd'))
    for r in allrows:
        L, O = r['leg'], r['lio']
        print('%-22s %6.1f %5.1f %-22s %5.2f %5.2f | %6.2f %+7.1f | %6s %6s %5s | %6s %6s %5s' % (
            r['bag'], r['t'], r['dur'], ','.join(s[:10] for s in r['sources'])[:22], r['fwd_med'], r['ang_med'], r['cmd_dist'], r['cmd_turn_deg'],
            '%.2f' % L['disp'] if L else '-', '%.1f' % L['turn_deg'] if L else '-', ratio(L['disp'], r['cmd_dist']) if L else '-',
            '%.2f' % O['disp'] if O else '-', '%.1f' % O['turn_deg'] if O else '-', ratio(O['disp'], r['cmd_dist']) if O else '-'))
    if out:
        pathlib.Path(out).write_text(json.dumps(allrows, indent=1))
