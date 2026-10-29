"""Per-update map->odom steps of a recorded physical run: old metric (translation of the transform) vs robot displacement
at the current odom->base pose. usage: replay_steps.py TF.json"""
import bisect, json, math, sys

d = json.load(open(sys.argv[1]))
mo, ob = d['map_odom'], d['odom_base']
ob_t = [r[0] for r in ob]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def base_at(t):
    i = min(max(bisect.bisect_left(ob_t, t), 0), len(ob) - 1)
    return tuple(ob[i][2:5])


rows, prev = [], None
for r in mo:
    v = tuple(r[2:5])
    if prev is not None and v != prev[1]:
        o = base_at(r[0])
        p0, p1 = compose(prev[1], o), compose(v, o)
        rows.append((r[0], math.hypot(v[0] - prev[1][0], v[1] - prev[1][1]), abs(wrap(v[2] - prev[1][2])),
                     math.hypot(p1[0] - p0[0], p1[1] - p0[1]), math.hypot(*o[:2])))
    prev = (r[0], v)
print('updates with a change: %d' % len(rows))
if rows: print("max old xy %.4f  max robot xy %.4f  max yaw %.4f" % (max(x[1] for x in rows), max(x[3] for x in rows), max(x[2] for x in rows)))
for x in sorted(rows, key=lambda x: -max(x[1], x[3]))[:6]:
    print('t=%.3f old_xy=%.4f yaw=%.4f robot_xy=%.4f |odom_base|=%.2f' % x)
