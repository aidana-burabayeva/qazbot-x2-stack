"""Per-window swept envelope from a joint recording: shows how the radius about the navigation point changes
between standing, walking, turning and stopping.  Usage: envelope_phases.py REC.npz [WINDOW_S=2] [STRIDE_S=0.1]
Also reports torso yaw rate (turn detection) and the overall maximum (R_dyn candidate)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import envelope as E   # noqa: E402

path = sys.argv[1]
win = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
stride = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
d = np.load(path)
names, q, imu, t = list(d["names"]), d["q"], d["imu_xyzw"], d["t"]
step = max(1, int(round(stride * 50)))
idx = list(range(0, len(t), step))
rows = [E.snapshot_envelope(dict(zip(names, q[k])), imu[k]) for k in idx]
tt = t[idx] - t[0]
yaw = np.unwrap([math.atan2(2 * (o[3] * o[2] + o[0] * o[1]), 1 - 2 * (o[1] ** 2 + o[2] ** 2)) for o in imu[idx]])
print("duration %.1f s, snapshots %d, analysed %d (every %.2f s)" % (t[-1] - t[0], len(t), len(idx), stride))
print("%-9s %7s %7s %7s %7s %7s %9s" % ("t0..t1,s", "r_max", "front", "back", "left", "right", "yaw_rate"))
for a in np.arange(0, tt[-1], win):
    m = (tt >= a) & (tt < a + win)
    if m.sum() < 2:
        continue
    r = [rows[i] for i in np.where(m)[0]]
    f = lambda k, fn: fn([x[k] for x in r])
    yr = (yaw[m][-1] - yaw[m][0]) / (tt[m][-1] - tt[m][0])
    print("%4.1f-%-4.1f %7.3f %7.3f %7.3f %7.3f %7.3f %8.2f" % (a, a + win, f("r", max), f("x_front", max), f("x_back", min),
                                                              f("y_left", max), f("y_right", min), yr))
print("R_dyn candidate (max over all): %.3f m" % max(x["r"] for x in rows))
