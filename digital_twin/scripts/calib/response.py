"""Command -> motion response of the X2 from a robot bag (run in the dev container):  response.py BAG_DIR [BAG_DIR...]
cmd = /aima/mc/locomotion/velocity (what MC receives, 50 Hz); motion = /aima/mc/leg_odometry (body-frame speed from pose differences).
Reports: steady-state gain, lag (cross-correlation), stop distance/time after the command drops to 0, yaw overshoot after an angular stop."""
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DT = 0.05  # resample step, s


def series(c, topic, fn, keep=lambda m: True):
    row = c.execute('select id, type from topics where name=?', (topic,)).fetchone()
    if not row:
        return None
    ty = get_message(row[1])
    t, v = [], []
    for ts, data in c.execute('select timestamp, data from messages where topic_id=? order by timestamp', (row[0],)):
        m = deserialize_message(data, ty)
        if keep(m):
            t.append(ts / 1e9)
            v.append(fn(m))
    return np.array(t), np.array(v)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def analyse(d):
    c = sqlite3.connect(f'file:{next(d.glob("*.db3"))}?mode=ro', uri=True)
    # the topic mixes sources: `rc` (vendor, near-zero keep-alive) and our `sdk_qazbot_nav2`; only ours is a Nav2 command
    cmd = series(c, '/aima/mc/locomotion/velocity', lambda m: (m.forward_velocity, m.lateral_velocity, m.angular_velocity),
                 keep=lambda m: m.source == 'sdk_qazbot_nav2')
    odo = series(c, '/aima/mc/leg_odometry', lambda m: (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)))
    if cmd is None or odo is None or len(cmd[0]) < 10:
        print(f'== {d.name}: no cmd/leg_odometry'); return
    t0 = max(cmd[0][0], odo[0][0]) - 2.0; t1 = min(cmd[0][-1], odo[0][-1]) + 3.0
    tt = np.arange(t0, t1, DT)
    idx = np.searchsorted(cmd[0], tt, side='right') - 1                        # hold the last command, MC zeroes it after 200 ms
    fresh = (idx >= 0) & (tt - cmd[0][np.clip(idx, 0, None)] <= 0.2)
    cv = np.where(fresh[:, None], cmd[1][np.clip(idx, 0, None)], 0.0)           # fwd, lat, ang command
    px, py = np.interp(tt, odo[0], odo[1][:, 0]), np.interp(tt, odo[0], odo[1][:, 1])
    ya = np.interp(tt, odo[0], np.unwrap(odo[1][:, 2]))
    k = int(0.3 / DT)                                       # 0.3 s central difference
    dx, dy = (px[2 * k:] - px[:-2 * k]) / (2 * k * DT), (py[2 * k:] - py[:-2 * k]) / (2 * k * DT)
    yc = ya[k:-k]
    vf = np.cos(yc) * dx + np.sin(yc) * dy                  # body forward speed
    vl = -np.sin(yc) * dx + np.cos(yc) * dy
    w = (ya[2 * k:] - ya[:-2 * k]) / (2 * k * DT)
    cv, tt = cv[k:-k], tt[k:-k]
    print(f'== {d.name}: {tt[-1] - tt[0]:.0f} s, cmd levels fwd {np.unique(np.round(cv[:, 0], 2))[:8]}  ang {np.unique(np.round(cv[:, 2], 2))[:8]}')
    for name, c_, m_, unit in (('forward', cv[:, 0], vf, 'm/s'), ('lateral', cv[:, 1], vl, 'm/s'), ('yaw', cv[:, 2], w, 'rad/s')):
        if np.abs(c_).max() < 0.02:
            print(f'  {name}: no command'); continue
        best = max(range(0, int(2.0 / DT)), key=lambda s: np.corrcoef(c_[:len(c_) - s], m_[s:])[0, 1] if s else np.corrcoef(c_, m_)[0, 1])
        lag = best * DT
        gains = []  # steady segments: command constant (|d|<0.01) >= 2 s, gain from the last 1 s
        i = 0
        while i < len(c_):
            j = i
            while j < len(c_) and abs(c_[j] - c_[i]) < 0.01:
                j += 1
            if (j - i) * DT >= 2.0 and abs(c_[i]) >= 0.05:
                seg = m_[int(j - 1.0 / DT):j]
                gains.append((float(c_[i]), float(seg.mean()), float(seg.std())))
            i = max(j, i + 1)
        print(f'  {name}: lag {lag:.2f} s (xcorr {np.corrcoef(c_[:len(c_) - best], m_[best:])[0, 1]:.2f}), steady segments {len(gains)}')
        for lvl in sorted({round(g[0], 2) for g in gains}):
            gs = [g for g in gains if round(g[0], 2) == lvl]
            print(f'     cmd {lvl:+.2f} {unit}: measured {np.mean([g[1] for g in gs]):+.3f} (n={len(gs)}, ratio {np.mean([g[1] for g in gs]) / lvl:.2f})')
        # stops: command >=0.05 for >=1 s then |cmd|<0.01 for >=1.5 s
        stops = []
        for i in range(int(1.0 / DT), len(c_) - int(1.5 / DT)):
            if abs(c_[i]) < 0.01 and abs(c_[i - 1]) >= 0.05 and all(abs(c_[i - int(1.0 / DT):i]) >= 0.05) and all(abs(c_[i:i + int(1.5 / DT)]) < 0.01):
                v0 = m_[i]
                after = m_[i:i + int(3.0 / DT)]
                settled = next((n for n, x in enumerate(after) if abs(x) < (0.03 if name != 'yaw' else 0.05)), len(after) - 1)
                path = float(np.trapz(m_[i:i + settled + 1], dx=DT))
                stops.append((float(c_[i - 1]), float(v0), settled * DT, path))
        for s in stops[:6]:
            print(f'     stop from cmd {s[0]:+.2f}: speed at command=0 {s[1]:+.3f}, to rest {s[2]:.2f} s, residual {"angle" if name == "yaw" else "distance"} {s[3]:+.3f} {"rad" if name == "yaw" else "m"}')
        if stops:
            print(f'     stops n={len(stops)}: residual median {np.median([abs(s[3]) for s in stops]):.3f} max {max(abs(s[3]) for s in stops):.3f}, time median {np.median([s[2] for s in stops]):.2f} s')


for p in sys.argv[1:]:
    analyse(Path(p))
