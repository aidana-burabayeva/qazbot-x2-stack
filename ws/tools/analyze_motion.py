#!/usr/bin/env python3
"""Offline analysis of a record_motion.sh bag. Reads files only; touches no ROS graph.
Usage: python3 tools/analyze_motion.py BAGDIR [JOINTS.npz]     (default joints: BAGDIR.joints.npz if present)
Needs the ROS environment (ws/src/navigation/env.sh) for rosbag2_py; skips topics whose type cannot be imported.
Sections: AMCL correction per 1 s window vs T5, cmd vs actual velocity (onset lag/threshold), stop time/distance,
FAST-LIO latency (header stamp vs bag receive time), trust READY/MOTION_OK transitions, dynamic envelope radius.
Thresholds below are analysis parameters, not robot limits."""
import json
import math
import os
import subprocess
import sys

import numpy as np

T5_XY, T5_YAW = 0.10, 0.10        # loc_trust.py jump_xy_max / jump_yaw_max (per jump window)
V_MOVE, W_MOVE = 0.02, 0.03       # actual speed counted as "moving" / "stopped" (m/s, rad/s)
CMD_EPS = 1e-3
T = dict(odom='/qazbot_lio/odometry', odl='/qazbot_lio/odom_level', scan='/qazbot_lio/scan',
         amcl='/qazbot_d1_map/amcl_pose', trust='/qazbot_d1_map/localization_trust/status',
         adapter='/qazbot_d1_map/motion_adapter/status',
         cmds=['/qazbot_d1_map/cmd_vel_authorized', '/qazbot_d1_map/cmd_vel', '/qazbot_d1_map/nav2/cmd_vel_nav'])


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls, bad = {}, set()
    for n, ty in types.items():
        try:
            cls[n] = get_message(ty)
        except Exception:
            bad.add(n)
    data = {}
    while r.has_next():
        n, raw, ts = r.read_next()
        if n in bad:
            continue
        data.setdefault(n, []).append((ts * 1e-9, deserialize_message(raw, cls[n])))
    return data, types, bad


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose_arr(msgs, stamp_hdr=False):
    """-> t, x, y, yaw (t = bag receive time unless stamp_hdr)."""
    rows = []
    for t, m in msgs:
        p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if stamp_hdr else t
        rows.append((ts, p.position.x, p.position.y, yaw_of(p.orientation)))
    a = np.array(rows)
    a[:, 3] = np.unwrap(a[:, 3])
    return a


def pct(v):
    v = np.asarray(v)
    return 'n=%d p50 %.4f p95 %.4f max %.4f' % (len(v), *np.percentile(v, [50, 95]), v.max()) if len(v) else 'n=0'


def sec_amcl(d):
    print('\n== AMCL correction (map->odom change) per 1 s window')
    if T['amcl'] not in d or T['odom'] not in d:
        return print('  missing amcl_pose or lio odometry: skipped')
    A, O = pose_arr(d[T['amcl']]), pose_arr(d[T['odom']])
    # map_T_odom = map_T_base * inv(odom_T_base), odom pose taken at the AMCL receive time
    ox = np.interp(A[:, 0], O[:, 0], O[:, 1]); oy = np.interp(A[:, 0], O[:, 0], O[:, 2])
    oth = np.interp(A[:, 0], O[:, 0], O[:, 3])
    yaw = A[:, 3] - oth
    c, s = np.cos(yaw), np.sin(yaw)
    mx = A[:, 1] - (c * ox - s * oy); my = A[:, 2] - (s * ox + c * oy)
    t0 = A[0, 0]; w = ((A[:, 0] - t0) // 1.0).astype(int)
    dxy, dyaw = [], []
    for k in np.unique(w):
        m = w == k
        if m.sum() < 2:
            continue
        dxy.append(np.hypot(np.ptp(mx[m]), np.ptp(my[m])))
        dyaw.append(abs(np.ptp(np.unwrap(yaw[m]))))
    print('  amcl samples %d, windows %d' % (len(A), len(dxy)))
    print('  xy  [m]  :', pct(dxy), '| windows > T5 %.2f: %d' % (T5_XY, sum(x > T5_XY for x in dxy)))
    print('  yaw [rad]:', pct(dyaw), '| windows > T5 %.2f: %d' % (T5_YAW, sum(x > T5_YAW for x in dyaw)))


def vel_from_pose(P, lag=0.2):
    """Body-frame speed and yaw rate from odometry pose differences over `lag` s."""
    t = P[:, 0]; j = np.searchsorted(t, t - lag)
    j = np.clip(j, 0, len(t) - 1); dt = np.maximum(t - t[j], 1e-3)
    dx, dy = P[:, 1] - P[j, 1], P[:, 2] - P[j, 2]
    v = (np.cos(P[:, 3]) * dx + np.sin(P[:, 3]) * dy) / dt
    vy = (-np.sin(P[:, 3]) * dx + np.cos(P[:, 3]) * dy) / dt
    return t, v, vy, (P[:, 3] - P[j, 3]) / dt


def sec_cmd(d):
    print('\n== commanded vs actual velocity, stop behaviour (actual from LIO odometry pose differences, 0.2 s)')
    if T['odom'] not in d:
        return print('  missing lio odometry: skipped')
    P = pose_arr(d[T['odom']]); t, v, vy, w = vel_from_pose(P)
    spd = np.hypot(v, vy)
    for ct in T['cmds']:
        if ct not in d or len(d[ct]) < 2:
            continue
        C = np.array([(tt, m.linear.x, m.linear.y, m.angular.z) for tt, m in d[ct]])
        mag = (np.abs(C[:, 1]) + np.abs(C[:, 2]) > CMD_EPS) | (np.abs(C[:, 3]) > CMD_EPS)
        print(' ', ct, 'msgs', len(C), 'nonzero', int(mag.sum()))
        ons, offs = [], []
        for i in range(1, len(C)):
            if mag[i] and not mag[i - 1]:
                ons.append(i)
            if not mag[i] and mag[i - 1]:
                offs.append(i)
        lags, thr = [], []
        for i in ons:
            ts = C[i, 0]; nxt = ts + 5.0
            m = (t >= ts) & (t <= nxt) & ((spd > V_MOVE) | (np.abs(w) > W_MOVE))
            if m.any():
                k = np.where(m)[0][0]
                lags.append(t[k] - ts)
                thr.append((C[i, 1], C[i, 2], C[i, 3]))
            else:
                thr.append((C[i, 1], C[i, 2], C[i, 3])); lags.append(float('nan'))
        for n, (i, lg) in enumerate(zip(ons, lags)):
            print('    onset %d t=%.2f cmd(vx,vy,wz)=(%.3f,%.3f,%.3f) -> motion after %s s' % (
                n, C[i, 0] - C[0, 0], *thr[n], 'no motion in 5 s' if math.isnan(lg) else '%.2f' % lg))
        for n, i in enumerate(offs):
            ts = C[i, 0]
            before = (t >= ts - 0.3) & (t < ts)
            if not before.any():
                continue
            v0 = spd[before].max()
            after = np.where((t >= ts) & (spd < V_MOVE) & (np.abs(w) < W_MOVE))[0]
            if len(after) == 0:
                print('    stop %d t=%.2f never settled in record' % (n, ts - C[0, 0])); continue
            k = after[0]; j = np.searchsorted(t, ts)
            dist = math.hypot(P[k, 1] - P[j, 1], P[k, 2] - P[j, 2])
            print('    stop %d t=%.2f speed_before %.3f m/s -> settled after %.2f s, travelled %.3f m (odometry; tape is the reference)' % (
                n, ts - C[0, 0], v0, t[k] - ts, dist))


def sec_latency(d):
    print('\n== LIO latency: bag receive time minus header.stamp (same host clock assumed; negative = clock mismatch)')
    for key in ('odom', 'odl', 'scan'):
        n = T[key]
        if n not in d:
            print(' ', n, 'not recorded'); continue
        lat = np.array([t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9) for t, m in d[n]])
        ts = np.array([t for t, _ in d[n]])
        print(' ', n, pct(lat), 'rate %.1f Hz' % ((len(ts) - 1) / max(ts[-1] - ts[0], 1e-6)), '| max gap %.2f s' % np.max(np.diff(ts)) if len(ts) > 1 else '')


def sec_trust(d):
    print('\n== trust READY / MOTION_OK transitions')
    for n in (T['trust'], T['adapter']):
        if n not in d:
            print(' ', n, 'not recorded'); continue
        t0 = d[n][0][0]; prev = None; cnt = 0
        print(' ', n, 'msgs', len(d[n]))
        for t, m in d[n]:
            try:
                j = json.loads(m.data)
            except Exception:
                continue
            if n == T['trust']:
                key = (j.get('ready'), j.get('motion_ok'))
                extra = j.get('reasons')
            else:
                key = (j.get('state'), j.get('reason'))
                extra = ''
            if key != prev:
                print('    t=%7.2f  %s %s' % (t - t0, key, extra if extra else '')); prev = key; cnt += 1
        print('    transitions:', cnt)


def sec_env(bag, joints):
    print('\n== dynamic envelope (tests/footprint/envelope.py)')
    if not joints or not os.path.exists(joints):
        return print('  no joint record found: skipped')
    here = os.path.dirname(os.path.abspath(__file__))
    env = dict(os.environ, X2_URDF_DIR=os.environ.get('X2_URDF_DIR', '/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0'))
    r = subprocess.run([sys.executable, os.path.join(here, '..', 'tests', 'footprint', 'envelope.py'), joints], capture_output=True, text=True, env=env)
    print(r.stdout.rstrip() or r.stderr.rstrip()[-600:])
    print('  reference: rest 0.340, vendor polygon 0.387, rectangle 0.454; R_dyn is not substituted by these')


def main():
    bag = sys.argv[1].rstrip('/')
    joints = sys.argv[2] if len(sys.argv) > 2 else bag + '.joints.npz'
    d, types, bad = read_bag(bag)
    print('bag', bag, '| topics', len(types), '| unreadable types skipped:', sorted(bad) or 'none')
    for n in sorted(d):
        print('  %-55s %6d msgs' % (n, len(d[n])))
    for f in (sec_amcl, sec_cmd, sec_latency, sec_trust):
        try:
            f(d)
        except Exception as e:
            print('  section %s failed: %r' % (f.__name__, e))
    sec_env(bag, joints)


if __name__ == '__main__':
    main()
