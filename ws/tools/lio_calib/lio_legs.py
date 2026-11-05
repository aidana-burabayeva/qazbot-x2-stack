#!/usr/bin/env python3
"""Measure FAST-LIO incremental error between STATIONARY windows of a closed rosbag2, using registration of the
raw chest-LiDAR clouds as the independent reference.  Offline, read-only; run under nice -n 19.

  lio_legs.py windows  BAG                       list stationary windows (LIO-based, EXPERIMENTAL thresholds)
  lio_legs.py selftest BAG [--k 4] [--out J]     registration accuracy: pairs inside the same window (truth: LIO
                                                 relative ~ sway only; reg - LIO should be ~0) + init perturbation
  lio_legs.py legs     BAG [--pairs consecutive|first|all] [--k 3] [--out J]
                                                 per window pair: e = true - LIO (end pose, start heading frame)

Error convention (per leg a->b, both stationary):
  T_lio = T_WB(a)^-1 T_WB(b) (FAST-LIO body = LiDAR IMU), T_reg = T_BL * ICP(cloud_b -> cloud_a, init T_lio) * T_BL^-1
  e_xyz = t_reg - t_lio in B(a), projected on the levelled heading frame of a (up = mean IMU specific force in
  window a; fwd = LiDAR x projected, as level_bridge.py); e_yaw = yaw part of R_lio^T R_reg (+ = true turned more CCW).
  e_along / e_perp: along / left-of the LIO leg direction (OmniMotionModel trans / strafe axes).
Requires topics: /qazbot_lio/odometry, /qazbot_lio/odom_level, raw cloud, LiDAR IMU.
"""
import argparse
import itertools
import json
import math
import sys

import numpy as np

import lc_common as C
from lc_icp import register

ENOUGH = dict(fitness=0.30, eig_trans=0.02)     # QC gates, EXPERIMENTAL: 1 m legs give 0.35-0.42 with consistent results (report)


def pick(t, a, b, k):
    m = np.where((t >= a) & (t <= b))[0]
    if m.size == 0:
        return []
    return sorted(set(m[np.linspace(0, m.size - 1, k).round().astype(int)].tolist()))


def heading_frame(up_B, T_BL):
    f = T_BL[:3, 0] - (T_BL[:3, 0] @ up_B) * up_B
    f /= np.linalg.norm(f)
    return f, np.cross(up_B, f), up_B


def pair_error(od_T, ia, ib, clouds, t, T_BL, up_a, init_perturb=None, rng=None):
    T_lio = C.inv(od_T[ia]) @ od_T[ib]
    T_lio_L = C.inv(T_BL) @ T_lio @ T_BL
    T0 = T_lio_L.copy()
    if init_perturb is not None:
        dp, da = init_perturb
        w = rng.normal(size=3); w *= da / np.linalg.norm(w)
        v = rng.normal(size=3); v *= dp / np.linalg.norm(v)
        P = np.eye(4); P[:3, :3] = C.exp_so3(w); P[:3, 3] = v
        T0 = P @ T0
    T_reg_L, info = register(clouds[t[ib]], clouds[t[ia]], T0)
    if not info['ok']:
        return None, info
    T_reg = T_BL @ T_reg_L @ C.inv(T_BL)
    f, l, u = heading_frame(up_a, T_BL)
    e = T_reg[:3, 3] - T_lio[:3, 3]
    d = T_lio[:3, 3]
    R_lio, R_reg = T_lio[:3, :3], T_reg[:3, :3]
    w_err = C.rotvec(R_lio.T @ R_reg)
    e_yaw = float(w_err @ (R_lio.T @ u))
    tilt = float(np.linalg.norm(w_err - e_yaw * (R_lio.T @ u)))
    dfl = np.array([d @ f, d @ l])
    efl = np.array([e @ f, e @ l])
    n = np.linalg.norm(dfl)
    if n > 0.05:
        ua = dfl / n
        e_al, e_pe = float(efl @ ua), float(-efl[0] * ua[1] + efl[1] * ua[0])
    else:
        e_al = e_pe = float('nan')
    yaw_lio = math.atan2(*(lambda g: (g @ l, g @ f))(R_lio @ T_BL[:3, 0] - ((R_lio @ T_BL[:3, 0]) @ u) * u))
    yaw_reg = math.atan2(*(lambda g: (g @ l, g @ f))(R_reg @ T_BL[:3, 0] - ((R_reg @ T_BL[:3, 0]) @ u) * u))
    res = dict(ta=float(t[ia]), tb=float(t[ib]),
               lio_fwd=float(dfl[0]), lio_left=float(dfl[1]), lio_up=float(d @ u), lio_yaw_wrapped=yaw_lio,
               reg_fwd=float(T_reg[:3, 3] @ f), reg_left=float(T_reg[:3, 3] @ l), reg_yaw_wrapped=yaw_reg,
               e_fwd=float(efl[0]), e_left=float(efl[1]), e_up=float(e @ u), e_along=e_al, e_perp=e_pe,
               e_yaw=e_yaw, e_tilt=tilt,
               fitness=info['fitness'], rmse=info['rmse'], eig_trans_min=min(info['eig_trans']),
               eig_rot_min=min(info['eig_rot']),
               sd_reg_trans=float(np.sqrt(np.diag(info['cov'])[3:]).max()),
               sd_reg_rot=float(np.sqrt(np.diag(info['cov'])[:3]).max()))
    res['qc_ok'] = bool(res['fitness'] >= ENOUGH['fitness'] and res['eig_trans_min'] >= ENOUGH['eig_trans'])
    return res, info


def load(bag, need_clouds_for=None):
    od = C.load_odom(bag, C.ODOM_TOPIC)
    if od is None:
        raise SystemExit('no ' + C.ODOM_TOPIC)
    lv = C.load_odom(bag, C.LEVEL_TOPIC)
    imu = C.load_imu(bag)
    return od, lv, imu


def summarize(rows, keys=('e_fwd', 'e_left', 'e_yaw', 'e_along', 'e_perp', 'e_up')):
    out = {}
    ok = [r for r in rows if r and r['qc_ok']]
    out['n_pairs'], out['n_qc_ok'] = len(rows), len(ok)
    for k in keys:
        v = np.array([r[k] for r in ok], float)
        v = v[np.isfinite(v)]
        if v.size:
            med = float(np.median(v))
            out[k] = med
            out[k + '_mad'] = float(1.4826 * np.median(np.abs(v - med)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=('windows', 'selftest', 'legs'))
    ap.add_argument('bag')
    ap.add_argument('--k', type=int, default=3, help='clouds per window')
    ap.add_argument('--pairs', default='consecutive', choices=('consecutive', 'first', 'all'))
    ap.add_argument('--min-dur', type=float, default=2.0)
    ap.add_argument('--windows', help='explicit windows "a1:b1,a2:b2" (unix s), overrides detection')
    ap.add_argument('--perturb', type=int, default=3, help='selftest: random init perturbations per window')
    ap.add_argument('--out')
    a = ap.parse_args(argv)

    od, lv, imu = load(a.bag)
    Tw = C.poses(od)
    t = od['t']
    if a.windows:
        wins = [tuple(map(float, w.split(':'))) for w in a.windows.split(',')]
    else:
        wins = C.stationary_windows(od, min_dur=a.min_dur)
    report = dict(bag=a.bag, cmd=a.cmd, windows=[])
    lt = ly = None
    if lv is not None:
        lt, lx, lyy, ly = C.level_yaw_track(lv)
    for i, (wa, wb) in enumerate(wins):
        up = C.up_in_B(imu, wa, wb)
        p = Tw[(t >= wa) & (t <= wb), :3, 3]
        report['windows'].append(dict(i=i, a=wa, b=wb, dur=wb - wa, up_B=None if up is None else up.tolist(),
                                      sway_xyz_sd=p.std(0).tolist() if len(p) else None))
    if a.cmd == 'windows':
        print(json.dumps(report, indent=1))
        return 0
    T_BL = C.extrinsic()
    sel = {i: pick(t, wa, wb, a.k) for i, (wa, wb) in enumerate(wins)}
    want = [t[j] for js in sel.values() for j in js]
    clouds = C.load_clouds(a.bag, want)
    print(f'{len(wins)} windows, {len(clouds)}/{len(want)} clouds matched', file=sys.stderr)
    ups = {i: (np.array(report['windows'][i]['up_B']) if report['windows'][i]['up_B'] else None) for i in sel}
    rng = np.random.default_rng(1)
    if a.cmd == 'selftest':
        res = []
        for i, js in sel.items():
            js = [j for j in js if t[j] in clouds]
            if len(js) < 2 or ups[i] is None:
                continue
            rows = []
            for ia, ib in itertools.combinations(js, 2):
                r, info = pair_error(Tw, ia, ib, clouds, t, T_BL, ups[i])
                rows.append(r)
            base, _ = pair_error(Tw, js[0], js[-1], clouds, t, T_BL, ups[i])
            pert = []
            for _ in range(a.perturb):
                r, _ = pair_error(Tw, js[0], js[-1], clouds, t, T_BL, ups[i], init_perturb=(0.2, math.radians(5)), rng=rng)
                if r and base:
                    pert.append(dict(d_fwd=r['e_fwd'] - base['e_fwd'], d_left=r['e_left'] - base['e_left'],
                                     d_yaw=r['e_yaw'] - base['e_yaw'], qc_ok=r['qc_ok']))
            res.append(dict(window=i, pairs=rows, summary=summarize(rows), perturb=pert,
                            sway_reg_xy=[math.hypot(r['reg_fwd'], r['reg_left']) for r in rows if r]))
        report['selftest'] = res
    else:
        ids = list(sel)
        if a.pairs == 'consecutive':
            prs = list(zip(ids[:-1], ids[1:]))
        elif a.pairs == 'first':
            prs = [(ids[0], j) for j in ids[1:]]
        else:
            prs = list(itertools.combinations(ids, 2))
        legs = []
        for wa_i, wb_i in prs:
            if ups[wa_i] is None:
                continue
            rows = []
            for ia in sel[wa_i]:
                for ib in sel[wb_i]:
                    if t[ia] in clouds and t[ib] in clouds:
                        r, info = pair_error(Tw, ia, ib, clouds, t, T_BL, ups[wa_i])
                        rows.append(r if r else None)
            leg = dict(wa=wa_i, wb=wb_i, pairs=[r for r in rows if r], summary=summarize(rows))
            if lt is not None:
                A = 0.5 * (wins[wa_i][0] + wins[wa_i][1]); B = 0.5 * (wins[wb_i][0] + wins[wb_i][1])
                ups_, rem = C.amcl_update_replay(lt, lx, lyy, ly, A, B)
                m = (lt >= A) & (lt <= B)
                path = float(np.sum(np.hypot(np.diff(lx[m]), np.diff(lyy[m])))) if m.sum() > 1 else 0.0
                leg.update(lio_level_dyaw_unwrapped=float(ly[m][-1] - ly[m][0]) if m.sum() else None,
                           lio_level_disp=float(math.hypot(lx[m][-1] - lx[m][0], lyy[m][-1] - lyy[m][0])) if m.sum() else None,
                           lio_level_path=path,
                           amcl_updates=len(ups_), S_tt=float(sum(d * d for d, _ in ups_)),
                           S_rr=float(sum(r * r for _, r in ups_)),
                           S_tt_rem=float(sum(d * d for d, _ in ups_) + rem[0] ** 2),
                           S_rr_rem=float(sum(r * r for _, r in ups_) + rem[1] ** 2),
                           abs_rot_path=float(np.sum(np.abs(np.diff(ly[m])))) if m.sum() > 1 else 0.0)
            legs.append(leg)
        # registration loop consistency: reg(a,b) o reg(b,c) vs reg(a,c) via first-cloud pairs
        report['legs'] = legs
    txt = json.dumps(report, indent=1, default=float)
    if a.out:
        open(a.out, 'w').write(txt)
    brief(report)
    return 0


def brief(rep):
    for w in rep['windows']:
        print(f"win {w['i']}: {w['a']:.2f}-{w['b']:.2f} dur {w['dur']:.1f}s sway_sd {np.round(w['sway_xyz_sd'],4) if w['sway_xyz_sd'] else None}")
    for s in rep.get('selftest', []):
        sm = s['summary']
        print(f"selftest win {s['window']}: pairs {sm['n_pairs']} ok {sm['n_qc_ok']} | reg-LIO med fwd {sm.get('e_fwd',float('nan'))*1000:.1f} "
              f"left {sm.get('e_left',float('nan'))*1000:.1f} up {sm.get('e_up',float('nan'))*1000:.1f} mm yaw {math.degrees(sm.get('e_yaw',float('nan'))):.3f} deg"
              f" | max|e|xy {max([math.hypot(r['e_fwd'],r['e_left']) for r in s['pairs'] if r] or [float('nan')])*1000:.1f} mm"
              f" max|e_yaw| {max([abs(math.degrees(r['e_yaw'])) for r in s['pairs'] if r] or [float('nan')]):.3f} deg"
              f" | perturb spread {max([math.hypot(p['d_fwd'],p['d_left']) for p in s['perturb']] or [float('nan')])*1000:.2f} mm"
              f" fit {min([r['fitness'] for r in s['pairs'] if r] or [0]):.2f} eig {min([r['eig_trans_min'] for r in s['pairs'] if r] or [0]):.3f}")
    for g in rep.get('legs', []):
        sm = g['summary']
        print(f"leg {g['wa']}->{g['wb']}: LIO disp {g.get('lio_level_disp', float('nan')):.3f} m dyaw {math.degrees(g.get('lio_level_dyaw_unwrapped') or float('nan')):.1f} deg "
              f"upd {g.get('amcl_updates')} | e fwd {sm.get('e_fwd',float('nan'))*1000:.1f} left {sm.get('e_left',float('nan'))*1000:.1f} "
              f"along {sm.get('e_along',float('nan'))*1000:.1f} perp {sm.get('e_perp',float('nan'))*1000:.1f} mm yaw {math.degrees(sm.get('e_yaw',float('nan'))):.3f} deg "
              f"(mad xy {sm.get('e_fwd_mad',float('nan'))*1000:.1f}/{sm.get('e_left_mad',float('nan'))*1000:.1f} mm) ok {sm['n_qc_ok']}/{sm['n_pairs']} "
              f"fit {min([r['fitness'] for r in g['pairs']] or [0]):.2f}")


if __name__ == '__main__':
    sys.exit(main())
