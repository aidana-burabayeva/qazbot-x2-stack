#!/usr/bin/env python3
"""alpha1-5 for nav2_amcl::OmniMotionModel from lio_legs.py 'legs' JSONs.

Model (navigation2-1.1.18 nav2_amcl/src/motion_model/omni_motion_model.cpp:53-87), per filter update with odom
increment (dt = |delta xy|, dr = delta yaw):
  sd_trans  = sqrt(a3 dt^2 + a4 dr^2)   along the motion bearing
  sd_rot    = sqrt(a1 dr^2 + a2 dt^2)
  sd_strafe = sqrt(a4 dr^2 + a5 dt^2)   perpendicular to the bearing
Independent per update => over a leg: Var_along = a3 S_tt + a4 S_rr, Var_perp = a5 S_tt + a4 S_rr,
Var_yaw = a1 S_rr + a2 S_tt, with S_tt = sum dt_k^2, S_rr = sum dr_k^2 from an exact replay of
AmclNode::shouldUpdateFilter (amcl_node.cpp:768-777) on /qazbot_lio/odom_level (lio_legs.py).
Pure-rotation legs: the bearing is undefined -> both position components carry a4 S_rr (isotropic).
Estimator: second moments (bias INCLUDED, AMCL noise is zero-mean so a systematic LIO error must be covered),
NNLS over legs; registration noise variance optionally subtracted (--reg-sd). Upper CI via chi-square on n legs.
"""
import argparse
import json
import math
import sys

import numpy as np
from scipy.optimize import nnls
from scipy.stats import chi2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('legs', nargs='+')
    ap.add_argument('--reg-sd', type=float, default=0.0, help='registration sd per position axis (m), subtracted')
    ap.add_argument('--reg-sd-yaw', type=float, default=0.0, help='registration yaw sd (rad), subtracted')
    ap.add_argument('--rem', action='store_true', help='count the not-yet-applied remainder as an update')
    ap.add_argument('--conf', type=float, default=0.95)
    a = ap.parse_args(argv)
    rows = []
    for f in a.legs:
        for g in json.load(open(f)).get('legs', []):
            s = g['summary']
            if s.get('n_qc_ok', 0) == 0 or 'e_yaw' not in s:
                continue
            Stt = g['S_tt_rem' if a.rem else 'S_tt']; Srr = g['S_rr_rem' if a.rem else 'S_rr']
            if Stt + Srr <= 0:
                continue
            rot_only = (g['lio_level_disp'] < 0.15 and abs(g['lio_level_dyaw_unwrapped']) > 0.5)   # path is useless: stepping sway adds ~1.7 m path on a 360 deg turn
            ea, ep = (s.get('e_along'), s.get('e_perp')) if not rot_only else (None, None)
            rows.append(dict(src=f, wa=g['wa'], wb=g['wb'], Stt=Stt, Srr=Srr, rot_only=rot_only,
                             e_fwd=s['e_fwd'], e_left=s['e_left'], e_yaw=s['e_yaw'], e_al=ea, e_pe=ep,
                             disp=g['lio_level_disp'], dyaw=g['lio_level_dyaw_unwrapped']))
    if not rows:
        print('no usable legs'); return 1
    r2, ry2 = a.reg_sd ** 2, a.reg_sd_yaw ** 2
    A_al, y_al, A_pe, y_pe, A_y, y_y = [], [], [], [], [], []
    for r in rows:
        A_y.append([r['Srr'], r['Stt']]); y_y.append(max(0.0, r['e_yaw'] ** 2 - 2 * ry2))
        if r['rot_only']:      # isotropic: each axis ~ a4 Srr  (a3/a5 terms ~0 because Stt ~0)
            for e in (r['e_fwd'], r['e_left']):
                A_al.append([r['Stt'], r['Srr']]); y_al.append(max(0.0, e * e - 2 * r2))
        else:
            A_al.append([r['Stt'], r['Srr']]); y_al.append(max(0.0, (r['e_al'] or 0) ** 2 - 2 * r2))
            A_pe.append([r['Stt'], r['Srr']]); y_pe.append(max(0.0, (r['e_pe'] or 0) ** 2 - 2 * r2))
    (a1, a2), _ = nnls(np.array(A_y), np.array(y_y))
    (a3, a4_al), _ = nnls(np.array(A_al), np.array(y_al))
    a5, a4_pe = (nnls(np.array(A_pe), np.array(y_pe))[0] if A_pe else (float('nan'), float('nan')))
    a4 = max(a4_al, a4_pe if np.isfinite(a4_pe) else 0)
    n = len(rows)
    up = (n) / chi2.ppf(1 - a.conf, n) if n > 1 else float('inf')   # one-sided upper CI factor on variance
    out = dict(n_legs=n, n_rot_only=sum(r['rot_only'] for r in rows), alpha_point=dict(alpha1=a1, alpha2=a2, alpha3=a3,
               alpha4=a4, alpha5=a5), chi2_upper_factor=up,
               alpha_upper=dict(alpha1=a1 * up, alpha2=a2 * up, alpha3=a3 * up, alpha4=a4 * up, alpha5=a5 * up),
               legs=rows, note='point = NNLS of second moments; upper = point * n/chi2_{1-conf}(n) (Gaussian, '
               'independent legs ASSUMED); p95 of |e| per class printed separately')
    for k in ('e_fwd', 'e_left', 'e_yaw'):
        v = np.abs([r[k] for r in rows])
        out[f'p95_abs_{k}'] = float(np.percentile(v, 95)) if v.size else None
    print(json.dumps(out, indent=1, default=float))
    return 0


if __name__ == '__main__':
    sys.exit(main())
