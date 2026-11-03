#!/usr/bin/env python3
"""Summarise the full-bag emulation per ground-truth window (pd04-analysis timeline).

usage: emu_summary.py EMU_PKL [OUT_JSON]
"""
import datetime as dt
import json
import pickle
import sys
from collections import Counter

import numpy as np

# Local-time windows from .orchestrator/reports/20261103-pd04-analysis.md section 1.
# User correction 2026-11-03: the bag has NO boxes; pd04 "low object" windows are operator-only windows.
WINDOWS = [
    ('empty_0', '18:30:16', '18:31:05', 'empty'),
    ('empty_1', '18:31:41', '18:34:05', 'empty'),
    ('person_front', '18:34:07', '18:34:51', 'person'),
    ('person_left_edge', '18:34:53', '18:35:18', 'person'),
    ('after_leave', '18:35:26', '18:35:36', 'empty'),
    ('walk_across_2m', '18:35:37', '18:35:44', 'person_far'),
    ('operator_win_A', '18:35:44', '18:36:26', 'operator (pd04 low-object window; no box per user)'),
    ('operator_cross', '18:36:27', '18:36:29', 'person'),
    ('quiet_B', '18:36:30', '18:36:44', 'empty'),
    ('operator_win_C', '18:36:45', '18:36:54', 'operator (pd04 low-object window; no box per user)'),
    ('empty_2', '18:36:59', '18:38:33', 'empty'),
    ('walker_blind_exit', '18:38:35', '18:38:41', 'person'),
    ('empty_3', '18:38:43', '18:42:16', 'empty'),
]
DAY = '2026-11-03'


def ns(hms):
    return int(dt.datetime.strptime(f'{DAY} {hms}', '%Y-%m-%d %H:%M:%S').timestamp() * 1e9)


def main():
    d = pickle.load(open(sys.argv[1], 'rb'))
    recs = d['recs']
    mhs = d['min_heights']
    out = {}
    for name, a, b, kind in WINDOWS:
        sel = [r for r in recs if ns(a) <= r['stamp'] < ns(b)]
        if not sel:
            continue
        w = dict(kind=kind, n=len(sel), t=[a, b])
        for mh in mhs:
            c = Counter(r[f'act_{mh:.2f}'] for r in sel)
            w[f'mh_{mh:.2f}'] = {k: round(v / len(sel), 3) for k, v in sorted(c.items())}
        xs = [r['x_near_corridor'] for r in sel if r['x_near_corridor'] is not None]
        w['x_near_corridor_min'] = round(min(xs), 3) if xs else None
        w['x_near_corridor_med'] = round(float(np.median(xs)), 3) if xs else None
        w['slant_min'] = round(min(r['slant_min'] for r in sel), 3)
        w['nstop_max_0.05'] = max(r['nstop_0.05'] for r in sel)
        out[name] = w
        print(name, json.dumps(w))
    # longest run of consecutive non-PASS decisions inside empty windows (false positive bursts)
    if len(sys.argv) > 2:
        json.dump(out, open(sys.argv[2], 'w'), indent=1)


if __name__ == '__main__':
    main()
