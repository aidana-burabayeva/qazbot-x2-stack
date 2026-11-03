#!/usr/bin/env python3
"""Analyse one run_scenario.sh output (probe.csv) against the full-bag emulation (offline_emulate.py).

usage: analyze_run.py RUN_DIR EMU_PKL [--min-height 0.05] [--slowdown 0.3]
Writes RUN_DIR/analysis.json and prints a short summary.
"""
import argparse
import bisect
import csv
import datetime as dt
import json
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'lib'))
import cm_eval_lib as L  # noqa: E402


def hms(ns):
    return dt.datetime.fromtimestamp(ns / 1e9).strftime('%H:%M:%S.%f')[:-4]


def load_probe(path):
    rows = {'cloud': [], 'in': [], 'out': []}
    with open(path) as f:
        for r in csv.DictReader(f):
            if int(r['sim_ns']) == 0:
                continue  # before the first /clock message
            rows[r['kind']].append((int(r['wall_ns']), int(r['sim_ns']), int(r['stamp_ns']),
                                    float(r['vx']), float(r['vy']), float(r['wz'])))
    return rows


def segments(times, labels):
    seg = []
    for t, c in zip(times, labels):
        if not seg or seg[-1][2] != c:
            seg.append([t, t, c, 0])
        seg[-1][1] = t
        seg[-1][3] += 1
    return seg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('emu')
    ap.add_argument('--min-height', type=float, default=0.05)
    ap.add_argument('--slowdown', type=float, default=0.3)
    ap.add_argument('--source-timeout', type=float, default=0.5)
    a = ap.parse_args()
    rows = load_probe(os.path.join(a.run_dir, 'probe.csv'))
    emu = pickle.load(open(a.emu, 'rb'))
    key = f'act_{a.min_height:.2f}'
    emu_by_stamp = {r['stamp']: r[key] for r in emu['recs']}
    vin = (0.30, 0.0, 0.0)
    clouds = rows['cloud']
    c_sim = [c[1] for c in clouds]
    c_wall = [c[0] for c in clouds]
    ins_wall = [r[0] for r in rows['in']]
    out = rows['out']
    labels, ages, agree, proc, n_cmp = [], [], 0, [], 0
    for w, s, _, vx, vy, wz in out:
        cls = L.classify_output((vx, vy, wz), vin, a.slowdown)
        labels.append(cls)
        i = bisect.bisect_right(c_wall, w) - 1  # latest cloud received before this output
        if i >= 0:
            age = (s - clouds[i][2]) / 1e9
            ages.append(age)
            exp = emu_by_stamp.get(clouds[i][2])
            if age > a.source_timeout:
                exp = L.PASS
            if exp is not None:
                n_cmp += 1
                agree += exp == cls
        j = bisect.bisect_right(ins_wall, w) - 1
        if j >= 0:
            proc.append((w - ins_wall[j]) / 1e6)
    # onset events: cloud whose emulated action is STOP while the previous one was not
    events = []
    for k in range(1, len(clouds)):
        e0, e1 = emu_by_stamp.get(clouds[k - 1][2]), emu_by_stamp.get(clouds[k][2])
        if e1 == L.STOP and e0 not in (L.STOP, None):
            w_rx = clouds[k][0]
            first = next(((w, s) for (w, s, *_), c in zip(out, labels) if w >= w_rx and c == L.STOP), None)
            if first:
                events.append(dict(cloud_stamp=hms(clouds[k][2]),
                                   stamp_to_stop_s=(first[1] - clouds[k][2]) / 1e9,
                                   rx_to_stop_wall_ms=(first[0] - w_rx) / 1e6))
    out_sim = [o[1] for o in out]
    gaps_sim = np.diff(out_sim) / 1e9 if len(out_sim) > 1 else np.array([])
    gaps_wall = np.diff([o[0] for o in out]) / 1e6 if len(out) > 1 else np.array([])
    seg = segments(out_sim, labels)
    res = dict(
        run=json.load(open(os.path.join(a.run_dir, 'run.json'))) if os.path.exists(
            os.path.join(a.run_dir, 'run.json')) else None,
        n_cloud=len(clouds), n_in=len(rows['in']), n_out=len(out),
        sim_span=[hms(out_sim[0]), hms(out_sim[-1])] if out_sim else None,
        counts={c: labels.count(c) for c in (L.STOP, L.SLOWDOWN, L.APPROACH, L.PASS)},
        emu_agreement=dict(n=n_cmp, agree=agree, rate=(agree / n_cmp if n_cmp else None)),
        data_age_at_output_s=L.latency_stats(ages),
        cmd_in_to_out_wall_ms=L.latency_stats(proc),
        stop_onset_events=events,
        onset_stamp_to_stop_s=L.latency_stats([e['stamp_to_stop_s'] for e in events]),
        onset_rx_to_stop_wall_ms=L.latency_stats([e['rx_to_stop_wall_ms'] for e in events]),
        out_gap_sim_s_max=float(gaps_sim.max()) if len(gaps_sim) else None,
        out_gap_wall_ms=L.latency_stats(list(gaps_wall)),
        cloud_gap_sim_s_max=float(np.diff(c_sim).max() / 1e9) if len(c_sim) > 1 else None,
        segments=[[hms(s0), hms(s1), c, n] for s0, s1, c, n in seg],
    )
    with open(os.path.join(a.run_dir, 'analysis.json'), 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k not in ('segments', 'stop_onset_events')}, indent=1))
    for s in res['segments'][:40]:
        print('  seg', s)


if __name__ == '__main__':
    main()
