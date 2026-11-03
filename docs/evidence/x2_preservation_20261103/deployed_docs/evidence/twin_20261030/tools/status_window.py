#!/usr/bin/env python3
"""Summarise the existing orch status_logger JSONL (no DDS): scan age p50/p95/max, T1 fraction, not-motion_ok fraction.
usage: status_window.py <jsonl> <window_s> [end_epoch]"""
import json, sys, time, os
path = sys.argv[1]; win = float(sys.argv[2]); end = float(sys.argv[3]) if len(sys.argv) > 3 else time.time()
sz = os.path.getsize(path); f = open(path, 'rb'); f.seek(max(0, sz - int(win * 2 * 6000 + 20000)))
ages = []; n = t1 = nok = qp = ss = 0
for line in f.read().decode(errors='ignore').splitlines()[1:]:
    try: r = json.loads(line)
    except Exception: continue
    if not (end - win <= r.get('t', 0) <= end): continue
    lt = r.get('localization_trust')
    if not lt: continue
    n += 1
    a = (lt.get('input_ages') or {}).get('scan')
    if a is not None: ages.append(a)
    rs = lt.get('reasons') or []
    if lt.get('t1_transient') or any(str(x).startswith('T1') for x in rs): t1 += 1
    if any('scan' in str(x) and str(x).startswith('T1') for x in rs): ss += 1
    if not lt.get('motion_ok'): nok += 1
    if lt.get('quality_pause'): qp += 1
ages.sort()
q = lambda p: round(ages[min(len(ages) - 1, int(p * len(ages)))], 3) if ages else None
print(json.dumps({'n': n, 'scan_age_p50': q(.5), 'scan_age_p95': q(.95), 'scan_age_max': ages[-1] if ages else None,
                  't1_frac': round(t1 / n, 3) if n else None, 'not_motion_ok_frac': round(nok / n, 3) if n else None,
                  't1_scan_stale_frac': round(ss / n, 3) if n else None, 'quality_pause_frac': round(qp / n, 3) if n else None}))
