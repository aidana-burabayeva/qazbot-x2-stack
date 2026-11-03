#!/usr/bin/env python3
"""10 s buckets of Trust scan input age from the status_logger JSONL. usage: scan_buckets.py <jsonl> <last_s>"""
import json, os, sys, time, collections
L = sys.argv[1]; last = float(sys.argv[2]); f = open(L, 'rb'); f.seek(max(0, os.path.getsize(L) - int(last * 2 * 4000 + 50000)))
b = collections.defaultdict(list); nok = collections.Counter(); t1 = collections.Counter(); end = time.time()
for line in f.read().decode(errors='ignore').splitlines()[1:]:
    try: r = json.loads(line)
    except Exception: continue
    lt = r.get('localization_trust') or {}; a = (lt.get('input_ages') or {}).get('scan')
    if r['t'] < end - last or a is None: continue
    k = int(r['t'] // 10) * 10; b[k].append(a); nok[k] += not lt.get('motion_ok'); t1[k] += any(str(x).startswith('T1') for x in lt.get('reasons') or [])
for k in sorted(b):
    v = sorted(b[k]); print(time.strftime('%H:%M:%S', time.localtime(k)), 'n', len(v), 'p50', round(v[len(v)//2], 3), 'max', round(v[-1], 3), '>0.5:', sum(x > .5 for x in v), 'T1:', t1[k], 'not_ok:', nok[k])
