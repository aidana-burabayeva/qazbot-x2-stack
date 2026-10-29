#!/usr/bin/env python3
"""Verdicts for one nightly run. usage: summarize_nightly.py <results/nightly_stamp> <campaign.tsv>
PASS: scenario rc=0, results.json present, no hard failures, no unexpected soft failures.
WARN: only unexpected soft failures. FAIL: hard failures. ERROR: no results.json or rc != 0. Exit 1 unless all PASS/WARN."""
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
meta = dict(line.split('=', 1) for line in (out / 'meta.env').read_text().splitlines() if '=' in line)
stamp = meta['stamp']
rows = []
for line in Path(sys.argv[2]).read_text().splitlines():
    f = line.split('\t')
    if len(f) < 4 or f[0].startswith('#') or f[1] not in ('lite', 'i1'):
        continue
    if f[1] == 'i1':  # verdict = scripts/i1/summarize.py exit code: 0 GO, 1 NO-GO
        rc_file = out / f'{f[0]}.rc'
        rc = int(rc_file.read_text()) if rc_file.exists() else None
        tm = next((kv.split('=', 1)[1] for kv in f[2].split() if kv.startswith('TWIN_MAP=')), meta.get('map'))
        rows.append({'scenario': f[0], 'tier': 'i1', 'map': tm, 'rc': rc, 'hard_failed': [], 'soft_unexpected': [],
                     'verdict': {0: 'PASS', 1: 'FAIL'}.get(rc, 'ERROR')})
        continue
    name, expected = f[0], set() if f[3] == '-' else set(f[3].split(','))
    rc_file, res_file = out / f'{name}.rc', out.parent / f'nightly_{stamp}_{name}' / 'results.json'
    rc = int(rc_file.read_text()) if rc_file.exists() else None
    row = {'scenario': name, 'map': meta.get('map'), 'rc': rc, 'verdict': 'ERROR', 'hard_failed': [], 'soft_unexpected': [], 'soft_expected': []}
    if rc == 0 and res_file.exists():
        res = json.loads(res_file.read_text())
        soft = [c['name'] for c in res['checks'] if not c['ok'] and not c['hard']]
        row['hard_failed'] = res['hard_failed']
        row['soft_unexpected'] = sorted(set(soft) - expected)
        row['soft_expected'] = sorted(set(soft) & expected)
        amcl = next((c['detail'] for c in res['checks'] if c['name'] == 'amcl_error_recorded'), None)
        row['amcl_error'] = amcl
        row['verdict'] = 'FAIL' if row['hard_failed'] else 'WARN' if row['soft_unexpected'] else 'PASS'
    rows.append(row)
summary = {**meta, 'scenarios': rows, 'counts': {v: sum(r['verdict'] == v for r in rows) for v in ('PASS', 'WARN', 'FAIL', 'ERROR')}}
(out / 'summary.json').write_text(json.dumps(summary, indent=1))
for r in rows:
    print(f"{r['verdict']:5} {r['scenario']} hard={r['hard_failed']} soft_unexpected={r['soft_unexpected']}")
print(summary['counts'])
sys.exit(0 if all(r['verdict'] in ('PASS', 'WARN') for r in rows) and rows else 1)
