#!/usr/bin/env python3
"""Summarize attempted chain runs, separating startup, admission, status and truth accuracy.
Usage: summarize_chain.py PREFIX [PREFIX...]
"""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]


def summarize(prefix):
    dirs = {p.name: p for p in (ROOT / 'tests/integration_results').glob(prefix + '*') if p.is_dir()}
    names = sorted(set(dirs) | {p.stem for p in (ROOT / 'logs/twin').glob(prefix + '*.log')})
    counts = dict(attempts=len(names), results=0, outbound_completed=0, return_completed=0,
                  outbound_succeeded=0, return_succeeded=0, outbound_accurate=0, return_accurate=0)
    print('==', prefix)
    for name in names:
        f = dirs.get(name, Path('/nonexistent')) / 'results.json'
        if not f.exists():
            print(name, 'NO_RESULT (startup/infrastructure; inspect log)')
            continue
        r = json.loads(f.read_text())
        c = {x['name']: x for x in r.get('checks', [])}
        counts['results'] += 1
        stages = []
        for leg, label in [('out', 'outbound'), ('back', 'return')]:
            status = c.get(f'goal_{leg}_succeeded', {}).get('detail')
            pose = c.get(f'goal_{leg}_pose', {})
            if status is not None:
                counts[label + '_completed'] += 1
                counts[label + '_succeeded'] += status == 4
            counts[label + '_accurate'] += bool(status == 4 and pose.get('ok'))
            err = pose.get('detail', {}).get('xy_error')
            stages.append(f'{label}: status={status}, xy={err:.3f}m, pose_ok={pose.get("ok")}' if err is not None
                          else f'{label}: status={status}, accepted={c.get(f"goal_{leg}_accepted", {}).get("ok")}')
        faults = sorted({e[2]['fault'] for e in r.get('events', []) if e[1] == 'coord' and e[2].get('fault')})
        print(name, ' | '.join(stages), 'faults=' + ','.join(faults))
    print(json.dumps(counts, ensure_ascii=False))
    return counts


if __name__ == '__main__':
    for prefix in sys.argv[1:]:
        summarize(prefix)
