"""Summarize live-chain runs: per run the outcome of both goals; aggregate counts.

  ros2 run x2_twin_scenarios summarize PREFIX [PREFIX...] [--results-dir DIR]
"""
import argparse
import json
import os
import pathlib


def summarize(results_dir: pathlib.Path, prefix: str):
    rows = []
    for d in sorted(results_dir.glob(prefix + '*')):
        f = d / 'results.json'
        if not f.exists():
            rows.append((d.name, None, None, None, None))
            continue
        r = json.loads(f.read_text())
        c = {x['name']: x for x in r['checks']}

        def status(n):
            return None if n not in c else c[n]['detail']

        def err(n):
            return c[n]['detail']['xy_error'] if n in c and isinstance(c[n]['detail'], dict) else None

        rows.append((d.name, status('goal_out_succeeded'), err('goal_out_pose'), status('goal_back_succeeded'), err('goal_back_pose')))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('prefix', nargs='+')
    ap.add_argument('--results-dir', default=os.environ.get('TWIN_RESULTS_DIR', '/results'))
    a = ap.parse_args()
    for prefix in a.prefix:
        rows = summarize(pathlib.Path(a.results_dir), prefix)
        print('==', prefix)

        def fmt(v):
            return '-' if v is None else ('%.2f' % v if isinstance(v, float) else str(v))

        for name, so, eo, sb, eb in rows:
            print(f'   {name}  out={"OK" if so == 4 else fmt(so)} xy_err={fmt(eo)} | back={"OK" if sb == 4 else fmt(sb)} xy_err={fmt(eb)}')
        n = len(rows)
        print(f'   n={n}  outbound OK {sum(r[1] == 4 for r in rows)}/{n}   return OK {sum(r[3] == 4 for r in rows)}/{n}')


if __name__ == '__main__':
    main()
