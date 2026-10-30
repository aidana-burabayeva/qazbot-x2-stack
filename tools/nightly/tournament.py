#!/usr/bin/env python3
"""Config tournament + Pareto selection (offline, SIM/MODEL). Same seeds and map for every config; winners are re-scored on UNSEEN seeds.
Fitness: collision / footprint contact is a HARD FAIL (score = -inf) regardless of reward; `REACHED` alone proves nothing.
Correct safe rejection of an impossible goal is rewarded. Keeps baseline + safest + most_accurate + fastest_among_safe + whole front.
usage: tournament.py --train 11,12 --unseen 91,92 --out docs/nightly/tournament/l1_YYYYMMDD
Real (L2/L3) results plug in via the same row format: id category expected result reason t_s path_m min_clear_m."""
import argparse, itertools, json, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / 'ws/src/maps/main_office'
EXE = ROOT / 'run/nightly/l1_runner'


def evaluate(cfg, seeds, work):
    rows = []
    for sd in seeds:
        d = work / f'seed{sd}'; d.mkdir(parents=True, exist_ok=True)
        if not (d / 's.tsv').exists():  # scenarios depend only on seed+map: generate once
            subprocess.run([sys.executable, str(ROOT / 'tools/nightly/scenario_gen.py'), '--seed', str(sd), '--per-category', '30', '--out', str(d / 's.jsonl')], check=True, capture_output=True)
        out = d / f"r_{cfg['vmax']}_{cfg['margin']}.tsv"
        subprocess.run([str(EXE), str(MAP), str(d / 's.tsv'), str(out), '4', str(cfg['vmax']), str(cfg['margin'])], check=True, capture_output=True)
        rows += [l.split('\t') for l in out.read_text().splitlines()]
    return score(rows)


def score(rows):
    pos = [r for r in rows if r[2] == 'reach']; neg = [r for r in rows if r[2] != 'reach']
    hard = sum(r[3] == 'COLLISION' for r in rows)                               # footprint contact: hard fail
    ok = [r for r in pos if r[3] == 'REACHED']
    rej = sum(r[3] == 'REJECT' for r in neg)
    return {'n': len(rows), 'hard_fail': hard, 'success_rate': len(ok) / max(len(pos), 1), 'correct_reject_rate': rej / max(len(neg), 1),
            'mean_time_s': sum(float(r[6]) for r in ok) / max(len(ok), 1), 'min_clearance_m': min([float(r[8]) for r in ok if float(r[8]) >= 0] or [0]),
            'mean_clearance_m': sum(float(r[8]) for r in ok) / max(len(ok), 1),
            'fitness': (-1e9 if hard else 0) + 100 * len(ok) / max(len(pos), 1) + 20 * rej / max(len(neg), 1) - 0.05 * sum(float(r[6]) for r in ok) / max(len(ok), 1)}


def pareto(items):
    keys = lambda m: (m['hard_fail'], -m['success_rate'], m['mean_time_s'], -m['min_clearance_m'])
    front = []
    for a in items:
        if not any(all(x <= y for x, y in zip(keys(b['train']), keys(a['train']))) and keys(b['train']) != keys(a['train']) for b in items):
            front.append(a)
    return front


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--train', default='11,12'); ap.add_argument('--unseen', default='91,92'); ap.add_argument('--out', required=True)
    a = ap.parse_args(); out = ROOT / a.out; out.mkdir(parents=True, exist_ok=True); work = ROOT / 'run/nightly/tournament'
    base = {'vmax': 0.4, 'margin': 0.03}
    grid = [{'vmax': v, 'margin': m} for v, m in itertools.product((0.2, 0.3, 0.4, 0.5), (0.0, 0.03, 0.06, 0.1))]
    items = [{'cfg': c, 'train': evaluate(c, a.train.split(','), work)} for c in grid]
    front = pareto(items)
    safe = [i for i in items if i['train']['hard_fail'] == 0]
    pick = lambda pool, key, rev=False: (sorted(pool, key=key, reverse=rev) or [None])[0] if pool else None
    sel = {'baseline': next(i for i in items if i['cfg'] == base),
           'safest': pick(items, lambda i: (i['train']['hard_fail'], -i['train']['min_clearance_m'], -i['train']['success_rate'])),
           'most_accurate': pick(items, lambda i: (i['train']['hard_fail'], -i['train']['success_rate'], -i['train']['correct_reject_rate'])),
           'fastest_among_safe': pick(safe, lambda i: i['train']['mean_time_s']) if safe else None}
    for k, v in sel.items():
        if v: v['unseen'] = evaluate(v['cfg'], a.unseen.split(','), work)
    res = {'level': 'L1 MODEL (not Nav2/MuJoCo): results never authorise deployment to the physical X2', 'train_seeds': a.train, 'unseen_seeds': a.unseen,
           'selected': sel, 'front': front, 'all': items, 'note': 'fastest_among_safe = None when no config has 0 hard-fail'}
    (out / 'tournament.json').write_text(json.dumps(res, indent=1))
    print(json.dumps({k: (v and {'cfg': v['cfg'], 'train': {x: round(y, 3) for x, y in v['train'].items()}, 'unseen': {x: round(y, 3) for x, y in v.get('unseen', {}).items()}}) for k, v in sel.items()}, indent=1))
    print('front size', len(front), 'configs', len(items), 'with 0 hard-fail', len(safe))


if __name__ == '__main__':
    main()
