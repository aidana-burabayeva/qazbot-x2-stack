#!/usr/bin/env python3
"""Offline analysis of one operator session (events.jsonl + trace.jsonl written by operator_panel.py). Pure python + numpy/scipy.

Reports only what was measured. Nothing is promoted to SIM_VERIFIED: the session is interactive and never qualifies.
`nightly_feedback` is filled only for objective defect signatures (an aborted goal the operator did not cancel, a stall, a
localization jump); a wrongly placed seed or an operator cancel is NOT a defect.
"""
import json
import math
from pathlib import Path
import sys

ROBOT_RADIUS = 0.454            # twin.yaml / nav_full robot_radius (read-only here)
STALL_SPEED = 0.03              # m/s
STALL_CMD = 0.10                # commanded m/s that should have produced motion
STALL_MIN_S = 5.0
GOAL_TOL = 0.35                 # .env.lab TWIN_XY_TOL (X2 actual xy_goal_tolerance)
TERMINAL = {'SUCCEEDED', 'CANCELED', 'ABORTED'}


def read_jsonl(p):
    out = []
    if Path(p).exists():
        for ln in Path(p).read_text().splitlines():
            try:
                out.append(json.loads(ln))
            except ValueError:
                pass
    return out


def clearance_fn(map_yaml):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ws/src/navigation'))
    import numpy as np
    import loc_trust as L
    grid = L.OccGrid.from_pgm(str(map_yaml))
    return lambda x, y: float(grid.dist_at(np.array([[x, y]]))[0]) - ROBOT_RADIUS


def speed_norm(cmd):
    return math.hypot(cmd[0], cmd[1]) if cmd else 0.0


def analyze(ev_dir, map_yaml=None):
    ev_dir = Path(ev_dir)
    events, trace = read_jsonl(ev_dir / 'events.jsonl'), read_jsonl(ev_dir / 'trace.jsonl')
    clr = clearance_fn(map_yaml) if map_yaml else None
    rep = dict(trace_rows=len(trace), events=len(events))
    # ---- seeds
    seeds = []
    for e in events:
        if e['kind'] == 'SEED':
            seeds.append(dict(t=e['t'], pose=e['pose'], truth=e.get('truth'), diag=None, ready_after_s=None))
        elif e['kind'] == 'SEED_DIAG' and seeds:
            seeds[-1]['diag'] = {k: v for k, v in e.items() if k not in ('t', 'kind')}
    for s in seeds:
        if s['truth']:
            s['seed_error_vs_truth_m'] = round(math.hypot(s['pose'][0] - s['truth'][0], s['pose'][1] - s['truth'][1]), 3)
        nxt = next((r for r in trace if r['t'] > s['t'] and r.get('ready')), None)
        s['ready_after_s'] = round(nxt['t'] - s['t'], 2) if nxt else None
    rep['seeds'] = seeds
    # ---- trust
    changes = [e for e in events if e['kind'] == 'TRUST_CHANGE']
    lost = [dict(t=e['t'], reasons=e['status'].get('reasons')) for e in changes if e['status'] and not e['status'].get('ready')]
    rep['trust'] = dict(changes=len(changes), not_ready_events=len(lost), reasons=sorted({r for l in lost for r in (l['reasons'] or [])}),
                        t5_jump_events=[l for l in lost if any(str(r).startswith('T5') for r in (l['reasons'] or []))])
    # ---- goals
    goals = [e for e in events if e['kind'] == 'GOAL']
    nav = [e for e in events if e['kind'] == 'NAV_STATUS']
    out = []
    for i, g in enumerate(goals):
        t_end = goals[i + 1]['t'] if i + 1 < len(goals) else float('inf')
        term = next((n for n in nav if g['t'] <= n['t'] < t_end and n['status'] in TERMINAL), None)
        rows = [r for r in trace if g['t'] <= r['t'] <= (term['t'] if term else min(t_end, (trace[-1]['t'] if trace else g['t'])))]
        truth = [r['truth'] for r in rows if r.get('truth')]
        path_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(truth, truth[1:]))
        final = truth[-1] if truth else None
        d = dict(t=g['t'], goal=g['goal'], start_truth=g.get('truth'), trust_ready_at_goal=g.get('trust_ready'),
                 nav2_outcome=(term['status'] if term else 'NO_TERMINAL_STATUS'), duration_s=round(rows[-1]['t'] - rows[0]['t'], 2) if rows else None,
                 path_length_m=round(path_len, 2), final_truth=final,
                 final_distance_to_goal_m=round(math.hypot(final[0] - g['goal'][0], final[1] - g['goal'][1]), 3) if final else None)
        d['reached_within_tolerance'] = d['final_distance_to_goal_m'] is not None and d['final_distance_to_goal_m'] <= GOAL_TOL
        if clr and truth:
            cl = [clr(p[0], p[1]) for p in truth[::3]]
            d['min_clearance_m'] = round(min(cl), 3)
            d['min_clearance_note'] = 'truth pose to nearest occupied map cell minus robot_radius 0.454; MuJoCo contact forces are not published'
        # stalls: commanded motion but measured speed ~0 for >= STALL_MIN_S, goal active
        stalls, start = [], None
        for r in rows:
            moving_cmd = speed_norm(r.get('cmd_auth')) > STALL_CMD or speed_norm(r.get('cmd_nav')) > STALL_CMD
            bad = r.get('v') is not None and r['v'] < STALL_SPEED and moving_cmd and r.get('busy')
            if bad and start is None:
                start = r
            if (not bad or r is rows[-1]) and start is not None:
                if r['t'] - start['t'] >= STALL_MIN_S:
                    p = start.get('truth') or [None, None, None]
                    stalls.append(dict(t=start['t'], duration_s=round(r['t'] - start['t'], 1), truth=p,
                                       distance_to_goal_m=round(math.hypot(p[0] - g['goal'][0], p[1] - g['goal'][1]), 2) if p[0] is not None else None,
                                       clearance_m=round(clr(p[0], p[1]), 3) if (clr and p[0] is not None) else None, cmd_nav=start.get('cmd_nav'),
                                       cmd_auth=start.get('cmd_auth'), mc_cmd=start.get('mc_cmd'), amcl=start.get('amcl')))
                start = None
        d['stalls'] = stalls
        out.append(d)
    rep['goals'] = out
    # ---- measured motion
    speeds = [r['v'] for r in trace if r.get('v') is not None]
    rep['motion'] = dict(max_speed_mps=round(max(speeds), 3) if speeds else None, mean_speed_when_busy=None)
    busy = [r['v'] for r in trace if r.get('v') is not None and r.get('busy')]
    if busy:
        rep['motion']['mean_speed_when_busy'] = round(sum(busy) / len(busy), 3)
    # ---- Nightly feedback: objective signatures only
    fb = {}
    for g in out:
        if g['stalls']:
            s = g['stalls'][0]
            fb = dict(component='navigation_stall', nav2_action_outcome=g['nav2_outcome'] if g['nav2_outcome'] not in ('SUCCEEDED',) else None,
                      error=(f"operator session stall: commanded motion but speed < {STALL_SPEED} m/s for {s['duration_s']} s at truth "
                             f"({s['truth'][0]:.2f}, {s['truth'][1]:.2f}), {s['distance_to_goal_m']} m from goal, clearance {s['clearance_m']} m"),
                      stalls=g['stalls'])
            break
        if g['nav2_outcome'] == 'ABORTED':
            fb = dict(component='nav2_action', nav2_action_outcome='ABORTED', error='Nav2 aborted an operator goal (no operator cancel recorded)')
            break
    if rep['trust']['t5_jump_events'] and not fb:
        fb = dict(component='localization_jump', localization_jumps=rep['trust']['t5_jump_events'][:5])
    rep['nightly_feedback'] = {k: v for k, v in fb.items() if v is not None}
    rep['summary'] = dict(seeds=len(seeds), seeds_not_fitting=sum(1 for s in seeds if (s['diag'] or {}).get('verdict') == 'DOES_NOT_FIT'),
                          goals=len(out), goals_succeeded=sum(1 for g in out if g['nav2_outcome'] == 'SUCCEEDED'),
                          goals_canceled=sum(1 for g in out if g['nav2_outcome'] == 'CANCELED'), stalls=sum(len(g['stalls']) for g in out))
    return rep


if __name__ == '__main__':
    print(json.dumps(analyze(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None), indent=1, default=str))
