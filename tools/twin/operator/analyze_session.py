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
STALL_CMD_W = 0.20              # commanded rad/s (in place turn) that should have produced yaw motion
STALL_W = 0.05                  # rad/s measured
STALL_MIN_S = 5.0
SETTLE_S = 6.0                  # measure the final pose this long after the terminal Nav2 status (coasting)
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


def yaw_rates(trace, win=1.0):
    """Measured yaw rate (rad/s) per row from the ground-truth yaw over +-win s (the panel only stores yaw, not its rate)."""
    rows = [(i, r['t'], r['truth'][2]) for i, r in enumerate(trace) if r.get('truth')]
    out = {}
    j = 0
    for k, (i, t, yaw) in enumerate(rows):
        while rows[j][1] < t - win:
            j += 1
        h = k
        while h + 1 < len(rows) and rows[h + 1][1] <= t + win:
            h += 1
        dt = rows[h][1] - rows[j][1]
        if dt >= win / 2:
            d = math.atan2(math.sin(rows[h][2] - rows[j][2]), math.cos(rows[h][2] - rows[j][2]))
            out[i] = d / dt
    return out


def reachable(map_yaml, start_xy, goal_xy, inflate=ROBOT_RADIUS):
    """Can a disc of radius `inflate` get from start to goal through the map's free cells? (unknown/occupied are blocked)"""
    import numpy as np
    from collections import deque
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ws/src/navigation'))
    import cv2
    import loc_trust as L
    grid = L.OccGrid.from_pgm(str(map_yaml))
    img = np.flipud(cv2.imread(str(Path(map_yaml).parent / 'map.pgm'), cv2.IMREAD_GRAYSCALE))
    ok = (img >= 250) & (grid.dist >= inflate)
    cell = lambda p: (int((p[1] - grid.oy) / grid.res), int((p[0] - grid.ox) / grid.res))
    s0, g0 = cell(start_xy), cell(goal_xy)
    if not (ok[s0] and ok[g0]):
        return False
    seen = np.zeros(ok.shape, bool)
    seen[s0] = True
    q = deque([s0])
    while q:
        y, x = q.popleft()
        if (y, x) == g0:
            return True
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < ok.shape[0] and 0 <= nx < ok.shape[1] and ok[ny, nx] and not seen[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return False


def speed_norm(cmd):
    return math.hypot(cmd[0], cmd[1]) if cmd else 0.0


def analyze(ev_dir, map_yaml=None):
    ev_dir = Path(ev_dir)
    events, trace = read_jsonl(ev_dir / 'events.jsonl'), read_jsonl(ev_dir / 'trace.jsonl')
    rate = yaw_rates(trace)
    for i, r in enumerate(trace):
        if r.get('w') is None and i in rate:
            r['w'] = rate[i]
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
        settle = [r['truth'] for r in trace if r.get('truth') and (term['t'] if term else g['t']) <= r['t'] <= min(t_end, (term['t'] if term else g['t']) + SETTLE_S)]
        final = (settle[-1] if settle else (truth[-1] if truth else None))        # the legged robot coasts after the terminal status
        d = dict(t=g['t'], goal=g['goal'], start_truth=g.get('truth'), trust_ready_at_goal=g.get('trust_ready'),
                 nav2_outcome=(term['status'] if term else 'NO_TERMINAL_STATUS'), duration_s=round(rows[-1]['t'] - rows[0]['t'], 2) if rows else None,
                 path_length_m=round(path_len, 2), final_truth=final,
                 final_distance_to_goal_m=round(math.hypot(final[0] - g['goal'][0], final[1] - g['goal'][1]), 3) if final else None)
        d['expected_reject_by_inflation'] = False
        if map_yaml and d['nav2_outcome'] == 'ABORTED' and d['path_length_m'] < 0.5 and g.get('start_truth'):
            d['expected_reject_by_inflation'] = not reachable(map_yaml, g['start_truth'][:2], g['goal'][:2])
            if d['expected_reject_by_inflation']:
                d['nav2_outcome_class'] = 'SAFE_REJECT'
                d['nav2_outcome_note'] = f'goal unreachable for a disc of radius {ROBOT_RADIUS} m (a doorway narrower than {2 * ROBOT_RADIUS:.3f} m): expected planner refusal, not a defect'
        d['reached_within_tolerance'] = d['final_distance_to_goal_m'] is not None and d['final_distance_to_goal_m'] <= GOAL_TOL
        if clr and truth:
            cl = [clr(p[0], p[1]) for p in truth[::3]]
            d['min_clearance_m'] = round(min(cl), 3)
            d['min_clearance_note'] = 'truth pose to nearest occupied map cell minus robot_radius 0.454; MuJoCo contact forces are not published'
        # stalls: commanded motion but measured speed ~0 for >= STALL_MIN_S, goal active
        stalls, start = [], None
        for r in rows:
            lin_cmd = speed_norm(r.get('cmd_auth')) > STALL_CMD or speed_norm(r.get('cmd_nav')) > STALL_CMD
            ang_cmd = abs((r.get('cmd_auth') or [0, 0, 0])[2]) > STALL_CMD_W or abs((r.get('mc_cmd') or [0, 0, 0])[2]) > STALL_CMD_W
            lin_stall = lin_cmd and r.get('v') is not None and r['v'] < STALL_SPEED
            ang_stall = ang_cmd and not lin_cmd and r.get('w') is not None and abs(r['w']) < STALL_W and (r.get('v') or 0) < STALL_SPEED
            bad = bool(r.get('busy')) and (lin_stall or ang_stall)
            if bad and start is None:
                start = r
            if (not bad or r is rows[-1]) and start is not None:
                if r['t'] - start['t'] >= STALL_MIN_S:
                    p = start.get('truth') or [None, None, None]
                    stalls.append(dict(t=start['t'], duration_s=round(r['t'] - start['t'], 1), truth=p, kind='ANGULAR_NO_YAW' if abs((start.get('mc_cmd') or [0, 0, 0])[2]) > STALL_CMD_W and speed_norm(start.get('cmd_auth')) <= STALL_CMD else 'LINEAR_NO_MOTION',
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
        if g['nav2_outcome'] == 'ABORTED' and not g.get('expected_reject_by_inflation'):
            fb = dict(component='nav2_action', nav2_action_outcome='ABORTED', error='Nav2 aborted an operator goal (no operator cancel recorded)')
            break
    misfit = [s for s in seeds if (s['diag'] or {}).get('verdict') == 'DOES_NOT_FIT' and s['ready_after_s'] is not None]
    rep['trust_admitted_misfitting_seed'] = [dict(pose=s['pose'], seed_score=s['diag']['seed_score'], ready_after_s=s['ready_after_s']) for s in misfit]
    if misfit and not fb:
        fb = dict(component='localization_trust_admission',
                  error=(f"Trust READY {misfit[0]['ready_after_s']} s after an operator seed whose scan fit {misfit[0]['diag']['seed_score']} is below "
                         f"hit_min_abs {misfit[0]['diag']['trust_hit_min_abs']} (go2 admission policy does not gate on scan fit)"))
    if rep['trust']['t5_jump_events'] and not fb:
        fb = dict(component='localization_jump', localization_jumps=rep['trust']['t5_jump_events'][:5])
    rep['nightly_feedback'] = {k: v for k, v in fb.items() if v is not None}
    rep['summary'] = dict(seeds=len(seeds), seeds_not_fitting=sum(1 for s in seeds if (s['diag'] or {}).get('verdict') == 'DOES_NOT_FIT'),
                          goals=len(out), goals_succeeded=sum(1 for g in out if g['nav2_outcome'] == 'SUCCEEDED'),
                          goals_canceled=sum(1 for g in out if g['nav2_outcome'] == 'CANCELED'), stalls=sum(len(g['stalls']) for g in out))
    return rep


if __name__ == '__main__':
    print(json.dumps(analyze(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None), indent=1, default=str))
