"""I1 verdict from an A/B pair: summarize.py TAG  (reads /results/i1-TAG-{baseline,candidate}/).

Per-phase count of 'RegulatedPurePursuitController detected collision ahead' in the controller log (ROS wall stamps vs the scenario's
phase windows), the chain checks of both runs and the GO/NO-GO rule:
  GO   = candidate: no hard failure, 'collision ahead' only while the real obstacle is present (phase A) and none in B, D, E;
         baseline: reproduces the bug (stale mark persists, free-route goal refused without motion, collision lines in E).
"""
import json
import re
import sys

tag = sys.argv[1]
R = {}
for mode in ('baseline', 'candidate'):
    d = f'/results/i1-{tag}-{mode}'
    res = json.load(open(f'{d}/results.json'))
    lines = open(f'{d}/stack_collision_lines.txt').read().splitlines()
    stamps = [float(m.group(1)) for m in (re.search(r'\[(\d{10}\.\d+)\]', ln) for ln in lines if 'collision ahead' in ln) if m]
    per = {name: sum(1 for t in stamps if w[0] <= t <= (w[1] or 1e18)) for name, w in res['phases'].items()}
    R[mode] = dict(res=res, per=per, checks={c['name']: c for c in res['checks']})
    print(f'=== {mode}: failed={res["failed"]} hard_failed={res["hard_failed"]}')
    for c in res['checks']:
        print(f'  [{"ok" if c["ok"] else "FAIL"}] {c["name"]}: {str(c["detail"])[:170]}')
    print('  collision-ahead lines per phase:', per)
b, c = R['baseline'], R['candidate']
bug = (b['checks'].get('B2_bug_reproduced_stale_mark_persists', {}).get('ok') and b['checks'].get('E1_bug_reproduced_goal_refused_without_motion', {}).get('ok')
       and b['per'].get('E_free_route', 0) > 0)
fix = (not c['res']['hard_failed'] and c['per'].get('B_removed_in_fov', 0) == 0 and c['per'].get('D_outside_fov', 0) == 0
       and c['per'].get('E_free_route', 0) == 0
       and c['checks'].get('A4_real_obstacle_blocks_motion', {}).get('ok'))
print('\nbaseline reproduces the bug:', bool(bug))
print('candidate passes every regression check:', bool(fix))
print('VERDICT:', 'GO' if (bug and fix) else 'NO-GO')
sys.exit(0 if (bug and fix) else 1)
