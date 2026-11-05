#!/usr/bin/env python3
"""Score completed random trials, including failures and measured motion."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'ws/src/navigation'))
from loc_trust import OccGrid,wrap
ap=argparse.ArgumentParser();ap.add_argument('batch',type=Path);ap.add_argument('targets',type=Path);ap.add_argument('--prefix',default='random_base');a=ap.parse_args()
targets=json.loads(a.targets.read_text());grid=OccGrid.from_pgm(str(ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml'));rows=[]
for target in targets['targets']:
    name=f"{a.prefix}_{target['index']:02d}";file=a.batch/(name+'.json')
    if not file.exists():continue
    result=json.loads(file.read_text());session=ROOT/'tests/integration_results'/name
    trace=[json.loads(line) for line in (session/'trace.jsonl').read_text().splitlines()]
    moving=[r for r in trace if r.get('truth') and r.get('velocity')];p=np.array([r['truth'] for r in moving]);v=np.array([r['velocity'] for r in moving]);t=np.array([r['t'] for r in moving]);final=p[-1]
    dt=np.diff(t);angular=np.abs([wrap(z) for z in np.diff(p[:,2])]);turn=longest=0.
    for i in range(1,len(moving)):
        if abs(v[i,0])<.02 and abs(v[i,1])>.1:turn+=dt[i-1];longest=max(longest,turn)
        else:turn=0.
    state=result['state']
    if result['returncode'] and state in ('outbound','return','settling','settling_return'):state='deadline_or_runner_error'
    row=dict(index=target['index'],goal=target['goal'],category=target['category'],state=state,passed=result.get('passed',False),
             duration_s=result['duration_s'],final_goal_xy_m=math.dist(final[:2],target['goal'][:2]),
             final_goal_yaw_rad=abs(wrap(final[2]-target['goal'][2])),
             outbound_error=result.get('endpoint',{}).get('outbound_error'),return_xy_m=result.get('endpoint',{}).get('xy_error'),
             return_yaw_rad=result.get('endpoint',{}).get('yaw_error'),
             collision_guard_samples=sum(bool(r.get('collision')) for r in trace),
             min_center_to_occupied_cell_m=float(grid.dist_at(p[:,:2]).min()),
             max_linear_m_s=float(np.abs(v[:,0]).max()),max_angular_rad_s=float(np.abs(v[:,1]).max()),
             path_length_m=float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),
             total_rotation_rad=float(angular.sum()),longest_stationary_turn_s=longest)
    rows.append(row)
summary=dict(seed=targets['seed'],requested=len(targets['targets']),completed=len(rows),passed=sum(r['passed'] for r in rows),results=rows)
(a.batch/'scored_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
