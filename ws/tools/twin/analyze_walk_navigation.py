#!/usr/bin/env python3
"""Compare action success to true endpoint accuracy and planner endpoint quantization."""
import argparse,json,math
from pathlib import Path
from parallel_turn_sweep import write_json

def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def main():
    ap=argparse.ArgumentParser();ap.add_argument('runs',type=Path,nargs='+');a=ap.parse_args()
    for run in a.runs:
        journey=json.loads((run/'journey.json').read_text());session=json.loads((run/'session.json').read_text());trace=[json.loads(s) for s in (run/'trace.jsonl').read_text().splitlines()]
        legs=[]
        for leg,finish in [('outbound','outbound_settled'),('return','complete')]:
            begin=next((x for x in journey['events'] if x.get('state')==leg),None)
            end=next((x for x in journey['events'] if x.get('state') in ([finish,'completed_outside_tolerance'] if leg=='return' else [finish])),None)
            if begin is None:continue
            record=dict(leg=leg,action_and_settle_completed=end is not None)
            if end:
                target=begin['goal'];sample=min(trace,key=lambda x:abs(x['t']-end['time']));estimate=sample['estimate'];path=sample['controller_paths'].get('nav2/plan',{});endpoint=path.get('last') if path.get('frame')=='map' else None
                record.update(true_xy_error=distance(end['truth'],target),true_yaw_error=abs(math.atan2(math.sin(end['truth'][2]-target[2]),math.cos(end['truth'][2]-target[2]))),
                              estimated_xy_error=distance(estimate,target) if estimate else None,planner_endpoint_offset=distance(endpoint,target) if endpoint else None,
                              estimate_to_planner_end=distance(estimate,endpoint) if estimate and endpoint else None)
                record['true_precision_passed']=record['true_xy_error']<=.1 and record['true_yaw_error']<=.1
            legs.append(record)
        commands=[x['mc_command'] for x in trace if x.get('mc_command')]
        output=dict(state=session['state'],legs=legs,max_forward_command=max([abs(c[0]) for c in commands],default=0),max_angular_command=max([abs(c[2]) for c in commands],default=0),reverse_command_seen=any(c[0]<-.001 for c in commands))
        write_json(run/'walk_accuracy_audit.json',output);print(run.name,json.dumps(output))
if __name__=='__main__':main()
