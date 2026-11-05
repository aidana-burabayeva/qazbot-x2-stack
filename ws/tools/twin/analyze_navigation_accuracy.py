#!/usr/bin/env python3
"""Separate AMCL error, estimated goal error and true endpoint error.

Reads completed interactive journeys only; never feeds truth back to navigation.
"""
import argparse
import json
import math
from pathlib import Path


def error(pose, target):
    return dict(xy_m=math.dist(pose[:2], target[:2]),
                yaw_rad=abs(math.atan2(math.sin(pose[2]-target[2]), math.cos(pose[2]-target[2]))))


def analyze(directory):
    journey=json.loads((directory/'journey.json').read_text())
    trace=[json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
    legs=[]
    for event in journey['events']:
        if event.get('state') not in ('outbound','return'):continue
        leg=event['state'];target=event['goal']
        samples=[r for r in trace if r['t']>=event['time'] and r['state']==leg and r.get('estimate') and r.get('truth')]
        if not samples:continue
        finish=next((r for r in trace if r['t']>=samples[-1]['t'] and r['state'] in
                     ('settling','settling_return','outbound_failed','return_failed')),samples[-1])
        estimate=finish.get('estimate');truth=finish.get('truth')
        if estimate is None or truth is None:continue
        errors=sorted(error(r['estimate'],r['truth'])['xy_m'] for r in samples)
        legs.append(dict(leg=leg,finish_state=finish['state'],duration_s=finish['t']-event['time'],
                         true_goal_error=error(truth,target),estimated_goal_error=error(estimate,target),
                         localization_error=error(estimate,truth),
                         localization_xy_p95_m=errors[min(len(errors)-1,int(.95*len(errors)))],
                         estimated_xy_within_10cm_at_finish=error(estimate,target)['xy_m']<=.1))
    return dict(name=directory.name,state=next(e['state'] for e in reversed(journey['events']) if 'state' in e),legs=legs)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directories',nargs='+',type=Path);args=ap.parse_args()
    for directory in args.directories:
        result=analyze(directory)
        (directory/'accuracy.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
