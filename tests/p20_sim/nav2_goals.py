#!/usr/bin/env python3
"""Full Nav2/Coordinator/P20 integration, truth localisation; simulation-only and not AMCL qualification."""
import argparse
import json
import math
import os
from pathlib import Path
import rclpy
from x2_twin_scenarios.harness import Harness, SUCCEEDED

if os.environ.get('TWIN_SANDBOX')!='1' or os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('SIM only')
ap=argparse.ArgumentParser();ap.add_argument('--out',default='nav2_truth_p20');ap.add_argument('--reuse-registration',action='store_true');ap.add_argument('--only-cw',action='store_true');args=ap.parse_args()
rclpy.init();h=Harness();result={'scope':__doc__,'goals':[]}
out=Path('/results')/args.out;out.mkdir(exist_ok=False)
try:
    h.wait(lambda:h.pose is not None and h.state['adapter'].get('registration')==('registered' if args.reuse_registration else 'not_requested'),120,'plant and adapter')
    h.wait(lambda:h.state['trust'].get('trusted') is True,30,'truth localisation')
    if not args.reuse_registration:
        r=h.trigger('motion_adapter/register');assert r.success,r.message
    h.wait(lambda:h.state['adapter'].get('registration')=='registered',20,'registration')
    h.wait(lambda:h.state['coord'].get('ready',False),120,'coordinator ready')
    start=tuple(h.pose)
    targets=[(start[0],start[1]+.5,0.)] if args.only_cw else [(start[0]+.5,start[1],math.pi/2),(start[0]+.5,start[1]+.5,0.)]
    for target in targets:
        h.wait(lambda:h.state['coord'].get('ready',False),30,'coordinator settled and ready for next goal')
        begin=h.now();handle=h.send_goal(*target);assert handle.accepted
        status=h.finish(handle)
        pose=tuple(h.pose)
        error=math.remainder(pose[2]-target[2],2*math.pi)
        result['goals'].append(dict(target=target,status=status,duration=h.now()-begin,pose=pose,
                                    xy_error=math.hypot(pose[0]-target[0],pose[1]-target[1]),yaw_error=error,
                                    coordinator=dict(h.state['coord'])))
        if status!=SUCCEEDED:break
    result['result']='PASS' if len(result['goals'])==len(targets) and all(g['status']==SUCCEEDED and abs(g['yaw_error'])<=.35 for g in result['goals']) else 'FAIL'
except Exception as e:result.update(result='FAIL',error=repr(e),states=h.state)
finally:
    (out/'result.json').write_text(json.dumps(result,indent=2,default=str))
    (out/'trace.json').write_text(json.dumps(h.trace))
    print(json.dumps(result,indent=2,default=str));h.destroy_node();rclpy.shutdown()

if result['result'] != 'PASS':
    raise SystemExit(1)
