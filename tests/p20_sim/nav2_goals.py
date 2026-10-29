#!/usr/bin/env python3
"""Full Nav2/Coordinator/P20 integration, truth localisation; simulation-only and not AMCL qualification."""
import argparse
import json
import math
import os
import subprocess
from pathlib import Path
import rclpy
from x2_twin_scenarios.harness import Harness, SUCCEEDED
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import parameter_value_to_python

if os.environ.get('TWIN_SANDBOX')!='1' or os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('SIM only')
ap=argparse.ArgumentParser();ap.add_argument('--out',default='nav2_truth_p20');ap.add_argument('--reuse-registration',action='store_true');ap.add_argument('--only-cw',action='store_true');ap.add_argument('--only-ccw',action='store_true');ap.add_argument('--record-phases',action='store_true');ap.add_argument('--xy-tol',type=float);ap.add_argument('--yaw-tol',type=float);args=ap.parse_args()
rclpy.init();h=Harness();result={'scope':__doc__,'goals':[]}
out=Path('/results')/args.out;out.mkdir(exist_ok=False)
recorder=None;recorder_log=None
try:
    h.wait(lambda:h.pose is not None and h.state['adapter'].get('registration')==('registered' if args.reuse_registration else 'not_requested'),120,'plant and adapter')
    h.wait(lambda:h.state['trust'].get('trusted') is True,30,'truth localisation')
    if not args.reuse_registration:
        r=h.trigger('motion_adapter/register');assert r.success,r.message
    h.wait(lambda:h.state['adapter'].get('registration')=='registered',20,'registration')
    h.wait(lambda:h.state['coord'].get('ready',False),120,'coordinator ready')
    names=['goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance','goal_checker.stateful',
           'FollowPath.use_rotate_to_heading','FollowPath.rotate_to_heading_angular_vel','FollowPath.use_collision_detection']
    overrides={'goal_checker.xy_goal_tolerance':args.xy_tol,'goal_checker.yaw_goal_tolerance':args.yaw_tol}
    overrides={k:v for k,v in overrides.items() if v is not None}
    if overrides:  # explicit A/B condition: the robot ran xy 0.20 / yaw 0.35 via launch args, the twin launch does not pass them
        request=SetParameters.Request()
        request.parameters=[ParameterMsg(name=k,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(v))) for k,v in overrides.items()]
        applied=h.call(SetParameters,'nav2/controller_server/set_parameters',request)
        assert all(r.successful for r in applied.results),applied.results
        result['parameter_overrides']=overrides
    request=GetParameters.Request();request.names=names
    response=h.call(GetParameters,'nav2/controller_server/get_parameters',request)
    result['controller_parameters']={name:parameter_value_to_python(value) for name,value in zip(names,response.values)}
    if args.record_phases:
        config=out/'recorder.yaml'
        config.write_text('p20_timing_recorder:\n  ros__parameters:\n    output_dir: '+str(out/'recorder')+'\n')
        recorder_log=(out/'recorder.log').open('w')
        recorder=subprocess.Popen(['/timing/build/timing_recorder','--ros-args','--params-file',str(config)],stdout=recorder_log,stderr=recorder_log)
        h.wait(lambda:(out/'recorder/motion.csv').exists() and (out/'recorder/motion.csv').stat().st_size>0,15,'command recorder ready')
    start=tuple(h.pose)
    targets=[(start[0],start[1]+.5,0.)] if args.only_cw else [(start[0]+.5,start[1],math.pi/2),(start[0]+.5,start[1]+.5,0.)]
    if args.only_ccw:targets=targets[:1]
    for target in targets:
        h.wait(lambda:h.state['coord'].get('ready',False),30,'coordinator settled and ready for next goal')
        begin=h.now();begin_ros_ns=h.get_clock().now().nanoseconds;handle=h.send_goal(*target);assert handle.accepted
        status=h.finish(handle)
        result_ros_ns=h.get_clock().now().nanoseconds
        at_result_pose=tuple(h.pose)
        h.wait(lambda:h.state['coord'].get('ready',False),30,'goal drained to zero and coordinator ready')
        zero_ros_ns=h.get_clock().now().nanoseconds
        h.spin(2.)  # measured post-stop settling interval, not a message-delivery sleep
        settled_ros_ns=h.get_clock().now().nanoseconds
        pose=tuple(h.pose)
        error=math.remainder(pose[2]-target[2],2*math.pi)
        result['goals'].append(dict(target=target,status=status,duration=h.now()-begin,pose=pose,
                                    xy_error=math.hypot(pose[0]-target[0],pose[1]-target[1]),yaw_error=error,
                                    begin_ros_ns=begin_ros_ns,result_ros_ns=result_ros_ns,zero_ros_ns=zero_ros_ns,settled_ros_ns=settled_ros_ns,at_result_pose=at_result_pose,
                                    coordinator=dict(h.state['coord'])))
        if status!=SUCCEEDED:break
    result['result']='PASS' if len(result['goals'])==len(targets) and all(g['status']==SUCCEEDED and abs(g['yaw_error'])<=.35 and g['xy_error']<=.20 for g in result['goals']) else 'FAIL'
except Exception as e:result.update(result='FAIL',error=repr(e),states=h.state)
finally:
    if recorder is not None:
        recorder.terminate()
        try:recorder.wait(timeout=10)
        except subprocess.TimeoutExpired:recorder.kill();recorder.wait()
    if recorder_log is not None:recorder_log.close()
    (out/'result.json').write_text(json.dumps(result,indent=2,default=str))
    (out/'trace.json').write_text(json.dumps(h.trace))
    print(json.dumps(result,indent=2,default=str));h.destroy_node();rclpy.shutdown()

if result['result'] != 'PASS':
    raise SystemExit(1)
