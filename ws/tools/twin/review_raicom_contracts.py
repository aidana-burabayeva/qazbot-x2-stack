#!/usr/bin/env python3
"""Offline review probes of selected, inspected methods. No ROS imports or nodes.
The external snapshot is data; only explicitly selected AST definitions run with local mocks.
"""
import argparse,ast,copy,json,math
from enum import Enum
from pathlib import Path
from types import SimpleNamespace as S
ROOT=Path(__file__).resolve().parents[3]
SNAPSHOT=ROOT/'vendor/review/raicom2026-agibot-X2/a97761820b398b6cab12b632ce9f313c394eeb7e'
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()

def selected(path,names,namespace):
    tree=ast.parse(path.read_text());nodes=[]
    for item in tree.body:
        if isinstance(item,(ast.FunctionDef,ast.ClassDef)) and item.name in names:nodes.append(item)
        elif isinstance(item,ast.ClassDef):
            nodes.extend(n for n in item.body if isinstance(n,ast.FunctionDef) and n.name in names)
    if {n.name for n in nodes}!=set(names):raise RuntimeError('snapshot definitions changed')
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

def pose_message():return S(header=S(stamp=None,frame_id=''),pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.)))
clock=S(now=lambda:S(to_msg=lambda:S(sec=100,nanosec=0)))
logger=S(info=lambda *args,**kw:None,warn=lambda *args,**kw:None)
nav=selected(SNAPSHOT/'src/race_task/race_task/task1_pose_nav.py', ['NavState','normalize_angle','forced_direction_yaw_error','apply_min_threshold','clamp_abs','control_loop'],dict(Enum=Enum,math=math,Twist=lambda:S(linear=S(x=0.),angular=S(z=0.))))
events=[]
fake=S(state=nav['NavState'].FINAL_ALIGN,pose=object(),goal_received=True,target_x=0.,target_y=0.,target_yaw=0.,goal_xy_tolerance=.25,goal_yaw_tolerance=.35,final_align_turn_direction='left',get_clock=lambda:clock,get_logger=lambda:logger,pose_is_fresh=lambda:True,get_current_xy_yaw=lambda:(2.,0.,0.),publish_stop=lambda:events.append('stop'),publish_reached_once=lambda:events.append('reached'))
nav['control_loop'](fake)
assert 'reached' in events
published=[]
bridge=selected(SNAPSHOT/'src/race_task/race_task/tf_pose_bridge.py',['timer_callback'],dict(rclpy=S(time=S(Time=lambda:None),duration=S(Duration=lambda **kw:None)),PoseStamped=pose_message,TransformException=RuntimeError))
cached=S(header=S(stamp=S(sec=10,nanosec=0)),transform=S(translation=S(x=0.,y=0.,z=0.),rotation=S(x=0.,y=0.,z=0.,w=1.)))
bridge['timer_callback'](S(tf_buffer=S(lookup_transform=lambda *args,**kw:cached),global_frame='map',base_frame='base_link',lookup_timeout_sec=.05,get_clock=lambda:clock,pose_pub=S(publish=published.append),get_logger=lambda:logger))
assert published[0].header.stamp.sec==100
odom=selected(SNAPSHOT/'src/race_task/race_task/robot_odom_bridge.py',['timer_callback'],dict(copy=copy,Odometry=lambda:S(),TransformStamped=lambda:S()))
odom_out=[];old=S(header=S(stamp=S(sec=10,nanosec=0),frame_id='leg_odom'),pose=S(),twist=S())
odom['timer_callback'](S(latest_msg=old,get_clock=lambda:clock,odom_frame_id='leg_odom',base_frame_id='lidar_imu_chest_front',pub=S(publish=odom_out.append),publish_tf=False))
assert odom_out[0].header.stamp.sec==100
result=dict(snapshot_commit='a97761820b398b6cab12b632ce9f313c394eeb7e',scope='Selected method probes with mocks, no ROS/MC or hardware execution',findings=dict(
 final_alignment=dict(actual_distance_m=2.,configured_xy_tolerance_m=.25,events=events),
 tf_retimestamp=dict(cached_source_sec=10,published_sec=published[0].header.stamp.sec),
 odom_retimestamp=dict(cached_source_sec=10,published_sec=odom_out[0].header.stamp.sec),
 command_floor=dict(requested=.01,minimum=.2,maximum=.3,output=nav['apply_min_threshold'](.01,.2,.3)),
 cap_order_with_invalid_parameters=dict(requested=.01,minimum=.2,maximum=.1,output=nav['apply_min_threshold'](.01,.2,.1)),
 forced_left=dict(shortest_error_rad=-.4,command_error_rad=nav['forced_direction_yaw_error'](-.4,0.,'left'))))
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
