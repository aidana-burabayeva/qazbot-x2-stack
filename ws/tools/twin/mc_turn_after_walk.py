#!/usr/bin/env python3
"""SIM ONLY: normalized stand -> walk -> zero -> turn, SDK priority 30.
Optional angular kick is an experiment in the official MC simulator, never a production policy.
Records true pelvis, FK torso/feet, commands and MC state. No navigation or localization involved.
"""
import argparse,json,math,os,sys,time
from pathlib import Path
import numpy as np
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):raise RuntimeError('isolated SIM only')
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from aimdk_msgs.msg import McLocomotionVelocity,McInputAction,McCommonState
from aimdk_msgs.srv import SetMcInputSource
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'ws/src/navigation'))
from control_geometry import transform

ap=argparse.ArgumentParser();ap.add_argument('output');ap.add_argument('--angular',type=float,default=-.5);ap.add_argument('--kick',type=float,default=0.);ap.add_argument('--kick-seconds',type=float,default=1.);ap.add_argument('--target-deg',type=float);ap.add_argument('--lead-time',type=float,default=.16);ap.add_argument('--tolerance',type=float,default=.06);ap.add_argument('--turn-seconds',type=float,default=12.);ap.add_argument('--adaptive-finish',action='store_true');ap.add_argument('--adapter-zero-burst',action='store_true');ap.add_argument('--walk-speed',type=float,default=.3);ap.add_argument('--walk-seconds',type=float,default=3.);ap.add_argument('--walk-only',action='store_true');a=ap.parse_args()
if not (0<a.tolerance<=.1 and 1<=a.turn_seconds<=40 and math.isfinite(a.lead_time) and 0<=a.lead_time<=.5 and (a.target_deg is None or math.isfinite(a.target_deg))) or abs(a.angular)>1 or abs(a.kick)>1 or a.kick_seconds<0 or a.kick_seconds>3:raise ValueError('invalid SIM probe limits')
if not (math.isfinite(a.walk_speed) and 0<a.walk_speed<=.3 and math.isfinite(a.walk_seconds) and 1<=a.walk_seconds<=10):raise ValueError('invalid walk limits')
out=Path(a.output)
if out.exists():raise RuntimeError('output exists')
out.mkdir(parents=True)
rclpy.init();n=Node('mc_turn_after_walk');t0=time.monotonic();origin=None;mc={};mc_at=-math.inf
odom=[];cmd=[];states=[];frames=[];graph={};root=None;root_stamp=None;aborted=None
pub=n.create_publisher(McLocomotionVelocity,'/aima/mc/locomotion/velocity',10)
client=n.create_client(SetMcInputSource,'/aimdk_5Fmsgs/srv/SetMcInputSource')
source='sdk_turn_after_walk';priority=30
from turn_regulator import TurnRegulator
controller=None if a.target_deg is None else TurnRegulator(math.radians(a.target_deg),speed=abs(a.angular),lead_time=a.lead_time,tolerance=a.tolerance)
start_yaw=None
zero_left=0
completed_at=None

def now():return time.monotonic()-t0
def on_odom(m):
    global root,root_stamp
    p,q=m.pose.pose.position,m.pose.pose.orientation;v=m.twist.twist
    root=transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);root_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    odom.append((now(),p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z))
def on_state(m):
    global mc,mc_at
    mc=dict(body=m.body_status.value,fsm=m.fsm_state.current_state,source=m.input_source.name,action=m.action_info.action_desc);mc_at=now();states.append(dict(t=now(),**mc))
def on_tf(m):
    for tr in m.transforms:
        p,q=tr.transform.translation,tr.transform.rotation
        graph[tr.child_frame_id]=(tr.header.frame_id,transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    if root is None or not m.transforms:return
    stamp=m.transforms[0].header.stamp;age=abs(root_stamp-(stamp.sec+stamp.nanosec*1e-9))
    if age>.05:return
    row=[now(),age]
    for frame in ('pelvis','torso_link','left_ankle_roll_link','right_ankle_roll_link'):
        chain=[];child=frame
        while child!='pelvis':
            if child not in graph:return
            parent,T=graph[child];chain.append(T);child=parent
        T=root.copy()
        for local in reversed(chain):T=T@local
        row.extend([T[0,3],T[1,3],T[2,3],math.atan2(T[1,0],T[0,0])])
    frames.append(row)
n.create_subscription(Odometry,'/aima/hal/odom/state',on_odom,qos_profile_sensor_data)
n.create_subscription(McCommonState,'/aima/mc/common/state',on_state,qos_profile_sensor_data)
n.create_subscription(TFMessage,'/tf',on_tf,qos_profile_sensor_data)

def send(f,w):
    global zero_left
    if a.adapter_zero_burst:
        if abs(f)+abs(w)>.005:zero_left=5
        elif zero_left>0:zero_left-=1
        else:return
    m=McLocomotionVelocity();m.header.stamp=n.get_clock().now().to_msg();m.source=source;m.forward_velocity=float(f);m.angular_velocity=float(w);pub.publish(m);cmd.append((now(),f,0.,w))
try:
    deadline=time.monotonic()+15
    while not (odom and mc and client.service_is_ready()):
        if time.monotonic()>deadline:raise RuntimeError('no HAL/MC/service')
        rclpy.spin_once(n,timeout_sec=.02)
    if mc['body']!=1 or mc['fsm']!=4:raise RuntimeError('MC not standing '+str(mc))
    req=SetMcInputSource.Request();req.action.value=McInputAction.INPUTACTION_ADD;req.input_source.name=source;req.input_source.priority=priority;req.input_source.timeout=1000;req.request.header.stamp=n.get_clock().now().to_msg()
    f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if f.done() and f.result().response.header.code!=0:
        req.action.value=McInputAction.INPUTACTION_ENABLE;req.request.header.stamp=n.get_clock().now().to_msg();f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if not f.done() or f.result().response.header.code!=0:raise RuntimeError('SDK registration refused')
    start=now();walk_on=start+2;walk_off=walk_on+a.walk_seconds;turn_on=walk_off+4;turn_off=turn_on+a.turn_seconds;end=turn_off+6
    if a.walk_only:end=walk_off+8;turn_on=end+1;turn_off=end+1
    def tick():
        global aborted,start_yaw
        t=now()
        if not odom or t-odom[-1][0]>.15 or t-mc_at>.5 or mc['body']!=1 or mc['fsm'] not in (4,5) or (odom and odom[-1][3]<.45):
            aborted=aborted or dict(t=t,state=mc,height=odom[-1][3] if odom else None,odom_age=t-odom[-1][0] if odom else None,mc_age=t-mc_at,reason='HAL/MC freshness, body mode or height guard')
        if aborted:send(0.,0.)
        elif walk_on<=t<walk_off:send(a.walk_speed,0.)
        elif turn_on<=t<turn_off:
            if controller is None:send(0.,a.kick if a.kick and t<turn_on+a.kick_seconds else a.angular)
            else:
                rows=np.asarray(odom[-200:]);q=rows[:,4:8];heading=np.unwrap(np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2)))
                if start_yaw is None:start_yaw=float(heading[-1])
                actual=math.atan2(root[1,0],root[0,0]);angle=math.atan2(math.sin(actual-start_yaw),math.cos(actual-start_yaw))
                # Targets up to 180 deg: maintain a continuous angle across the branch cut.
                if not hasattr(tick,'last'):tick.last=angle;tick.total=angle
                else:tick.total+=math.atan2(math.sin(angle-tick.last),math.cos(angle-tick.last));tick.last=angle
                recent=rows[:,0]>t-.15
                rate=float(np.polyfit(rows[recent,0],heading[recent],1)[0]) if recent.sum()>=3 else 0.
                send(0.,controller.update(t,tick.total,rate))
        else:send(0.,0.)
    n.create_timer(.02,tick)
    while now()<end:
        rclpy.spin_once(n,timeout_sec=.005)
        if a.adaptive_finish and controller is not None and completed_at is None and (controller.done or controller.failed):
            completed_at=now();turn_off=completed_at;end=completed_at+6.
    np.savez_compressed(out/'data.npz',odom=np.asarray(odom),cmd=np.asarray(cmd),frames=np.asarray(frames),meta=np.array([0.,0.,a.angular,turn_off-turn_on,9.,6.,turn_on,turn_off,end]))
    meta=dict(source=source,priority=priority,walk_command=a.walk_speed,walk_seconds=a.walk_seconds,walk_on=walk_on,walk_off=walk_off,walk_only=a.walk_only,zero_before_turn_s=4.,turn_seconds=turn_off-turn_on,turn_limit_seconds=a.turn_seconds,completed_at=completed_at,adaptive_finish=a.adaptive_finish,zero_policy='five_then_silence' if a.adapter_zero_burst else 'continuous',angular=a.angular,kick=a.kick,kick_seconds=a.kick_seconds,turn_on=turn_on,turn_off=turn_off,aborted=aborted,target_deg=a.target_deg,controller=None if controller is None else vars(controller),odom_samples=len(odom),frame_samples=len(frames),states=states)
    (out/'meta.json').write_text(json.dumps(meta,indent=2));print(json.dumps({k:v for k,v in meta.items() if k!='states'}),flush=True)
finally:
    for _ in range(5):
        send(0.,0.)
        try:rclpy.spin_once(n,timeout_sec=.02)
        except Exception:pass
    n.destroy_node();rclpy.try_shutdown()
