"""Bounded passive monitor for the physical RViz workflow. No publishers/clients.

Keeps one DDS participant alive so status checks do not repeatedly join domain0.
Writes only under robot control_backend/docs; never sends initialpose/goals.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from aimdk_msgs.msg import McCommonState

ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--duration',type=float,default=600)
a=ap.parse_args();root=(Path.home()/'control_backend/docs').resolve();out=a.out.resolve()
if root not in out.parents or not 10<=a.duration<=900:raise SystemExit('bounded duration and robot docs output required')
out.mkdir(parents=True,exist_ok=False)
stop=[False]
signal.signal(signal.SIGINT,lambda *_:stop.__setitem__(0,True));signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
rclpy.init();n=rclpy.create_node('qazbot_v13_readiness_ro',enable_rosout=False,start_parameter_services=False)
state={};received={};odom={};ages={};edges={};events=(out/'events.jsonl').open('w');refs=[]
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def event(kind,payload):
 events.write(json.dumps(dict(received_real=time.time(),kind=kind,data=payload))+'\n');events.flush()
def status(key,m):state[key]=json.loads(m.data);received[key]=time.monotonic()
def mc(m):
 state['mc']=dict(source=m.input_source.name,body=m.body_status.value,fsm=m.fsm_state.current_state,action=m.action_info.action_desc)
 received['mc']=time.monotonic()
def pose(kind,m):
 p=m.pose.pose if kind in ['amcl','initialpose'] else m.pose;q=p.orientation
 d=dict(stamp=stamp(m),frame=m.header.frame_id,pose=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
 state[kind]=d;received[kind]=time.monotonic();event(kind,d)
def on_odom(key,m):
 v=m.twist.twist;p=m.pose.pose.position
 odom[key]=dict(stamp=stamp(m),frame=m.header.frame_id,child=m.child_frame_id,
  xy=[p.x,p.y],speed=math.hypot(v.linear.x,v.linear.y),yaw_rate=v.angular.z)
 ages.setdefault(key,deque(maxlen=300)).append(time.time()-stamp(m))
def tf(m):
 for t in m.transforms:edges[t.header.frame_id+'->'+t.child_frame_id]=stamp(t)
for key,topic in [('trust','/qazbot_d1_map/localization_trust/status'),('adapter','/qazbot_d1_map/motion_adapter/status'),
 ('coordinator','/qazbot_d1_map/goal_coordinator/status'),('native_status','/qazbot_native/status')]:
 refs.append(n.create_subscription(String,topic,lambda m,k=key:status(k,m),qos_profile_sensor_data))
refs.append(n.create_subscription(McCommonState,'/aima/mc/common/state',mc,qos_profile_sensor_data))
for key,topic in [('native','/qazbot_native/odom'),('lio','/qazbot_lio/odom_level')]:
 refs.append(n.create_subscription(Odometry,topic,lambda m,k=key:on_odom(k,m),qos_profile_sensor_data))
refs.append(n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/amcl_pose',lambda m:pose('amcl',m),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
refs.append(n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/initialpose',lambda m:pose('initialpose',m),qos_profile_sensor_data))
refs.append(n.create_subscription(PoseStamped,'/qazbot_d1_map/goal_pose',lambda m:pose('goal_pose',m),qos_profile_sensor_data))
refs.append(n.create_subscription(TFMessage,'/qazbot_d1_map/tf',tf,qos_profile_sensor_data))
start=time.monotonic();written=0.;previous=None
try:
 while not stop[0] and time.monotonic()-start<a.duration:
  rclpy.spin_once(n,timeout_sec=.05)
  now=time.monotonic()
  if now-written<.5:continue
  written=now
  snapshot=dict(t_real=time.time(),duration_s=now-start,state=state,
   status_receive_ages_s={k:now-v for k,v in received.items()},odom=odom,tf_edges=edges,
   odom_age_p50_p95_p99_max_s={k:np.percentile(list(v),[50,95,99,100]).tolist() for k,v in ages.items()})
  summary={k:state.get(k) for k in ['mc','initialpose']}
  summary['trusted']=state.get('trust',{}).get('trusted');summary['ready']=state.get('coordinator',{}).get('ready')
  summary['reasons']=state.get('trust',{}).get('reasons');summary['adapter_live']=state.get('adapter',{}).get('live')
  text=json.dumps(summary,sort_keys=True)
  if text!=previous:event('state_change',summary);previous=text
  tmp=out/'latest.json.tmp';tmp.write_text(json.dumps(snapshot,indent=2));os.replace(tmp,out/'latest.json')
 print(json.dumps(dict(out=str(out),duration_s=time.monotonic()-start,initialpose_observed='initialpose' in state)))
finally:
 events.close();n.destroy_node();rclpy.shutdown()
