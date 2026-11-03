#!/usr/bin/env python3
"""Finite read-only body-pose verification against live sensors, private outputs."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

root=Path(__file__).resolve().parent.parent
out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
rclpy.init();node=rclpy.create_node('qazbot_body_shadow_verifier')
poses=[];source=set();states=[];edges=set();procs=[];logs=[]
def key(stamp):return stamp.sec*10**9+stamp.nanosec
def pose(msg):poses.append([key(msg.header.stamp),msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
def status(msg):states.append(json.loads(msg.data))
def tf(msg):
    for t in msg.transforms:edges.add((t.header.frame_id,t.child_frame_id))
node.create_subscription(PoseStamped,'/qazbot_body_shadow/pose',pose,100)
node.create_subscription(Odometry,'/qazbot_lio/odometry',lambda m:source.add(key(m.header.stamp)),qos_profile_sensor_data)
node.create_subscription(String,'/qazbot_body_shadow/status',status,100)
node.create_subscription(TFMessage,'/qazbot_body_shadow/tf',tf,100)
node.create_subscription(TFMessage,'/qazbot_body_shadow/tf_static',tf,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def start(name,args):
    log=(out/(name+'.log')).open('w');logs.append(log)
    p=subprocess.Popen(['nice','-n','10']+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    procs.append(p)
try:
    start('bridge',['python3',str(root/'lio/level_bridge.py'),'--ros-args','-r','__node:=qazbot_body_shadow_level',
        '-r','/Odometry:=/qazbot_lio/odometry','-r','/cloud_registered:=/qazbot_lio/cloud_registered',
        '-r','/scan:=/qazbot_body_shadow/scan','-r','/odom_level:=/qazbot_body_shadow/odom_level',
        '-r','/tf:=/qazbot_body_shadow/tf','-r','/tf_static:=/qazbot_body_shadow/tf_static',
        '-p','range_max:=8.0','-p','scan_every:=2'])
    start('adapter',['python3',str(root/'navigation/body_pose_adapter.py'),'--ros-args','-r','__node:=qazbot_body_shadow_adapter',
        '-p','output_tf:=/qazbot_body_shadow/tf','-p','output_pose:=/qazbot_body_shadow/pose',
        '-p','output_status:=/qazbot_body_shadow/status','-p','level_tf_static:=/qazbot_body_shadow/tf_static'])
    begin=time.monotonic()
    while time.monotonic()-begin<20:
        rclpy.spin_once(node,timeout_sec=.1)
        if any(p.poll() is not None for p in procs):raise RuntimeError('Shadow child exited')
    violations={}
    for topic in ['/tf','/cmd_vel','/aima/mc/locomotion/velocity']:
        violations[topic]=[i.node_name for i in node.get_publishers_info_by_topic(topic) if i.node_name.startswith('qazbot_body_shadow')]
    result={'duration_s':time.monotonic()-begin,'poses':len(poses),'source_messages':len(source),
        'unmatched_pose_stamps':[p[0] for p in poses if p[0] not in source],
        'nonmonotonic_pose_stamps':sum(b[0]<=a[0] for a,b in zip(poses,poses[1:])),
        'max_abs_pose_z':max([abs(p[3]) for p in poses],default=None),
        'tf_edges':sorted(edges),'last_status':states[-1] if states else None,
        'forbidden_publishers':violations,'qualification':'Body frame/transport only; not a position-accuracy or MC execution test.'}
    result['passed']=bool(poses) and not result['unmatched_pose_stamps'] and result['nonmonotonic_pose_stamps']==0 and result['max_abs_pose_z']==0 and not any(violations.values()) and ('odom','camera_init') in edges and ('odom','x2_base_footprint') in edges
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise RuntimeError('Body shadow verification failed')
finally:
    for p in reversed(procs):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in reversed(procs):
        if p.poll() is None:p.wait(timeout=15)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
