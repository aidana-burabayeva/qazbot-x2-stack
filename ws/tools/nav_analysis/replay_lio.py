#!/usr/bin/env python3
"""Finite isolated replay, derived from .89 diagnostic; never joins hardware topics."""
import argparse
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--input', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--dense', action='store_true')
a = p.parse_args()
if not (a.input/'metadata.yaml').is_file() or a.output.exists():
    p.error('Input must be finalized and output must not exist')
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
# Preserve the robot's tested SHM capacity: the default segment cannot hold
# this sensor's PointCloud2 messages. Restrict its UDP transport to loopback.
profile = Path.home()/'.aima/env/ros_dds_configuration.xml'
profile_text=profile.read_text()
addresses=list(ET.fromstring(profile_text).iter('{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}address'))
if not addresses:
    raise RuntimeError('No interface allowlist in robot DDS profile')
for address in addresses:
    profile_text=profile_text.replace('<address>'+address.text+'</address>','<address>127.0.0.1</address>')
a.output.mkdir()
isolated_profile=a.output/'dds_loopback.xml'
isolated_profile.write_text(profile_text)
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(isolated_profile)
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
import rclpy
from rclpy.node import Node
rclpy.init()
n = Node('qazbot_isolated_replay_readiness')
root = Path(__file__).resolve().parent.parent
procs, logs = [], []
raw = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
imu = '/aima/hal/sensor/lidar_chest_front/imu'
remaps = ['-r',raw+':=/replay/lidar','-r',imu+':=/replay/imu']
def start(name, args):
    log = (a.output/(name+'.log')).open('w'); logs.append(log)
    child = subprocess.Popen(['nice','-n','10']+args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    procs.append(child)
    return child
try:
    start('lio',[str(root/'runtime/fastlio_mapping'),'--ros-args','--params-file',str(root/'lio/x2.yaml'),
                 '-p','use_sim_time:=true','-p','publish.dense_publish_en:='+str(a.dense).lower()]+remaps)
    start('bridge',['python3',str(root/'lio/level_bridge.py'),'--ros-args','-p','use_sim_time:=true',
                    '-p','range_max:=8.0','-p','scan_every:=2']+remaps)
    start('recorder',['ros2','bag','record','-o',str(a.output/'bag'),'/cloud_registered','/Odometry','/scan','/odom_level'])
    required=['/replay/lidar','/replay/imu','/cloud_registered','/Odometry','/scan','/odom_level']
    deadline=time.monotonic()+30
    while not all(n.count_subscribers(t)>0 for t in required):
        rclpy.spin_once(n,timeout_sec=.1)
        if any(c.poll() is not None for c in procs):
            raise RuntimeError('Replay child exited before playback')
        if time.monotonic()>deadline:
            raise RuntimeError('Replay graph discovery incomplete; no playback started')
    player=start('player',['python3',str(root/'navigation/matched_player.py'),str(a.input)])
    rc=player.wait(timeout=90)
    if rc:
        raise RuntimeError('Playback failed: '+str(rc))
    print(json.dumps({'playback_exit_code':rc,'dense':a.dense,'domain':87,'hardware_topics_remapped':True}),flush=True)
finally:
    for c in reversed(procs):
        if c.poll() is None:os.killpg(c.pid,signal.SIGINT)
    for c in reversed(procs):
        if c.poll() is None:c.wait(timeout=15)
    for log in logs:log.close()
    n.destroy_node();rclpy.shutdown()
