#!/usr/bin/env python3
"""Replay only lidar+IMU, after actual publisher/subscriber matching, in caller's isolated domain."""
import sqlite3
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock

folder=Path(sys.argv[1])
if not (folder/'metadata.yaml').is_file():raise RuntimeError('Input bag not finalized')
db=sqlite3.connect(next(folder.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
names={'/aima/hal/sensor/lidar_chest_front/lidar_pointcloud':'/replay/lidar',
       '/aima/hal/sensor/lidar_chest_front/imu':'/replay/imu'}
types={i:(n,get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics') if n in names}
rows=[(tid,stamp,data) for tid,stamp,data in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp') if tid in types]
db.close()
if not rows:raise RuntimeError('Input has no required sensor messages')
rclpy.init();node=rclpy.create_node('qazbot_matched_replay_player')
pubs={tid:node.create_publisher(cls,names[name],QoSProfile(depth=400,reliability=ReliabilityPolicy.RELIABLE)) for tid,(name,cls) in types.items()}
clock=node.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
try:
    deadline=time.monotonic()+30
    while not all(p.get_subscription_count()>0 for p in pubs.values()) or clock.get_subscription_count()<2:
        rclpy.spin_once(node,timeout_sec=.1)
        if time.monotonic()>deadline:raise RuntimeError('Replay publishers not matched; no input replayed')
    start=time.monotonic();begin=rows[0][1]
    for tid,stamp,data in rows:
        due=start+(stamp-begin)/1e9/.5
        while time.monotonic()<due:
            rclpy.spin_once(node,timeout_sec=max(0.,min(.01,due-time.monotonic())))
        c=Clock();c.clock.sec=stamp//10**9;c.clock.nanosec=stamp%10**9;clock.publish(c)
        pubs[tid].publish(deserialize_message(data,types[tid][1]))
    for pub in pubs.values():
        if not pub.wait_for_all_acked(rclpy.duration.Duration(seconds=5)):
            raise RuntimeError('Replay transport acknowledgement incomplete')
    print('Replay complete; input messages:',len(rows),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
