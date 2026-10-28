#!/usr/bin/env python3
"""Passive stream regression capture. Duration is an observation window, not recovery logic."""
import json
import math
import sys
import time
from collections import defaultdict
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = rclpy.create_node('qazbot_stream_regression_capture')
samples = defaultdict(list)
last_scan = None
refs = []
topics = {'/qazbot_lio/scan': LaserScan, '/qazbot_lio/odometry': Odometry,
          '/qazbot_lio/odom_level': Odometry,
          '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': PointCloud2,
          '/aima/mc/leg_odometry': Odometry}
def receive(topic, msg):
    global last_scan
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    samples[topic].append((stamp, n.get_clock().now().nanoseconds/1e9-stamp))
    if isinstance(msg, LaserScan):
        last_scan = {'frame_id': msg.header.frame_id, 'bins': len(msg.ranges),
                     'finite': sum(math.isfinite(x) for x in msg.ranges),
                     'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
                     'angle_increment': msg.angle_increment,
                     'ranges': [x if math.isfinite(x) else None for x in msg.ranges]}
for topic, cls in topics.items():
    refs.append(n.create_subscription(cls, topic, lambda m, t=topic: receive(t,m), qos_profile_sensor_data))
start = time.monotonic()
while time.monotonic() - start < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
result = {'duration_s': time.monotonic()-start, 'last_scan': last_scan, 'streams': {}}
for topic in topics:
    s = samples[topic]
    ages = sorted(x[1] for x in s)
    result['streams'][topic] = {'count': len(s),
        'stamp_regressions': sum(b[0]<a[0] for a,b in zip(s,s[1:])),
        'stamp_duplicates': sum(b[0]==a[0] for a,b in zip(s,s[1:])),
        'observed_hz': (len(s)-1)/(s[-1][0]-s[0][0]) if len(s)>1 and s[-1][0]>s[0][0] else None,
        'age_s_min_median_p95_max': [ages[0], ages[len(ages)//2], ages[int((len(ages)-1)*.95)], ages[-1]] if ages else None}
result['motion_endpoints'] = {}
for topic in ['/cmd_vel','/cmd_vel_final','/aima/mc/locomotion/velocity']:
    result['motion_endpoints'][topic] = [i.node_name for i in n.get_publishers_info_by_topic(topic)]
with open(sys.argv[1], 'w') as out:
    json.dump(result,out,indent=2,allow_nan=False)
print(json.dumps(result['streams'],indent=2))
n.destroy_node()
rclpy.shutdown()
