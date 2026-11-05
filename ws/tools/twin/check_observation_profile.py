#!/usr/bin/env python3
"""Passive ROS observation contract, including limits that remain UNMATCHED."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

ap=argparse.ArgumentParser();ap.add_argument('namespace');ap.add_argument('output',type=Path);ap.add_argument('--seconds',type=float,default=8.);a=ap.parse_args()
rclpy.init();n=Node('observation_profile_check');counts={};ts={};latest={};finite=[];outside=[];ages=[]
def cb(k,m):
    counts[k]=counts.get(k,0)+1;latest[k]=m;ts.setdefault(k,[]).append(time.monotonic())
    if k=='scan':
        ranges=np.asarray(m.ranges);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment
        finite.append(float(np.isfinite(ranges).mean()));outside.append(int(np.isfinite(ranges[abs(angles)>math.radians(51.51)]).sum()))
    if k=='status':
        d=json.loads(m.data)
        age=d.get('source_age_at_publication_s',d['source_age_s'])
        if age is not None:ages.append(age)
for k,cls,topic in [('status',String,'twin/status'),('scan',LaserScan,'scan'),('odom',Odometry,'odom'),('truth',Odometry,'model_truth'),('observation_truth',Odometry,'model_observation_truth')]:
    n.create_subscription(cls,'/'+a.namespace+'/'+topic,lambda m,k=k:cb(k,m),10)
end=time.monotonic()+a.seconds
try:
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    hz={k:(len(t)-1)/(t[-1]-t[0]) for k,t in ts.items() if len(t)>1}
    scan=latest.get('scan');status=json.loads(latest['status'].data) if 'status' in latest else {}
    def xy(k):
        m=latest[k];return np.array([m.pose.pose.position.x,m.pose.pose.position.y])
    offset=float(np.linalg.norm(xy('truth')-xy('observation_truth'))) if all(k in latest for k in ['truth','observation_truth']) else None
    checks=dict(required_streams=all(counts.get(k,0)>1 for k in ['truth','observation_truth','odom','scan','status']),
                bins_1440=scan is not None and len(scan.ranges)==1440,
                scan_5hz=4<=hz.get('scan',0)<=6,odom_10hz=8<=hz.get('odom',0)<=12,
                outside_wedge_unobserved=bool(outside) and max(outside)==0,
                delayed_source_stamps=bool(ages) and .035<=float(np.median(ages))<=.1,
                distinct_chest_and_pelvis=offset is not None and .05<offset<.2)
    result=dict(checks=checks,passed=all(checks.values()),counts=counts,hz=hz,chest_pelvis_offset_m=offset,
                median_age_s=float(np.median(ages)) if ages else None,finite_fraction_median=float(np.median(finite)) if finite else None,
                status=status,remaining_mismatch='2D ray density/height filtering and LIO error are not validated; this is not physical qualification')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
finally:n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
