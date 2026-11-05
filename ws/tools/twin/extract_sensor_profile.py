#!/usr/bin/env python3
"""Read-only measurements from .66 bags. Does not infer physical range noise from scene returns."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def quantiles(a):
    return {k: float(v) for k,v in zip(('p05','p50','p95'), np.percentile(a,[5,50,95]))} if len(a) else None


def extract(bags):
    sources=[]; periods={'scan':[],'odom':[]}; ages={'scan':[],'odom':[]}; masks=[]; geometry=None
    for bag in bags:
        dbs=sorted(bag.glob('*.db3'))
        if len(dbs)!=1 or not (bag/'metadata.yaml').exists():raise ValueError('requires finalized unsplit bag')
        db=sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro',uri=True)
        ids={n:i for i,n in db.execute('select id,name from topics')}
        sources.append(dict(bag=bag.name, metadata_sha256=hashlib.sha256((bag/'metadata.yaml').read_bytes()).hexdigest(),db_bytes=dbs[0].stat().st_size))
        for kind,topic,cls in [('scan','/qazbot_lio/scan',LaserScan),('odom','/qazbot_lio/odom_level',Odometry)]:
            prev=None
            for ts,raw in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids[topic],)):
                m=deserialize_message(raw,cls);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                ages[kind].append(ts/1e9-stamp)
                if prev is not None and stamp>prev:periods[kind].append(stamp-prev)
                prev=stamp
                if kind=='scan':
                    g=dict(bins=len(m.ranges),angle_min=m.angle_min,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max)
                    if geometry is not None and g!=geometry:raise ValueError('scan geometry changed')
                    geometry=g;masks.append(np.isfinite(m.ranges))
        db.close()
    period={k:quantiles(v) for k,v in periods.items()};age={k:quantiles(v) for k,v in ages.items()}
    probability=np.mean(masks,axis=0)
    config_path=Path(__file__).resolve().parents[2]/'ws/src/lio/x2.yaml'
    mapping=yaml.safe_load(config_path.read_text())['/**']['ros__parameters']['mapping']
    imu_lidar=np.eye(4);imu_lidar[:3,:3]=np.array(mapping['extrinsic_R']).reshape(3,3);imu_lidar[:3,3]=mapping['extrinsic_T']
    return dict(imu_from_lidar=imu_lidar.tolist(),snapshot_scope='Recorded bags; current live .66 settings not verified',lio_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),schema=1,id='robot66_observation_v1',qualification='Partial sensor parity; NOT validated dynamics or simulated FAST-LIO.',
                sources=sources,scan=geometry,period_s=period,receive_minus_header_s=age,
                observed_finite_probability=probability.tolist(),
                measured_finite_fraction=quantiles(np.mean(masks,axis=1)),
                model=dict(fov_deg=103.0,fov_center_deg=0.0,ray_step_m=0.025,twist_baseline_s=0.3,
                           scan_hz=1/period['scan']['p50'],odom_hz=1/period['odom']['p50'],
                           latency_s=max(0.0,age['odom']['p50']),drift='none',
                           fov_source='ws/src/lio/x2.yaml: approximate 103 degree chest wedge',
                           geometry_source='vendor FK pelvis->lidar_chest_front plus inverse extrinsic from ws/src/lio/x2.yaml',
                           limitations=['2D map rays are not a 3D LiDAR','no raw cloud/IMU FAST-LIO dynamics','finite probabilities describe recorded scene and are NOT dropout calibration']))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('bags',type=Path,nargs='+');a=ap.parse_args()
    r=extract(a.bags);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k!='observed_finite_probability'},indent=2))
