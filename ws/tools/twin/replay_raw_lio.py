#!/usr/bin/env python3
"""Offline-only raw .66 cloud+IMU -> actual FAST-LIO + actual level_bridge.
Only private sensor topics are published. No MC API, adapter or command publisher.
Recorded odometry/scan are read-only references, never estimator inputs.
Preserves original headers and per-point times; no live freshness claims.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import re
import sqlite3
import subprocess
import sys
import time
import numpy as np
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):raise RuntimeError('isolated SIM only')
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2,LaserScan

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ws/src/navigation'))
from control_geometry import transform


def key(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

def pose(m):
    p,q=m.pose.pose.position,m.pose.pose.orientation
    return transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('bag',type=Path);ap.add_argument('name');ap.add_argument('--seconds',type=float,default=0);ap.add_argument('--rate',type=float,default=1.);ap.add_argument('--publication',choices=['configured','sparse','dense'],default='configured');a=ap.parse_args()
    if not 0<a.rate<=1:raise ValueError('replay rate must be in (0,1]')
    out=ROOT/'tests/integration_results'/a.name;out.mkdir(exist_ok=False)
    dbs=sorted(a.bag.glob('*.db3'))
    if len(dbs)!=1 or not (a.bag/'metadata.yaml').exists():raise ValueError('requires finalized unsplit bag')
    db=sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro',uri=True);ids={n:i for i,n in db.execute('select id,name from topics')}
    cloud='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud';imu='/aima/hal/sensor/lidar_chest_front/imu'
    kinds={ids[cloud]:('cloud',PointCloud2),ids[imu]:('imu',Imu)}
    records=list(db.execute('select topic_id,timestamp,data from messages where topic_id in (?,?) order by timestamp',tuple(kinds)))
    t0=records[0][1];records=[r for r in records if not a.seconds or r[1]-t0<=a.seconds*1e9]
    ref={}
    for _,raw in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids['/qazbot_lio/odometry'],)):
        m=deserialize_message(raw,Odometry);ref[key(m)]=pose(m)
    ref_finite=[]
    for raw, in db.execute('select data from messages where topic_id=?',(ids['/qazbot_lio/scan'],)):
        m=deserialize_message(raw,LaserScan);ref_finite.append(float(np.isfinite(m.ranges).mean()))
    db.close()
    binary=ROOT/'runtime_x86/lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'
    rclpy.init();n=Node('raw_lio_replayer');ns='/qazbot_raw_replay_'+re.sub('[^A-Za-z0-9_]','_',a.name);pub={k:n.create_publisher(cls,ns+'/'+k,qos_profile_sensor_data) for k,cls in [('cloud',PointCloud2),('imu',Imu)]}
    outputs={};levels=[];scans=[];processes=[];files=[];sent=dict(cloud=0,imu=0);result={}
    n.create_subscription(Odometry,ns+'/odometry',lambda m:outputs.__setitem__(key(m),pose(m)),100)
    n.create_subscription(Odometry,ns+'/odom_level',lambda m:levels.append(key(m)),100)
    n.create_subscription(LaserScan,ns+'/scan',lambda m:scans.append(dict(stamp=key(m),finite_fraction=float(np.isfinite(m.ranges).mean()),bins=len(m.ranges),range_max=m.range_max)),20)
    def start(name,cmd):
        f=(out/(name+'.log')).open('w');files.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,p))
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if any(p.poll() is not None for _,p in processes):raise RuntimeError('estimator child exited')
            rclpy.spin_once(n,timeout_sec=.002)
    try:
        start('fast_lio',[str(binary),'--ros-args','--params-file',str(ROOT/'ws/src/lio/x2.yaml'),
            '-r','__node:=twin_raw_fast_lio','-r',cloud+':='+ns+'/cloud','-r',imu+':='+ns+'/imu',
            '-r','/Odometry:='+ns+'/odometry','-r','/cloud_registered:='+ns+'/cloud_registered',
            '-r','/cloud_registered_body:='+ns+'/cloud_body','-r','/cloud_effected:='+ns+'/effected',
            '-r','/Laser_map:='+ns+'/map','-r','/path:='+ns+'/path','-r','/tf:='+ns+'/lio_tf'] +
            ([] if a.publication=='configured' else ['-p','publish.dense_publish_en:='+('true' if a.publication=='dense' else 'false')]))
        start('level_bridge',['python3',str(ROOT/'ws/src/lio/level_bridge.py'),'--ros-args','-r','__node:=twin_raw_level_bridge',
            '-r',imu+':='+ns+'/imu','-r','/Odometry:='+ns+'/odometry','-r','/cloud_registered:='+ns+'/cloud_registered',
            '-r','/scan:='+ns+'/scan','-r','/odom_level:='+ns+'/odom_level','-r','/tf:='+ns+'/tf',
            '-p','range_max:=8.0','-p','scan_every:=2'])
        deadline=time.monotonic()+20
        def inputs_ready():
            readers=lambda k:{x.node_name for x in n.get_subscriptions_info_by_topic(ns+'/'+k)}
            return 'twin_raw_fast_lio' in readers('cloud') and {'twin_raw_fast_lio','twin_raw_level_bridge'} <= readers('imu')
        while not inputs_ready():
            if time.monotonic()>deadline:raise RuntimeError('input discovery timed out')
            spin_for(.1)
        spin_for(1.)
        play=time.monotonic()
        for tid,stamp,raw in records:
            due=play+(stamp-t0)/1e9/a.rate
            while time.monotonic()<due:rclpy.spin_once(n,timeout_sec=min(.002,max(0,due-time.monotonic())))
            kind,cls=kinds[tid];pub[kind].publish(deserialize_message(raw,cls));sent[kind]+=1
            rclpy.spin_once(n,timeout_sec=0)
        spin_for(3.)
        shared=sorted(ref.keys()&outputs.keys());warm=shared[0]+2*10**9 if shared else 0;shared=[k for k in shared if k>=warm]
        if not shared:raise RuntimeError('no same-stamp estimator/reference poses after warm-up')
        align=ref[shared[0]]@np.linalg.inv(outputs[shared[0]]);errors=[];rot=[]
        for k in shared:
            estimate=align@outputs[k];errors.append(float(np.linalg.norm(estimate[:3,3]-ref[k][:3,3])))
            R=ref[k][:3,:3].T@estimate[:3,:3];rot.append(math.degrees(math.acos(np.clip((np.trace(R)-1)/2,-1,1))))
        np.savez_compressed(out/'poses.npz',stamps_ns=np.array(sorted(outputs),dtype=np.int64),poses=np.stack([outputs[k] for k in sorted(outputs)]))
        result=dict(namespace=ns,replay_rate=a.rate,publication=a.publication,sent=sent,raw_poses=len(outputs),level_poses=len(levels),scan_count=len(scans),same_stamp_pairs=len(shared),
            reference='recorded FAST-LIO output, NOT external truth; recorded map history not available to fresh replay',
            relative_position_error_m=dict(rmse=float(np.sqrt(np.mean(np.array(errors)**2))),p95=float(np.percentile(errors,95)),max=max(errors)),
            orientation_error_deg=dict(p95=float(np.percentile(rot,95)),max=max(rot)),
            scan_finite_fraction_median=float(np.median([s['finite_fraction'] for s in scans])) if scans else None,
            reference_scan_finite_fraction_median=float(np.median(ref_finite)) if ref_finite else None,
            smoke_passed=len(outputs)>=50 and len(levels)>=30 and len(scans)>=15,
            physically_validated=False,qualification='Actual raw-sensor estimator replay; not interactive simulated sensors or a qualified MC twin.',
            source_bag=str(a.bag),source_metadata_sha256=hashlib.sha256((a.bag/'metadata.yaml').read_bytes()).hexdigest(),
            binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),config_sha256=hashlib.sha256((ROOT/'ws/src/lio/x2.yaml').read_bytes()).hexdigest(),
            timestamp_policy=f'original headers and point times; receive-time pacing {a.rate}x; private output topics only')
    finally:
        for _,p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        codes={}
        for name,p in processes:
            try:codes[name]=p.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);codes[name]=p.wait(timeout=5)
        for f in files:f.close()
        result['shutdown']=codes;(out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        n.destroy_node();rclpy.shutdown()
    print(json.dumps(result,indent=2));return 0 if result.get('smoke_passed') else 1


if __name__=='__main__':raise SystemExit(main())
