#!/usr/bin/env python3
"""Passive raw-cloud height histogram in a gravity-aligned IMU frame; no runtime changes."""
import json
import sqlite3
import sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

folder=Path(sys.argv[1])
if not (folder/'metadata.yaml').is_file():raise RuntimeError('Input bag not finalized')
db=sqlite3.connect(next(folder.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
topics={n:i for i,n in db.execute('SELECT id,name FROM topics')}
it=topics['/aima/hal/sensor/lidar_chest_front/imu'];ct=topics['/aima/hal/sensor/lidar_chest_front/lidar_pointcloud']
imu=db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(it,)).fetchall()
cloud=db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(ct,)).fetchall()
db.close();acc=[]
for (raw,) in imu:
    m=deserialize_message(raw,Imu);v=m.linear_acceleration;acc.append([v.x,v.y,v.z])
up=np.median(acc,axis=0);up/=np.linalg.norm(up)
ext=json.loads(Path(sys.argv[2]).read_text());R=np.array(ext['R_imu_lidar']);t=np.array(ext['T_imu_lidar'])
result={'up_in_lidar_imu':up.tolist(),'assumption':'Stationary capture; acceleration used as gravity direction. Peaks are candidates, not verified floor.', 'frames':[]}
for idx in np.linspace(len(cloud)//4,len(cloud)-1,3,dtype=int):
    msg=deserialize_message(cloud[idx][0],PointCloud2)
    fields=read_points(msg,field_names=('x','y','z'),skip_nans=True)
    points=np.column_stack([fields[k] for k in ('x','y','z')]).astype(float)
    height=(points@R.T+t)@up
    bins=np.arange(-2.0,-.2,.01);counts,edges=np.histogram(height,bins)
    peaks=np.argsort(counts)[-5:][::-1]
    result['frames'].append({'index':int(idx),'point_count':len(points),'negative_height_peaks_m':
        [{'height':float((edges[i]+edges[i+1])/2),'count':int(counts[i])} for i in peaks]})
Path(sys.argv[3]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
