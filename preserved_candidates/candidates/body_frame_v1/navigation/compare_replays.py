#!/usr/bin/env python3
"""Compare common-stamp estimator outputs and finite scan rays in finalized bags."""
import json
import math
import sqlite3
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def load(folder):
    folder=Path(folder)
    if not (folder/'metadata.yaml').exists():
        raise RuntimeError('Replay bag not finalized: '+str(folder))
    db=sqlite3.connect(next(folder.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
    types={i:(n,get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics')}
    rows=db.execute('SELECT topic_id,data FROM messages ORDER BY timestamp').fetchall();db.close()
    odom={};counts=[]
    for tid,data in rows:
        name,cls=types[tid]
        if name not in ['/Odometry','/scan']:continue
        m=deserialize_message(data,cls)
        if name=='/scan':counts.append(sum(math.isfinite(r) for r in m.ranges))
        else:
            q=m.pose.pose.orientation;p=m.pose.pose.position
            odom[(m.header.stamp.sec,m.header.stamp.nanosec)]=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    return odom,counts
d,dc=load(sys.argv[1]);s,sc=load(sys.argv[2]);common=sorted(d.keys()&s.keys())
result={'common_odometry_stamps':len(common),
        'max_abs_pose_component_difference':max((max(abs(a-b) for a,b in zip(d[k],s[k])) for k in common),default=None),
        'dense_scan_count':len(dc),'sparse_scan_count':len(sc),
        'dense_finite_min_median_max': [min(dc),sorted(dc)[len(dc)//2],max(dc)] if dc else None,
        'sparse_finite_min_median_max': [min(sc),sorted(sc)[len(sc)//2],max(sc)] if sc else None,
        'interpretation':'Estimator equality and ray counts only; no claim of map localization or hardware precision'}
Path(sys.argv[3]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not common or not dc or not sc:raise SystemExit('Replay did not produce comparable outputs')
