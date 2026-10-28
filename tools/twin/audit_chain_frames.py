#!/usr/bin/env python3
"""Join HAL pelvis truth to vendor FK at command-plateau endpoints (SIM bag only)."""
import bisect
import json
import math
from pathlib import Path
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

p=Path(sys.argv[1]); audit=json.loads((p/'audit.json').read_text())
db=sqlite3.connect(f'file:{next(p.glob("*.db3"))}?mode=ro',uri=True)
topics={i:n for i,n in db.execute('select id,name from topics')}
history={}; parents={}; hal=[]
def matrix(v,q):
    x,y,z,w=q.x,q.y,q.z,q.w
    r=np.eye(4);r[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    r[:3,3]=[v.x,v.y,v.z];return r
for tid,ts,raw in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name=topics[tid];t=ts/1e9
    if name=='/aima/hal/odom/state':
        m=deserialize_message(raw,Odometry);hal.append((t,matrix(m.pose.pose.position,m.pose.pose.orientation)))
    elif name in ('/tf','/tf_static'):
        for tr in deserialize_message(raw,TFMessage).transforms:
            parents[tr.child_frame_id]=tr.header.frame_id
            history.setdefault(tr.child_frame_id,[]).append((t,matrix(tr.transform.translation,tr.transform.rotation),name=='/tf_static'))
ht=[r[0] for r in hal];times={f:[r[0] for r in rows] for f,rows in history.items()}
def pose(frame,t):
    i=bisect.bisect_right(ht,t)-1
    if i<0 or t-ht[i]>.05:raise RuntimeError('missing HAL sample')
    root=hal[i][1]; chain=[];child=frame;age=t-ht[i]
    while child!='pelvis':
        rows=history[child];j=bisect.bisect_right(times[child],t)-1
        if j<0:raise RuntimeError('missing joint transform')
        stamp,mat,static=rows[j]
        if not static:age=max(age,t-stamp)
        chain.append(mat);child=parents[child]
    if age>.05:raise RuntimeError('stale joint transform')
    for mat in reversed(chain):root=root@mat
    return root,age
rows=[]
for s in audit['plateaus']:
    r=dict(command=s['command'],duration=s['duration'],frames={})
    for f in ('pelvis','torso_link','left_ankle_roll_link','right_ankle_roll_link'):
        a,age0=pose(f,s['start']);b,age1=pose(f,s['end'])
        ya=math.atan2(a[1,0],a[0,0]);yb=math.atan2(b[1,0],b[0,0])
        r['frames'][f]=dict(endpoint_yaw_change_deg=math.degrees(math.atan2(math.sin(yb-ya),math.cos(yb-ya))),
                           endpoint_xy_change_m=float(np.linalg.norm(b[:2,3]-a[:2,3])),max_join_age_s=max(age0,age1))
    rows.append(r)
(p/'frame_endpoints.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows,indent=2))
