#!/usr/bin/env python3
"""Compare relative LIO and leg-odometry motion; no external ground truth claim."""
import json
import math
import sqlite3
import sys
from pathlib import Path
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

folder=Path(sys.argv[1])
if not (folder/'metadata.yaml').exists():raise RuntimeError('Bag not finalized')
db=sqlite3.connect(next(folder.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
topics={i:n for i,n in db.execute('SELECT id,name FROM topics')}
names=['/qazbot_lio/odom_level','/aima/mc/leg_odometry']
rows=[(topics[i],t,b) for i,t,b in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp') if topics[i] in names];db.close()
f=np.array(json.loads(Path(sys.argv[2]).read_text())['R_imu_lidar'])[:,0]
series={name:[] for name in names}
for name,t,b in rows:
    m=deserialize_message(b,Odometry);po=m.pose.pose.position;q=m.pose.pose.orientation;x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    forward=R@(f if name==names[1] else np.array([1.,0.,0.]))
    series[name].append([t/1e9,po.x,po.y,math.atan2(forward[1],forward[0])])
out={'qualification':'Relative comparison only; leg_odom assumed gravity aligned. No measured physical ground truth.'}
for name,values in series.items():
    v=np.array(values);a=np.unwrap(v[:,3]);d=np.linalg.norm(v[:,1:3]-v[0,1:3],axis=1)
    out[name]={'samples':len(v),'max_translation_m':float(d.max()),'end_translation_m':float(d[-1]),
               'yaw_span_deg':math.degrees(float(np.ptp(a))),'end_yaw_change_deg':math.degrees(float(a[-1]-a[0]))}
Path(sys.argv[3]).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
