#!/usr/bin/env python3
"""Read finalized interactive MC bags; evaluate pelvis/torso/ankles per navigation leg.
Vendor TF and private Nav TF stay separate. HAL pelvis truth supplies only the audit root.
"""
import bisect
import json
import math
from pathlib import Path
import sqlite3
import sys
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_ros import Buffer,TransformException
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'ws/src/navigation'))
from control_geometry import transform


def matrix(p,q):return transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
def wrap(a):return math.atan2(math.sin(a),math.cos(a))


def main():
    run=Path(sys.argv[1]);bag=run/'frames'
    if not (bag/'metadata.yaml').exists():raise RuntimeError('bag must be finalized')
    db=sqlite3.connect(next(bag.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
    topics={i:n for i,n in db.execute('select id,name from topics')};limits=db.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    rclpy.init();buffer=Buffer(cache_time=Duration(nanoseconds=int(limits[1]-limits[0]+10**9)));roots=[]
    for tid,ts,raw in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name=topics[tid]
        if name in ('/tf','/tf_static'):
            for tf in deserialize_message(raw,TFMessage).transforms:
                if name=='/tf_static':buffer.set_transform_static(tf,'vendor_audit')
                else:buffer.set_transform(tf,'vendor_audit')
        elif name=='/aima/hal/odom/state':
            m=deserialize_message(raw,Odometry);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            roots.append((stamp,matrix(m.pose.pose.position,m.pose.pose.orientation)))
    db.close();events=json.loads((run/'journey.json').read_text())['events'];legs=[]
    for kind,finish in [('outbound','settling'),('return','settling_return')]:
        begin=next((e for e in events if e.get('state')==kind),None);end=next((e for e in events if e.get('state')==finish),None)
        if begin is None or end is None:continue
        selected=[r for r in roots if begin['time']<=r[0]/1e9<=end['time']]
        if not selected:raise RuntimeError('no HAL samples in navigation leg')
        selected=selected[::20]+selected[-1:];series={f:[] for f in ['pelvis','torso_link','left_ankle_roll_link','right_ankle_roll_link']};missing=0
        for stamp,world in selected:
            for frame in series:
                try:
                    local=buffer.lookup_transform('pelvis',frame,Time(nanoseconds=stamp))
                    p,q=local.transform.translation,local.transform.rotation;T=world@matrix(p,q)
                    series[frame].append([stamp/1e9,T[0,3],T[1,3],math.atan2(T[1,0],T[0,0])])
                except TransformException:missing+=1
        start_yaw=series['pelvis'][0][3];desired=wrap(begin['goal'][2]-start_yaw)
        result=dict(leg=kind,target_relative_deg=math.degrees(desired),tf_failures=missing,frames={})
        for frame,rows in series.items():
            if not rows:raise RuntimeError('missing '+frame)
            a=np.array(rows);heading=np.unwrap(a[:,3]);change=heading[-1]-heading[0]
            result['frames'][frame]=dict(turn_deg=math.degrees(float(change)),relative_goal_error_deg=math.degrees(wrap(change-desired)),
                max_xy_m=float(np.linalg.norm(a[:,1:3]-a[0,1:3],axis=1).max()),samples=len(rows))
        legs.append(result)
    output=dict(scope='MuJoCo HAL pelvis truth + time-matched vendor FK; ankle-roll frames, not measured sole contacts',legs=legs)
    (run/'turn_frame_audit.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2));rclpy.shutdown()


if __name__=='__main__':main()
