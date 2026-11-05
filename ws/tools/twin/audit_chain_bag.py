#!/usr/bin/env python3
"""Read-only audit of chain bag: frame provenance, command plateaus and pelvis truth.
Run inside the SIM ROS environment: audit_chain_bag.py BAG_DIR.
"""
import json
import math
from pathlib import Path
import sqlite3
import sys
from collections import Counter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

p = Path(sys.argv[1])
dbs = sorted(p.glob('*.db3'))
if len(dbs) != 1:
    raise RuntimeError('expected one unsplit sqlite bag')
db = sqlite3.connect(f'file:{dbs[0]}?mode=ro', uri=True)
topics = {i: (n,t) for i,n,t in db.execute('select id,name,type from topics')}
counts = {topics[i][0]: n for i,n in db.execute('select topic_id,count(*) from messages group by topic_id')}
edges, segments, active, truth = {}, [], None, []
last_t = None
for tid, ts, raw in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name, typ = topics[tid]
    if typ not in ('tf2_msgs/msg/TFMessage', 'aimdk_msgs/msg/McLocomotionVelocity', 'nav_msgs/msg/Odometry'):
        continue
    m = deserialize_message(raw, get_message(typ))
    t = ts/1e9
    if typ == 'tf2_msgs/msg/TFMessage':
        for tr in m.transforms:
            key = (name, tr.header.frame_id, tr.child_frame_id)
            edges[key] = edges.get(key, 0) + 1
    if typ == 'aimdk_msgs/msg/McLocomotionVelocity':
        key = (m.source, round(m.forward_velocity,3), round(m.lateral_velocity,3), round(m.angular_velocity,3))
        if active is not None and active['command'] == key and t-last_t < .15:
            active['end'] = t
            active['n'] += 1
        else:
            active = dict(command=key,start=t,end=t,n=1)
            segments.append(active)
        last_t = t
    if name == '/qazbot_e2e/model_truth':
        q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        truth.append((t,m.pose.pose.position.x,m.pose.pose.position.y,yaw))
for seg in segments:
    seg['duration'] = seg['end']-seg['start']
    rows=[r for r in truth if seg['start']<=r[0]<=seg['end']]
    if len(rows)>1:
        total=sum(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])) for a,b in zip(rows,rows[1:]))
        seg['pelvis_turn_deg']=math.degrees(total)
        seg['pelvis_xy_m']=math.hypot(rows[-1][1]-rows[0][1],rows[-1][2]-rows[0][2])
result=dict(counts=counts,frames=[dict(topic=k[0],parent=k[1],child=k[2],n=v) for k,v in sorted(edges.items())],
            plateaus=[s for s in segments if s['duration']>=.5],
            note='rounded commands at MC boundary; durations measured first-to-last packet; frames kept separate by TF topic')
(p/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(counts=counts, frame_edges=len(edges),plateaus=result['plateaus']),indent=2))
