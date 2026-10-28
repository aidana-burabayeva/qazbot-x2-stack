#!/usr/bin/env python3
"""Summarize recorded operator motion; does not treat covariance as ground truth."""
import argparse
import json
import math
import sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

p = argparse.ArgumentParser()
p.add_argument('bag', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
if not (a.bag / 'metadata.yaml').is_file():
    p.error('Bag is not finalized: metadata.yaml is absent; do not open an active recorder database')
dbpath = next(a.bag.glob('*.db3')).resolve()
db = sqlite3.connect(dbpath.as_uri() + '?mode=ro', uri=True)
wanted = {'/qazbot_lio/odom_level', '/qazbot_d1_map/amcl_pose', '/qazbot_d1_map/tf', '/qazbot_lio/scan'}
topics = {i: (n, get_message(t)) for i,n,t in db.execute('SELECT id,name,type FROM topics') if n in wanted}
if not topics:
    raise RuntimeError('No localization topics in finalized bag')
# Release the read transaction before deserialization/analysis. Only finalized
# bags are supported; this tool must never compete with a recording writer.
ids = list(topics)
rows = db.execute('SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN (' +
                  ','.join('?' for _ in ids) + ') ORDER BY timestamp', ids).fetchall()
db.close()
series, poses, scans, tf = {}, [], [], []
def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for tid,bt,data in rows:
    name, cls = topics[tid]
    if name not in ['/qazbot_lio/odom_level', '/qazbot_d1_map/amcl_pose', '/qazbot_d1_map/tf', '/qazbot_lio/scan']:
        continue
    m = deserialize_message(data,cls)
    if name == '/qazbot_lio/odom_level':
        po=m.pose.pose
        series.setdefault(name,[]).append([bt/1e9,po.position.x,po.position.y,yaw(po.orientation)])
    elif name == '/qazbot_d1_map/amcl_pose':
        po=m.pose.pose
        poses.append({'bag_time_s':bt/1e9,'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                      'x':po.position.x,'y':po.position.y,'yaw_deg':math.degrees(yaw(po.orientation)),
                      'variance':[m.pose.covariance[i] for i in [0,7,35]]})
    elif name == '/qazbot_d1_map/tf':
        for t in m.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                tr=t.transform.translation;q=t.transform.rotation
                tf.append([bt/1e9,tr.x,tr.y,yaw(q)])
    else:
        scans.append({'bag_time_s':bt/1e9,'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                      'angle_min':m.angle_min,'angle_increment':m.angle_increment,
                      'range_min':m.range_min,'range_max':m.range_max,
                      'ranges':[r if math.isfinite(r) else None for r in m.ranges]})
result={'bag':str(a.bag),'local_motion':None,'amcl_updates':len(poses),'amcl_poses':poses,
        'scan_count':len(scans),'ground_truth':'not measured; operator-controlled manual motion'}
local=np.array(series.get('/qazbot_lio/odom_level',[]))
if len(local):
    angles=np.unwrap(local[:,3]);dist=np.linalg.norm(local[:,1:3]-local[0,1:3],axis=1)
    result['local_motion']={'samples':len(local),'yaw_span_deg':math.degrees(float(np.ptp(angles))),
        'end_yaw_change_deg':math.degrees(float(angles[-1]-angles[0])),
        'max_translation_m':float(dist.max()),'end_translation_m':float(dist[-1])}
result['scan_snapshots']=[]
for idx in sorted(set([0,len(scans)//2,len(scans)-1])) if scans else []:
    s=scans[idx];before=local[local[:,0]<=s['bag_time_s']]
    corr=[t for t in tf if t[0]<=s['bag_time_s']]
    if not len(before) or not corr:continue
    po=before[-1];co=corr[-1];c,ss=math.cos(co[3]),math.sin(co[3])
    s['approx_map_pose']=[co[1]+c*po[1]-ss*po[2],co[2]+ss*po[1]+c*po[2],co[3]+po[3]]
    result['scan_snapshots'].append(s)
result['snapshot_note']='Pose reconstructed from latest bag-received correction/local pose; overlay is diagnostic, not ground truth or an exact synchronized residual test.'
a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:result[k] for k in ['local_motion','amcl_updates','scan_count']},indent=2))
