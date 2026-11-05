#!/usr/bin/env python3
"""Offline diagnostic SE(2) alignment of an operator turn. Never publishes a pose or command."""
import json, math, sqlite3, sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, str(Path.home()/'control_backend/ws/src/navigation'))
from loc_trust import OccGrid, scan_endpoints, to_map
bag, output = map(Path, sys.argv[1:3])
if not (bag/'metadata.yaml').exists():
    raise RuntimeError('Only finalized bags are supported')
db=sqlite3.connect(next(bag.glob('*.db3')).as_uri()+'?mode=ro',uri=True)
names={'/qazbot_lio/odom_level','/qazbot_lio/scan','/qazbot_d1_map/amcl_pose'}
types={i:(name,get_message(t)) for i,name,t in db.execute('select id,name,type from topics') if name in names}
rows=db.execute('select topic_id,data from messages where topic_id in ('+','.join('?' for _ in types)+') order by timestamp',list(types)).fetchall()
db.close()
odom=[];scans=[];poses=[]
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for tid,data in rows:
 name,typ=types[tid];m=deserialize_message(data,typ)
 if name.endswith('odom_level'):
  p=m.pose.pose;odom.append([stamp(m),p.position.x,p.position.y,yaw(p.orientation)])
 elif name.endswith('amcl_pose'):
  p=m.pose.pose;poses.append([stamp(m),p.position.x,p.position.y,yaw(p.orientation)])
 else:scans.append(m)
odom=np.array(odom);odom=odom[np.argsort(odom[:,0])];odom[:,3]=np.unwrap(odom[:,3])
def at(t):return np.array([np.interp(t,odom[:,0],odom[:,i]) for i in (1,2,3)])
key=[];previous=None
for m in scans:
 t=stamp(m)
 if t<odom[0,0] or t>odom[-1,0]:continue
 p=at(t)
 if previous is None or abs(p[2]-previous)>=math.radians(30):
  key.append((m,p));previous=p[2]
if len(key)<6:raise RuntimeError('Insufficient turn coverage for multiview diagnostic')
last_scan=scans[-1];final_local=at(stamp(last_scan))
# Compare against the actual map->odom correction propagated to the final scan time.
ap=np.array(poses[-1][1:]);al=at(poses[-1][0])
dyaw=ap[2]-al[2];c,s=math.cos(dyaw),math.sin(dyaw)
delta=final_local[:2]-al[:2]
amcl_final=np.array([ap[0]+c*delta[0]-s*delta[1],ap[1]+s*delta[0]+c*delta[1],final_local[2]+dyaw])
points=[];origins=[]
for m,p in key:
 b=scan_endpoints(m.ranges,m.angle_min,m.angle_increment,.5,8.)[::3]
 points.append(to_map(b,p))
 origins.append(np.repeat(p[None,:2],len(b),axis=0))
points=np.concatenate(points)
q=points-final_local[:2];c,s=math.cos(final_local[2]),math.sin(final_local[2])
points=np.column_stack((c*q[:,0]+s*q[:,1],-s*q[:,0]+c*q[:,1]))
o=np.concatenate(origins)-final_local[:2]
origins=np.column_stack((c*o[:,0]+s*o[:,1],-s*o[:,0]+c*o[:,1]))
grid=OccGrid.from_pgm(Path.home()/'control_backend/ws/src/maps/first_map_nav2_v1/map.yaml')
def scores(poses, sample=None, sigma=.10):
 sample=points if sample is None else sample
 result=[]
 for start in range(0,len(poses),256):
  p=poses[start:start+256];c=np.cos(p[:,2,None]);s=np.sin(p[:,2,None])
  x=p[:,0,None]+c*sample[None,:,0]-s*sample[None,:,1]
  y=p[:,1,None]+s*sample[None,:,0]+c*sample[None,:,1]
  ij=np.column_stack((x.ravel(),y.ravel()))
  d=grid.dist_at(ij).reshape(x.shape)
  result.extend(np.exp(-np.minimum(d,2.)**2/(2*sigma**2)).mean(axis=1))
 return np.array(result)
if '--global' in sys.argv:
 import cv2,yaml
 meta=yaml.safe_load((Path.home()/'control_backend/ws/src/maps/first_map_nav2_v1/map.yaml').read_text())
 image=np.flipud(cv2.imread(str(Path.home()/'control_backend/ws/src/maps/first_map_nav2_v1'/meta['image']),0))
 probability=(255-image.astype(float))/255
 rows,cols=np.nonzero((probability<meta['free_thresh'])[::2,::2]);rows*=2;cols*=2
 xy=np.column_stack((grid.ox+(cols+.5)*grid.res,grid.oy+(rows+.5)*grid.res))
 angles=np.radians(np.arange(-180,180,6))
 candidates=np.column_stack((np.repeat(xy[:,0],len(angles)),np.repeat(xy[:,1],len(angles)),np.tile(angles,len(xy))))
 values=scores(candidates,points[::3],.15)
 print('Global coarse candidates '+str(len(candidates)),flush=True)
else:
 xs=amcl_final[0]+np.arange(-2.,2.001,.1)
 ys=amcl_final[1]+np.arange(-2.,2.001,.1)
 angles=amcl_final[2]+np.radians(np.arange(-60.,60.01,3.))
 xx,yy,aa=np.meshgrid(xs,ys,angles,indexing='ij')
 candidates=np.column_stack((xx.ravel(),yy.ravel(),aa.ravel()))
 candidates=candidates[grid.dist_at(candidates[:,:2])>.454]
 values=scores(candidates)
def ranked(candidates,values,count):
 peaks=[]
 for index in np.argsort(-values):
  p=candidates[index]
  if all(np.linalg.norm(p[:2]-np.array(q['pose'][:2]))>.4 or abs(math.atan2(math.sin(p[2]-q['pose'][2]),math.cos(p[2]-q['pose'][2])))>math.radians(10) for q in peaks):
   peaks.append(dict(score=float(values[index]),pose=p.tolist()))
  if len(peaks)==count:break
 return peaks
peaks=ranked(candidates,values,12 if '--global' in sys.argv else 8)
if '--global' in sys.argv:
 refined=[]
 for peak in peaks:
  p=np.array(peak['pose'])
  xx,yy,aa=np.meshgrid(p[0]+np.arange(-.15,.15001,.025),p[1]+np.arange(-.15,.15001,.025),p[2]+np.radians(np.arange(-6,6.01,.5)),indexing='ij')
  candidates=np.column_stack((xx.ravel(),yy.ravel(),aa.ravel()))
  values=scores(candidates)
  refined.extend(ranked(candidates,values,1))
 refined=sorted(refined,key=lambda p:-p['score'])
 peaks=ranked(np.array([p['pose'] for p in refined]),np.array([p['score'] for p in refined]),8)
result=dict(scope='Offline multiview endpoint diagnostic, no ground truth, no runtime acceptance or automatic pose publication.',
 global_search='--global' in sys.argv,keyframes=len(key),endpoints=len(points),yaw_coverage_deg=math.degrees(float(np.ptp(odom[:,3]))),
 amcl_final_pose=amcl_final.tolist(),amcl_score=float(scores(amcl_final[None,:])[0]),peaks=peaks)
def beam_obstruction(pose):
 endpoints=to_map(points,pose);starts=to_map(origins,pose)
 directions=endpoints-starts
 ranges=np.linalg.norm(directions,axis=1)
 distances=np.arange(.5,8.,grid.res/2)
 sample=starts[:,None,:]+directions[:,None,:]*distances[None,:,None]/ranges[:,None,None]
 # Occupied cell centres at d=0. Unknown cells are not fabricated obstacles.
 occupied=grid.dist_at(sample.reshape(-1,2)).reshape(len(points),-1)==0
 before=distances[None,:]<(ranges[:,None]-2*grid.res)
 return float(np.any(occupied & before,axis=1).mean())
result['raycast_note']='Fraction of observed rays crossing an occupied cell more than two map cells before the endpoint; diagnostic, not ground truth.'
result['amcl_blocked_ray_fraction']=beam_obstruction(amcl_final)
for peak in result['peaks']:peak['blocked_ray_fraction']=beam_obstruction(peak['pose'])
output.write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
