#!/usr/bin/env python3
"""Stationary raw-cloud height diagnostic. No ROS publishers, no runtime parameter changes."""
import json,math,sqlite3,sys
from pathlib import Path
import numpy as np,yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
nav=Path.home()/'control_backend/ws/src/navigation';sys.path.insert(0,str(nav));sys.path.insert(0,str(nav.parent/'lio'))
from level_bridge import quat_to_rot,level_rotation,LIDAR_FWD_IN_IMU
from loc_trust import OccGrid,scan_endpoints,to_map,soft_score
bag,out=map(Path,sys.argv[1:3])
if not (bag/'metadata.yaml').exists():raise RuntimeError('finalized bag required')
db=sqlite3.connect(next(bag.glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics') if n in [
 '/qazbot_lio/odometry','/qazbot_lio/odom_level','/qazbot_d1_map/amcl_pose',
 '/aima/hal/sensor/lidar_chest_front/imu','/aima/hal/sensor/lidar_chest_front/lidar_pointcloud']}
def read(name,last_ns=None):
 i,typ=topics[name]
 sql='select data from messages where topic_id=?';args=[i]
 if last_ns is not None:sql+=' and timestamp>=?';args.append(last_ns)
 return [deserialize_message(data,typ) for data, in db.execute(sql+' order by timestamp',args)]
raw='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
end_ns=db.execute('select max(timestamp) from messages where topic_id=?',(topics[raw][0],)).fetchone()[0]
start_ns=end_ns-5_000_000_000
world=read('/qazbot_lio/odometry',start_ns);local=read('/qazbot_lio/odom_level',start_ns)
imus=read('/aima/hal/sensor/lidar_chest_front/imu',start_ns);clouds=read(raw,start_ns)
amcl=read('/qazbot_d1_map/amcl_pose')[-1];db.close()
def t(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
times=np.array([t(m) for m in world])
rot=[quat_to_rot(m.pose.pose.orientation) for m in world]
up=[]
for m in imus:
 k=int(np.argmin(abs(times-t(m))));a=m.linear_acceleration
 up.append(rot[k]@np.array([a.x,a.y,a.z]))
R=level_rotation(np.median(up,axis=0))
wl=world[-1];ll=local[-1]
f=R@quat_to_rot(wl.pose.pose.orientation)@LIDAR_FWD_IN_IMU
delta=yaw(ll.pose.pose.orientation)-math.atan2(f[1],f[0]);c,s=math.cos(delta),math.sin(delta)
R=np.array([[c,-s,0],[s,c,0],[0,0,1]])@R
cfg=yaml.safe_load((nav.parent/'lio/x2.yaml').read_text())['/**']['ros__parameters']['mapping']
E=np.array(cfg['extrinsic_R']).reshape(3,3);T=np.array(cfg['extrinsic_T'])
all_points=[]
for m in clouds[::2]:
 # FAST-LIO pose is at scan end. Stationarity bounds this diagnostic's residual deskew error.
 k=int(np.argmin(abs(times-(t(m)+.1))))
 values=pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)
 P=np.column_stack([values[name].ravel() for name in ('x','y','z')]).astype(float)
 P=(P@E.T+T)@rot[k].T@R.T
 heading=yaw(ll.pose.pose.orientation);c,s=math.cos(heading),math.sin(heading)
 all_points.append(np.column_stack((c*P[:,0]+s*P[:,1],-s*P[:,0]+c*P[:,1],P[:,2])))
P=np.concatenate(all_points);radius=np.linalg.norm(P[:,:2],axis=1)
low=(P[:,2]<-.5)&(P[:,2]>-2)&(radius>.5)&(radius<5)
hist,edges=np.histogram(P[low,2],bins=np.arange(-2.,-.499,.02))
peaks=np.argsort(-hist)[:8]
height_peaks=[dict(z_relative_imu=float((edges[k]+edges[k+1])/2),points=int(hist[k])) for k in peaks]
# Report the broad lower plane; do not silently deploy a ground-height estimate.
k=int(peaks[0]);z0=(edges[k]+edges[k+1])/2
floor=P[low & (abs(P[:,2]-z0)<.06)]
A=np.column_stack((floor[:,:2],np.ones(len(floor))))
coef=np.linalg.lstsq(A,floor[:,2],rcond=None)[0]
residual=floor[:,2]-A@coef
z_floor=float(coef[2]);floor_tilt=math.degrees(math.atan(np.linalg.norm(coef[:2])))
q=amcl.pose.pose;pose=np.array([q.position.x,q.position.y,yaw(q.orientation)])
# Propagate the last AMCL estimate to the latest local sample using recorded LIO.
# Rotation dominates this near-stationary interval; include source-time odometry from the full analysis.
analysis=json.loads((out.parent/'manual_amcl_turn_01_raycast.json').read_text())
pose=np.array(analysis['amcl_final_pose'])
grid=OccGrid.from_pgm(nav.parent/'maps/first_map_nav2_v1/map.yaml')
measure=[]
xx,yy,aa=np.meshgrid(pose[0]+np.arange(-2,2.001,.1),
 pose[1]+np.arange(-2,2.001,.1),pose[2]+np.radians(np.arange(-60,60.01,3)),indexing='ij')
candidates=np.column_stack((xx.ravel(),yy.ravel(),aa.ravel()))
candidates=candidates[grid.dist_at(candidates[:,:2])>.454]
def best_fit(points):
 values=[]
 for k in range(0,len(candidates),256):
  p=candidates[k:k+256];c=np.cos(p[:,2,None]);s=np.sin(p[:,2,None])
  x=p[:,0,None]+c*points[None,:,0]-s*points[None,:,1]
  y=p[:,1,None]+s*points[None,:,0]+c*points[None,:,1]
  d=grid.dist_at(np.column_stack((x.ravel(),y.ravel()))).reshape(x.shape)
  values.extend(np.exp(-np.minimum(d,2.)**2/(2*.1**2)).mean(axis=1))
 order=np.argsort(-np.array(values));peaks=[]
 for k in order:
  p=candidates[k]
  if all(np.linalg.norm(p[:2]-np.array(q['pose'][:2]))>.4 or abs(math.atan2(math.sin(p[2]-q['pose'][2]),math.cos(p[2]-q['pose'][2])))>math.radians(10) for q in peaks):
   peaks.append(dict(pose=p.tolist(),score=float(values[k])))
  if len(peaks)==3:return peaks
for name,zmin,zmax in [('current',-.65,.8),('D1_halfband_5cm',z_floor+.165,z_floor+.265),
 ('D1_halfband_10cm',z_floor+.115,z_floor+.315),('D1_halfband_15cm',z_floor+.065,z_floor+.365)]:
 valid=(radius>.5)&(radius<8)&(P[:,2]>zmin)&(P[:,2]<zmax)
 if name=='current':valid&=P[:,2]>(-.65+.035*radius)
 pts=P[valid];a=np.arctan2(pts[:,1],pts[:,0]);r=np.linalg.norm(pts[:,:2],axis=1)
 bins=((a+math.pi)/(2*math.pi)*1440).astype(int)%1440
 ranges=np.full(1440,np.inf);np.minimum.at(ranges,bins,r)
 b=scan_endpoints(ranges,-math.pi,2*math.pi/1440,.5,8.)
 d=grid.dist_at(to_map(b,pose))
 measure.append(dict(name=name,rays=len(b),height_relative_imu=[float(zmin),float(zmax)],
  score_10cm=soft_score(b,grid,pose,.1),median_distance_m=float(np.median(d)),
  within_10cm=float(np.mean(d<=.1)),within_25cm=float(np.mean(d<=.25)),best_fits=best_fit(b)))
result=dict(scope='Offline stationary raw cloud, approximate scan-end association; not a deployed scan or proved localization.',
 raw_clouds=len(clouds),source_odom_xy_span_m=float(np.ptp([[m.pose.pose.position.x,m.pose.pose.position.y] for m in world],axis=0).max()),
 height_peaks=height_peaks,floor_fit=dict(z_relative_imu=z_floor,tilt_deg=floor_tilt,points=len(floor),
 p95_residual_m=float(np.percentile(abs(residual),95))),amcl_pose=pose.tolist(),slices=measure)
out.write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(result,indent=2))
