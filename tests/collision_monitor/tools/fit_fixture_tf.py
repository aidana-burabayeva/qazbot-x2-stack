#!/usr/bin/env python3
"""One-off fixture-TF fit (base_footprint -> lidar_chest_front) from 20 raw clouds at bag t+10 s (empty scene):
gravity from LiDAR-IMU accel, floor plane refit, x = LiDAR x on the floor, origin = IMU on the floor.
Writes tf_fit.json in the CWD; values were copied into config/harness.yaml. Offline analysis (prints).
"""
import rosbag2_py, numpy as np, json, sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
BAG='/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
T0=1793701816633712810
T='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=[T]))
rd.seek(T0+int(10e9))
clouds=[]
while rd.has_next() and len(clouds)<20:
    n,d,ts=rd.read_next(); m=deserialize_message(d,PointCloud2); clouds.append((ts,m))
m=clouds[0][1]
print('frame',m.header.frame_id,'w',m.width,'h',m.height,'fields',[(f.name,f.datatype) for f in m.fields],'pstep',m.point_step,'bytes',len(m.data),'dense',m.is_dense)
print('stamp-recv ms',[round((ts-(mm.header.stamp.sec*10**9+mm.header.stamp.nanosec))/1e6,1) for ts,mm in clouds[:10]])
P=np.concatenate([np.column_stack([pc2.read_points(mm,field_names=('x','y','z'),skip_nans=True)[k] for k in 'xyz']).astype(float) for _,mm in clouds])
r=np.linalg.norm(P,axis=1); print('npts/cloud',len(P)/len(clouds),'range min %.3f p0.1 %.3f'%(r[r>0].min(),np.percentile(r[r>0],0.1)), 'zeros',(r==0).sum())
ext=json.load(open('/home/agi/control_backend/docs/evidence/x2_extrinsics.json'))
Ril=np.array(ext['R_imu_lidar']); Til=np.array(ext['T_imu_lidar'])

TI='/aima/hal/sensor/lidar_chest_front/imu'
from sensor_msgs.msg import Imu
rd2=rosbag2_py.SequentialReader(); rd2.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd2.set_filter(rosbag2_py.StorageFilter(topics=[TI])); rd2.seek(T0+int(10e9)); A=[]
while rd2.has_next() and len(A)<2000:
    n,d,ts=rd2.read_next(); mm=deserialize_message(d,Imu); a=mm.linear_acceleration; A.append([a.x,a.y,a.z])
ai=np.mean(A,0); print('imu acc mean',ai,'norm',np.linalg.norm(ai))
up_l=Ril.T@(ai/np.linalg.norm(ai)); print('up in lidar frame (from imu acc)',up_l)
hgt=P@up_l; rr=np.linalg.norm(P-np.outer(hgt,up_l),axis=1)
C=P[(hgt<-0.75)&(rr>0.4)&(rr<3.0)]
nrm=up_l.copy(); dd=None
for it in range(5):
    d0=-np.median(C@nrm) if dd is None else dd
    inl=np.abs(C@nrm+d0)<0.03; Q=C[inl]; c=Q.mean(0); w,v=np.linalg.eigh(np.cov((Q-c).T)); nrm=v[:,0]
    if nrm@up_l<0: nrm=-nrm
    dd=-nrm@c
print('inliers',inl.sum(),'/',len(C),'floor normal(lidar)',nrm,'angle vs imu-gravity deg %.2f'%np.degrees(np.arccos(min(1,nrm@up_l))))
h=dd  # plane: n.p + dd = 0 ; lidar origin signed dist = dd
z=nrm; x=np.array([1,0,0.]); x=x-(x@z)*z; x/=np.linalg.norm(x); y=np.cross(z,x)
R_bl=np.vstack([x,y,z])
imu_l=-Ril.T@Til  # imu origin in lidar frame
o=imu_l-(imu_l@z+dd)*z  # projection of imu origin to floor (lidar frame)
t_bl=-R_bl@o
print('lidar height above floor %.4f ; imu height %.4f'%(dd, imu_l@z+dd))
print('pitch of lidar x vs horizontal deg %.2f'%np.degrees(np.arcsin(np.array([1,0,0])@z)))
Pb=P@R_bl.T+t_bl
hb=Pb[:,2]; rb=np.hypot(Pb[:,0],Pb[:,1]); bb=np.degrees(np.arctan2(Pb[:,1],Pb[:,0]))
for lo,hi in [(0.45,1.0),(1.0,1.5),(1.5,2.0)]:
    s=(rb>lo)&(rb<hi)&(np.abs(hb)<0.3); print('floor h band',lo,hi,'n',s.sum(),'p50 %.3f p99 %.3f p99.9 %.3f max %.3f'%tuple(np.percentile(hb[s],[50,99,99.9]).tolist()+[hb[s].max()]))
print('lidar origin in base', t_bl, 'min horiz range of floor pts %.3f'%rb[np.abs(hb)<0.05].min())
print('bearing p0.1/p99.9', np.percentile(bb,[0.1,99.9]))
import scipy.spatial.transform as st
q=st.Rotation.from_matrix(R_bl).as_quat(); print('quat xyzw',q.tolist(),'rpy deg',st.Rotation.from_matrix(R_bl).as_euler('xyz',degrees=True))
json.dump(dict(R_bl=R_bl.tolist(),t_bl=t_bl.tolist(),q_xyzw=q.tolist(),lidar_h=float(dd)),open('tf_fit.json','w'),indent=1)
