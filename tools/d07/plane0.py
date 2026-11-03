import rosbag2_py, numpy as np, datetime as dt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
BAG='/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
T='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=[T]))
C=get_message('sensor_msgs/msg/PointCloud2')
n=0
while rd.has_next():
    name,data,ts=rd.read_next(); n+=1
    if n<100: continue
    m=deserialize_message(data,C); break
print(dt.datetime.fromtimestamp(ts/1e9), m.header.stamp)
a=pc2.read_points(m,skip_nans=False)
P=np.column_stack([np.asarray(a[k],float) for k in 'xyz'])
rng=np.random.default_rng(0); best=None
for it in range(3000):
    i=rng.choice(len(P),3,replace=False); p=P[i]
    nrm=np.cross(p[1]-p[0],p[2]-p[0]); L=np.linalg.norm(nrm)
    if L<1e-6: continue
    nrm/=L; d=-nrm@p[0]
    inl=np.abs(P@nrm+d)<0.02
    if best is None or inl.sum()>best[0]: best=(inl.sum(),nrm,d)
k,nrm,d=best
if d<0: nrm,d=-nrm,-d
inl=np.abs(P@nrm+d)<0.03
# refine
Q=P[inl]; c=Q.mean(0); u,s,vt=np.linalg.svd(Q-c); nrm=vt[2];
if nrm@c>0: nrm=-nrm
d=-nrm@c
print('inliers',inl.sum(),'normal',nrm,'height',d,'pitch of x axis wrt floor deg', np.degrees(np.arcsin(-nrm[0])) , 'roll', np.degrees(np.arcsin(-nrm[1])))
h=P@nrm+d
rh=np.hypot(P[:,0],P[:,1])
fl=np.abs(h)<0.03
# horizontal distance of floor points in levelled frame
proj=P-np.outer(P@nrm, nrm)
hd=np.linalg.norm(proj,axis=1)
print('floor pts',fl.sum(),'min horiz dist of floor', hd[fl].min(), 'p1',np.percentile(hd[fl],1))
print('residual std floor (|h|<0.05, hd<3):', h[(np.abs(h)<0.05)&(hd<3)].std())
np.save('plane0.npy',np.r_[nrm,d])
