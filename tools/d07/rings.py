import rosbag2_py, numpy as np, sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
BAG='/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
T='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=[T]))
C=get_message('sensor_msgs/msg/PointCloud2')
n=0; A=[]
while rd.has_next():
    name,data,ts=rd.read_next(); n+=1
    if n<20 or n%10: continue
    m=deserialize_message(data,C)
    a=pc2.read_points(m,skip_nans=False)
    A.append(np.column_stack([np.asarray(a[k],float) for k in ('x','y','z','ring','intensity','timestamp')]))
    if len(A)>=20: break
P=np.concatenate(A)
x,y,z,ring=P[:,0],P[:,1],P[:,2],P[:,3]
rh=np.hypot(x,y); el=np.degrees(np.arctan2(z,rh)); az=np.degrees(np.arctan2(y,x)); r=np.sqrt(x*x+y*y+z*z)
print('npts/frame',len(P)/len(A),'r min',r.min())
for k in range(0,96,1):
    s=ring==k
    if s.sum()==0: continue
    print(k, s.sum()//len(A), 'el %.1f..%.1f med %.1f'%(np.percentile(el[s],1),np.percentile(el[s],99),np.median(el[s])), 'az %.1f..%.1f'%(az[s].min(),az[s].max()))
print('overall el',el.min(),el.max(),'az',az.min(),az.max())
# timestamp field
t=P[:,5]; print('ts field', t.min(), t.max())
