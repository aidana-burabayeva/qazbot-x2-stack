import rosbag2_py, numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG='/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
T='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=[T,'/aima/hal/sensor/lidar_chest_front/imu']))
C=get_message('sensor_msgs/msg/PointCloud2')
n=0
while rd.has_next():
    name,data,ts=rd.read_next()
    if name!=T: continue
    m=deserialize_message(data,C); n+=1
    if n==1:
        print(m.header, m.height, m.width, m.point_step, m.row_step, m.is_dense, [(f.name,f.offset,f.datatype,f.count) for f in m.fields])
        from sensor_msgs_py import point_cloud2 as pc2
        a=pc2.read_points(m,skip_nans=False)
        print(a.dtype, a[:5])
        x,y,z=[np.asarray(a[k],float) for k in 'xyz']
        r=np.sqrt(x*x+y*y+z*z); ok=np.isfinite(r)&(r>0)
        print('finite',ok.sum(),'of',len(r),'rmin',r[ok].min(),'rmed',np.median(r[ok]))
        for k in a.dtype.names: 
            v=np.asarray(a[k]); print(k, v.min(), v.max(), len(np.unique(v)))
    if n>=3: break
