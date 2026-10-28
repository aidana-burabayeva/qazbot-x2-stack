import sys, time, math, numpy as np, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from sensor_msgs_py import point_cloud2 as pc2
up=np.array([float(v) for v in sys.argv[1:4]]); up/=np.linalg.norm(up)
def level_rotation(u):
    z=u/np.linalg.norm(u); a=np.array([1.0,0,0]) if abs(z[0])<0.9 else np.array([0,1.0,0]); x=a-(a@z)*z; x/=np.linalg.norm(x); return np.vstack([x,np.cross(z,x),z])
Rlv=level_rotation(up)
rclpy.init(); n=rclpy.create_node('cell_probe'); od=[]; cl=[]; mo=[]
n.create_subscription(Odometry,'/qazbot_lio/odom_level',od.append,10)
n.create_subscription(PointCloud2,'/qazbot_lio/cloud_registered',lambda m:cl.append(pc2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True).astype(float)),QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT))
n.create_subscription(TFMessage,'/qazbot_d1_map/tf',lambda m:[mo.append(t) for t in m.transforms if t.child_frame_id=='odom'],QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE))
s=time.time()
while time.time()-s<150 and not(od and mo and len(cl)>=12): rclpy.spin_once(n,timeout_sec=0.2)
print('got clouds',len(cl),'odom',len(od),'tf',len(mo))
P=np.vstack(cl); imu_z=od[-1].pose.pose.position.z
Po=P@Rlv.T; t=mo[-1].transform; th=2*math.atan2(t.rotation.z,t.rotation.w); c,sn=math.cos(th),math.sin(th)
mx=t.translation.x+c*Po[:,0]-sn*Po[:,1]; my=t.translation.y+sn*Po[:,0]+c*Po[:,1]; mz=Po[:,2]-imu_z
fl=np.median(mz[mz<np.percentile(mz,15)]); print('points',len(P),'floor (median of lowest 15%%) = %.2f m rel IMU'%fl)
np.savez('cells_cloud.npz',mx=mx,my=my,h=mz-fl)
for nm,(cx,cy) in (('occupied (1.43,-5.88)',(1.43,-5.88)),('unknown (0.38,-7.72)',(0.38,-7.72)),('free reference (0.5,-4.5)',(0.5,-4.5))):
    for r in (0.15,0.30):
        k=np.hypot(mx-cx,my-cy)<r
        if k.sum()==0: print(f'{nm} r={r}: no points'); continue
        h=(mz-fl)[k]; print(f'{nm} r={r}: {int(k.sum())} pts; height above floor min/median/max {h.min():.2f}/{np.median(h):.2f}/{h.max():.2f}; >0.10 m: {int((h>0.10).sum())}, >0.35 m: {int((h>0.35).sum())}')
