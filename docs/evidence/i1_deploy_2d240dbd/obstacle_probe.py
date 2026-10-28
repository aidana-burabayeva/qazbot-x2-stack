import json,math,time,signal
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry,OccupancyGrid
p=Path('/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd');rclpy.init();n=rclpy.create_node('i1_passive_obstacle_probe');running=True
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT);latest={};f=(p/'obstacle_probe.jsonl').open('w',buffering=1)
def stop(*a):
 global running
 running=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def cloud(k,m):
 latest[k]={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'points':[[float(v) for v in row] for row in point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)]}
def odom(m):
 a=m.pose.pose;z=a.orientation;latest['pose']=[a.position.x,a.position.y,math.atan2(2*(z.w*z.z+z.x*z.y),1-2*(z.y*z.y+z.z*z.z))]
def grid(m):
 latest['grid']=m
for k in ('scan_obstacles','scan_clearing'):
 n.create_subscription(PointCloud2,'/qazbot_lio/'+k,lambda m,k=k:cloud(k,m),q)
n.create_subscription(Odometry,'/qazbot_native/odom',odom,q)
n.create_subscription(OccupancyGrid,'/costmap/costmap',grid,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def sample():
 if not all(k in latest for k in ('pose','grid','scan_obstacles','scan_clearing')):return
 g=latest['grid'];x,y,yaw=latest['pose'];c,s=math.cos(yaw),math.sin(yaw);roi=[]
 for iy in range(g.info.height):
  for ix in range(g.info.width):
   wx=g.info.origin.position.x+(ix+.5)*g.info.resolution-x;wy=g.info.origin.position.y+(iy+.5)*g.info.resolution-y
   bx,by=c*wx+s*wy,-s*wx+c*wy
   if .55<bx<1.45 and abs(by)<.3 and g.data[iy*g.info.width+ix]==100:roi.append([bx,by])
 out={'wall':time.time(),'phase':(p/'obstacle_phase.txt').read_text().strip() if (p/'obstacle_phase.txt').exists() else 'waiting_obstacle','odom_pose':latest['pose'],'grid_stamp':g.header.stamp.sec+g.header.stamp.nanosec*1e-9,'roi_lethal':roi,'marking':latest['scan_obstacles'],'clearing':latest['scan_clearing']}
 f.write(json.dumps(out)+'\n')
n.create_timer(.5,sample)
try:
 while running:rclpy.spin_once(n,timeout_sec=.1)
finally:f.close();n.destroy_node();rclpy.try_shutdown()
