import rclpy,time,math,json,numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from tf2_msgs.msg import TFMessage
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
rclpy.init();n=Node('plan');TF={};G={}
be=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def tf(m):
    for t in m.transforms: TF[(t.header.frame_id,t.child_frame_id)]=t.transform
n.create_subscription(TFMessage,'/qazbot_d1_map/tf',tf,be)
n.create_subscription(OccupancyGrid,'/qazbot_d1_map/nav2/global_costmap/costmap',lambda m:G.__setitem__('g',m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
e=time.time()+4
while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
m=TF[('map','odom')];o=TF[('odom','base_footprint')];ym=yaw(m.rotation)
x0=m.translation.x+math.cos(ym)*o.translation.x-math.sin(ym)*o.translation.y
y0=m.translation.y+math.sin(ym)*o.translation.x+math.cos(ym)*o.translation.y
yw=ym+yaw(o.rotation);yw=math.atan2(math.sin(yw),math.cos(yw))
gx,gy=x0+3*math.cos(yw),y0+3*math.sin(yw)
print('start %.3f %.3f yaw %.3f -> goal %.3f %.3f yaw %.3f'%(x0,y0,yw,gx,gy,yw))
c=ActionClient(n,ComputePathToPose,'/qazbot_d1_map/nav2/compute_path_to_pose');assert c.wait_for_server(10)
def ps(x,y,w):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(w/2);p.pose.orientation.w=math.cos(w/2);return p
g=ComputePathToPose.Goal();g.start=ps(x0,y0,yw);g.goal=ps(gx,gy,yw);g.use_start=True;g.planner_id='GridBased'
f=c.send_goal_async(g)
while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
h=f.result();print('accepted',h.accepted)
r=h.get_result_async()
while not r.done(): rclpy.spin_once(n,timeout_sec=0.05)
res=r.result();pts=[(p.pose.position.x,p.pose.position.y) for p in res.result.path.poses]
print('status',res.status,'poses',len(pts),'plan_time',res.result.planning_time.sec+res.result.planning_time.nanosec*1e-9)
L=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]));print('path length %.2f vs straight 3.00'%L)
# lateral deviation from straight line
ux,uy=math.cos(yw),math.sin(yw);dev=max(abs(-(p[0]-x0)*uy+(p[1]-y0)*ux) for p in pts);print('max lateral dev %.3f m'%dev)
g_=G['g'];res_=g_.info.resolution;a=np.array(g_.data).reshape(g_.info.height,g_.info.width)
mx=0;lat=[]
for p in pts:
    for l in (-0.5,-0.25,0,0.25,0.5):
        xx=p[0]-l*uy;yy=p[1]+l*ux;i=int((xx-g_.info.origin.position.x)/res_);j=int((yy-g_.info.origin.position.y)/res_);mx=max(mx,int(a[j,i]))
print('max global costmap value within +-0.5 m of path (static map incl.): ',mx)
json.dump({'start':[x0,y0,yw],'goal':[gx,gy,yw],'path':pts,'len':L,'dev':dev,'maxcost':mx},open('plan_e2e02.json','w'))
