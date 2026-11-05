import rclpy,math,time,statistics as st
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
rclpy.init();n=Node('odq');R={'lvl':[],'lio':[],'nat':[]};be=QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def mk(k): 
    def f(m): R[k].append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,time.time(),yaw(m.pose.pose.orientation),m.twist.twist.angular.z,m.header.frame_id))
    return f
for k,t in (('lvl','/qazbot_lio/odom_level'),('lio','/qazbot_lio/odometry'),('nat','/qazbot_native/odom')): n.create_subscription(Odometry,t,mk(k),be)
e=time.time()+15
while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
for k,v in R.items():
    if len(v)<3: print(k,'n',len(v));continue
    ds=[b[0]-a[0] for a,b in zip(v,v[1:])];lag=[r-s for s,r,*_ in v]
    rate=[math.remainder(b[2]-a[2],2*math.pi)/(b[0]-a[0]) for a,b in zip(v,v[1:]) if b[0]>a[0]]
    print(k,v[0][4],'n',len(v),'hz %.1f'%(len(v)/15),'dt med %.3f max %.3f'%(st.median(ds),max(ds)),'gaps>0.2s',sum(d>0.2 for d in ds),'lag med %.3f'%st.median(lag),'yawrate sd %.3f max %.3f'%(st.pstdev(rate),max(abs(x) for x in rate)),'twist_wz sd %.3f'%st.pstdev([x[3] for x in v]),'yaw %.3f'%v[-1][2])
