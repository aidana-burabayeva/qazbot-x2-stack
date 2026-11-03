# When the robot is still (native odom moved <2 cm and <1 deg over 1 s) request an AMCL no-motion update at 2 Hz
# (same effect as after operator initialpose). Stops as soon as the robot moves. Logs one line per still period.
import rclpy,math,time
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
rclpy.init();n=Node('orch_standstill_refine');h=[]
def od(m):
    p=m.pose.pose;h.append((time.time(),p.position.x,p.position.y,yaw(p.orientation)))
    while h and time.time()-h[0][0]>1.0: h.pop(0)
n.create_subscription(Odometry,'/qazbot_native/odom',od,20)
MAXN=10
c=n.create_client(Empty,'/qazbot_d1_map/request_nomotion_update');c.wait_for_service(10)
still_since=None;last=0;cnt=0
while rclpy.ok():
    rclpy.spin_once(n,timeout_sec=0.05)
    if len(h)<10 or time.time()-h[-1][0]>0.5: still_since=None;continue
    dp=max(math.hypot(a[1]-h[-1][1],a[2]-h[-1][2]) for a in h);dy=max(abs(math.atan2(math.sin(a[3]-h[-1][3]),math.cos(a[3]-h[-1][3]))) for a in h)
    if dp<0.02 and dy<math.radians(1.0):
        if still_since is None: still_since=time.time();cnt=0
        if time.time()-still_since>1.0 and time.time()-last>0.5 and cnt<MAXN:
            c.call_async(Empty.Request());last=time.time();cnt+=1
            if cnt==1: print('%.1f still -> refining'%time.time(),flush=True)
    else:
        if still_since and cnt: print('%.1f moving, %d refines'%(time.time(),cnt),flush=True)
        still_since=None
