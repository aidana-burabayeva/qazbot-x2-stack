import rclpy,time,math,json,re,sys
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_msgs.msg import SpeedLimit
gx,gy,gyaw=float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]);LOG=sys.argv[4]
rclpy.init();n=Node('go');S={'v':0,'w':0,'cmd':(0,0),'auth':(0,0),'trust':None,'gc':None,'pose':None};T=[]
be=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Odometry,'/qazbot_lio/odom_level',lambda m:S.update(v=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y),w=m.twist.twist.angular.z),be)
n.create_subscription(Twist,'/qazbot_d1_map/cmd_vel',lambda m:S.update(cmd=(m.linear.x,m.angular.z)),10)
n.create_subscription(Twist,'/qazbot_d1_map/cmd_vel_authorized',lambda m:S.update(auth=(m.linear.x,m.angular.z)),10)
n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/amcl_pose',lambda m:S.update(pose=(m.pose.pose.position.x,m.pose.pose.position.y)),10)
n.create_subscription(String,'/qazbot_d1_map/localization_trust/status',lambda m:S.update(trust=json.loads(m.data)),10)
n.create_subscription(String,'/qazbot_d1_map/goal_coordinator/status',lambda m:S.update(gc=json.loads(m.data)),10)
SL=float(sys.argv[5]) if len(sys.argv)>5 else 0.0
if SL>0:
    pub=n.create_publisher(SpeedLimit,'/qazbot_d1_map/nav2/speed_limit',10)
    for _ in range(10):
        m=SpeedLimit();m.percentage=False;m.speed_limit=SL;pub.publish(m);rclpy.spin_once(n,timeout_sec=0.1)
    print('speed_limit',SL,flush=True)
c=ActionClient(n,NavigateToPose,'/qazbot_d1_map/navigate_to_pose')
assert c.wait_for_server(timeout_sec=10)
g=NavigateToPose.Goal();p=g.pose;p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg()
p.pose.position.x=gx;p.pose.position.y=gy;p.pose.orientation.z=math.sin(gyaw/2);p.pose.orientation.w=math.cos(gyaw/2)
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.1)
start=S['pose'];print('start',start,'goal',gx,gy,flush=True)
fut=c.send_goal_async(g);t0=time.time()
while not fut.done() and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.05)
h=fut.result();print('accepted',h.accepted,flush=True)
if not h.accepted: sys.exit(1)
rf=h.get_result_async();t0=time.time();last=0;stall=[];abort=None;maxv=0;maxw=0
while not rf.done():
    rclpy.spin_once(n,timeout_sec=0.05);t=time.time()-t0
    maxv=max(maxv,S['v']);maxw=max(maxw,abs(S['w']))
    if t-last>=0.5:
        last=t;gc=S['gc'] or {};tr=S['trust'] or {}
        T.append((round(t,1),round(S['v'],2),round(S['w'],2),tuple(round(x,2) for x in S['cmd']),tuple(round(x,2) for x in S['auth']),S['pose'] and tuple(round(x,2) for x in S['pose']),tr.get('ready'),gc.get('gate'),gc.get('fault')))
    # stall event (no-progress, not a speed threshold): pure rotation commanded >=0.3 rad/s while the measured
    # yaw rate and position are flat for 5 s
    if abs(S['auth'][1])>=0.3 and abs(S['auth'][0])<0.02:
        stall.append((t,S['w'],S['pose']))
    else: stall=[]
    stall=[x for x in stall if t-x[0]<=5.0]
    if len(stall)>10 and stall[-1][0]-stall[0][0]>=4.9 and max(abs(x[1]) for x in stall)<0.05:
        abort='stall: rotation commanded %.2f rad/s, measured yaw rate <0.05 for 5 s'%S['auth'][1]
    if t>75: abort='procedural timeout 75s'
    if abort:
        print('ABORT',abort,flush=True);h.cancel_goal_async();
        for _ in range(40): rclpy.spin_once(n,timeout_sec=0.05)
        break
res=rf.result() if rf.done() else None
print('result status',res.status if res else None,'maxv %.2f maxw %.2f'%(maxv,maxw),'end pose',S['pose'],'time %.1f'%(time.time()-t0))
open(LOG,'w').write(json.dumps({'start':start,'goal':(gx,gy),'timeline':T,'abort':abort,'status':res.status if res else None,'end':S['pose'],'maxv':maxv,'maxw':maxw}))
