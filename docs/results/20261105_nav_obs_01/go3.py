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
import numpy as np
from tf2_msgs.msg import TFMessage
from nav2_msgs.srv import GetCostmap
THR=0.25  # test-script event: cancel the Goal (normal procedure) if the swept footprint is closer than this to the box; planned minimum 0.43 m
TFS={}
n.create_subscription(TFMessage,'/qazbot_d1_map/tf',lambda m:[TFS.__setitem__((t.header.frame_id,t.child_frame_id),t.transform) for t in m.transforms],be)
def _yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def mpose():
    M=TFS[('map','odom')];O=TFS[('odom','base_footprint')];ym=_yaw(M.rotation)
    return (M.translation.x+math.cos(ym)*O.translation.x-math.sin(ym)*O.translation.y,M.translation.y+math.sin(ym)*O.translation.x+math.cos(ym)*O.translation.y,ym+_yaw(O.rotation))
FOOT=[(0.17,0.229),(0.008,0.34),(-0.182,0.34),(-0.305,0.229),(-0.305,-0.261),(-0.182,-0.372),(0.008,-0.372),(0.17,-0.261)]
FPTS=[]
for _q in range(len(FOOT)):
    _a=FOOT[_q];_b=FOOT[(_q+1)%len(FOOT)]
    for _t in (0,0.25,0.5,0.75): FPTS.append((_a[0]+(_b[0]-_a[0])*_t,_a[1]+(_b[1]-_a[1])*_t))
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
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.1)
P0=mpose();ux,uy=math.cos(P0[2]),math.sin(P0[2])
cc=n.create_client(GetCostmap,'/qazbot_d1_map/nav2/global_costmap/get_costmap');assert cc.wait_for_service(5)
_f=cc.call_async(GetCostmap.Request())
while not _f.done(): rclpy.spin_once(n,timeout_sec=0.05)
_r=_f.result().map;_a=np.array(_r.data,dtype=np.uint8).reshape(_r.metadata.size_y,_r.metadata.size_x);_res=_r.metadata.resolution
_ys,_xs=np.where(_a==254);_px=_r.metadata.origin.position.x+(_xs+0.5)*_res;_py=_r.metadata.origin.position.y+(_ys+0.5)*_res
cells=[((x-P0[0])*ux+(y-P0[1])*uy,-(x-P0[0])*uy+(y-P0[1])*ux) for x,y in zip(_px,_py)]
cells=[c for c in cells if 1.5<c[0]<2.8 and abs(c[1])<0.7]
assert len(cells)>=3,'box not marked in the global costmap'
a0=min(c[0] for c in cells);lc=(min(c[1] for c in cells)+max(c[1] for c in cells))/2
BOX=(a0-0.025,a0+0.12,lc-0.1175,lc+0.1175)
print('box polygon in start frame: along [%.2f,%.2f] lateral [%.2f,%.2f]; cells %d'%(BOX+(len(cells),)),flush=True)
def clearance():
    p=mpose();best=9.9
    c,s_=math.cos(p[2]),math.sin(p[2])
    for fx,fy in FPTS:
        X=p[0]+c*fx-s_*fy;Y=p[1]+s_*fx+c*fy;la=(X-P0[0])*ux+(Y-P0[1])*uy;lb=-(X-P0[0])*uy+(Y-P0[1])*ux
        dx=max(BOX[0]-la,0,la-BOX[1]);dy=max(BOX[2]-lb,0,lb-BOX[3]);best=min(best,math.hypot(dx,dy))
    return best,p
minclr=9.9
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
        T.append((round(t,1),round(clearance()[0],2) if TFS else None,round(S['v'],2),round(S['w'],2),tuple(round(x,2) for x in S['cmd']),tuple(round(x,2) for x in S['auth']),S['pose'] and tuple(round(x,2) for x in S['pose']),tr.get('ready'),gc.get('gate'),gc.get('fault')))
    # stall event (no-progress, not a speed threshold): pure rotation commanded >=0.3 rad/s while the measured
    # yaw rate and position are flat for 5 s
    if abs(S['auth'][1])>=0.3 and abs(S['auth'][0])<0.02:
        stall.append((t,S['w'],S['pose']))
    else: stall=[]
    stall=[x for x in stall if t-x[0]<=5.0]
    if len(stall)>10 and stall[-1][0]-stall[0][0]>=4.9 and max(abs(x[1]) for x in stall)<0.05:
        abort='stall: rotation commanded %.2f rad/s, measured yaw rate <0.05 for 5 s'%S['auth'][1]
    try:
        _c,_p=clearance();minclr=min(minclr,_c)
        if _c<THR: abort='footprint-box clearance %.2f m < %.2f m (cancel via normal procedure)'%(_c,THR)
    except KeyError: pass
    if t>75: abort='procedural timeout 75s'
    if abort:
        print('ABORT',abort,flush=True);h.cancel_goal_async();
        for _ in range(40): rclpy.spin_once(n,timeout_sec=0.05)
        break
res=rf.result() if rf.done() else None
print('min footprint-box clearance %.3f m'%minclr);print('result status',res.status if res else None,'maxv %.2f maxw %.2f'%(maxv,maxw),'end pose',S['pose'],'time %.1f'%(time.time()-t0))
open(LOG,'w').write(json.dumps({'start':start,'goal':(gx,gy),'timeline':T,'abort':abort,'status':res.status if res else None,'end':S['pose'],'maxv':maxv,'maxw':maxw,'min_clearance':minclr,'box':BOX,'start_map':P0}))
