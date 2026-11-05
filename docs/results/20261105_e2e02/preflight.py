import rclpy,time,math,json,sys
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from aimdk_msgs.msg import McCommonState,McLocomotionVelocity
from nav_msgs.msg import Odometry
rclpy.init();n=Node('pf');S={};TF={};MV={}
be=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
tl=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/amcl_pose',lambda m:S.update(amcl=m),tl)
for k,t in (('gc','goal_coordinator/status'),('tr','localization_trust/status'),('ad','motion_adapter/status')):
    n.create_subscription(String,'/qazbot_d1_map/'+t,(lambda k:lambda m:S.update({k:m.data}))(k),10)
n.create_subscription(McCommonState,'/aima/mc/common/state',lambda m:S.update(mc=m),be)
def tf(m):
    for t in m.transforms: TF[(t.header.frame_id,t.child_frame_id)]=(t.transform,t.header.stamp.sec+t.header.stamp.nanosec*1e-9,time.time())
n.create_subscription(TFMessage,'/qazbot_d1_map/tf',tf,be)
def mv(m): MV[m.source]=MV.get(m.source,0)+1
n.create_subscription(McLocomotionVelocity,'/aima/mc/locomotion/velocity',mv,be)
e=time.time()+5
while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
a=S['amcl'].pose.pose;print('amcl_pose(latched) x %.2f y %.2f yaw %.3f stamp_age %.0fs'%(a.position.x,a.position.y,yaw(a.orientation),time.time()-(S['amcl'].header.stamp.sec)))
for k in (('map','odom'),('odom','base_footprint')):
    if k in TF: t,ts,tr=TF[k];print(k,'x %.2f y %.2f yaw %.3f age %.2fs'%(t.translation.x,t.translation.y,yaw(t.rotation),time.time()-ts))
    else: print(k,'MISSING')
if ('map','odom') in TF and ('odom','base_footprint') in TF:
    m,_,_=TF[('map','odom')];o,_,_=TF[('odom','base_footprint')];ym=yaw(m.rotation);yo=yaw(o.rotation)
    x=m.translation.x+math.cos(ym)*o.translation.x-math.sin(ym)*o.translation.y
    y=m.translation.y+math.sin(ym)*o.translation.x+math.cos(ym)*o.translation.y
    print('map->base composed x %.3f y %.3f yaw %.3f'%(x,y,ym+yo))
mc=S['mc'];print('MC input_source',mc.input_source.name,'prio',mc.input_source.priority,'body',mc.body_status.value,'fsm',mc.fsm_state.current_state,'speed_mode',mc.speed_status.speed_mode.value,'fwd',mc.speed_status.forward_bounds.min_value,mc.speed_status.forward_bounds.max_value,'ang',mc.speed_status.angular_bounds.min_value,mc.speed_status.angular_bounds.max_value)
print('MC velocity msgs by source (5s):',MV)
print('GC',S['gc']);tr=json.loads(S['tr']);print('TRUST',{k:tr.get(k) for k in ('trusted','ready','reasons','diagnostics','metrics')})
print('AD',S['ad'])
