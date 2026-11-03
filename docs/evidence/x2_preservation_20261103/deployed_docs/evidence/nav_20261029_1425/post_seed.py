# After operator initialpose: N AMCL no-motion updates, report Trust metrics, resume GC+adapter only if Trust ready.
import rclpy,json,time,sys
from rclpy.node import Node
from std_srvs.srv import Empty,Trigger
from std_msgs.msg import String
rclpy.init();n=Node('orch_post_seed');st={}
for k in ['localization_trust','goal_coordinator','motion_adapter']:
    n.create_subscription(String,f'/qazbot_d1_map/{k}/status',lambda m,k=k:st.__setitem__(k,json.loads(m.data)),10)
def spin(s):
    t=time.time()
    while time.time()-t<s: rclpy.spin_once(n,timeout_sec=0.1)
nm=n.create_client(Empty,'/qazbot_d1_map/request_nomotion_update');nm.wait_for_service(5)
for i in range(12):
    nm.call_async(Empty.Request());spin(0.5)
spin(3);tr=st.get('localization_trust',{});m=tr.get('metrics',{})
print('trust ready',tr.get('ready'),'motion_ok',tr.get('motion_ok'),'reasons',tr.get('reasons'),'sigma_xy %.2f'%m.get('sigma_xy',-1),'second_mode %.2f'%m.get('second_mode_weight',-1),'scan_score %.2f'%m.get('scan_score',-1))
if not tr.get('ready'): print('NOT READY - no resume');sys.exit(1)
for k in ['goal_coordinator','motion_adapter']:
    if st.get(k,{}).get('latched') or st.get(k,{}).get('latch'):
        c=n.create_client(Trigger,f'/qazbot_d1_map/{k}/resume');c.wait_for_service(5);f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5);print(k,'resume',f.result().message if f.result() else None)
spin(3);print('gc',{k:st['goal_coordinator'].get(k) for k in ('latched','ready','fault')});print('adapter',{k:st['motion_adapter'].get(k) for k in ('state','latch','registration')})
