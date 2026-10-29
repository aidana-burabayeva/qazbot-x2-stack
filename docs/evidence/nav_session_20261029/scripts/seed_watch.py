# Waits for operator initialpose, then 12 AMCL no-motion updates at 2 Hz, then prints Trust metrics every 2 s for 20 s.
import rclpy,json,time
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init();n=Node('orch_seed_watch');st={};got=[]
n.create_subscription(String,'/qazbot_d1_map/localization_trust/status',lambda m:st.__setitem__('t',json.loads(m.data)),10)
n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/initialpose',lambda m:got.append(time.time()),10)
nm=n.create_client(Empty,'/qazbot_d1_map/request_nomotion_update');nm.wait_for_service(5)
def spin(s):
    t=time.time()
    while time.time()-t<s: rclpy.spin_once(n,timeout_sec=0.05)
print('waiting initialpose',flush=True)
while not got: spin(0.2)
print('initialpose received',flush=True);spin(0.3)
for i in range(12): nm.call_async(Empty.Request());spin(0.5)
for k in range(10):
    spin(2);t=st.get('t',{});m=t.get('metrics',{})
    print('ready',t.get('ready'),'motion_ok',t.get('motion_ok'),'reasons',t.get('reasons'),'sigma %.2f'%m.get('sigma_xy',-1),'2nd %.2f'%m.get('second_mode_weight',-1),'score %.2f'%m.get('scan_score',-1),flush=True)
