import rclpy,json,time
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
rclpy.init();n=Node('orch_resume_all');st={}
for k in ['goal_coordinator','motion_adapter','localization_trust']:
    n.create_subscription(String,f'/qazbot_d1_map/{k}/status',lambda m,k=k:st.__setitem__(k,json.loads(m.data)),10)
def spin(s):
    t=time.time()
    while time.time()-t<s: rclpy.spin_once(n,timeout_sec=0.1)
def call(s):
    c=n.create_client(Trigger,s);c.wait_for_service(5);f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5);return f.result().message if f.result() else None
spin(3)
if st.get('motion_adapter',{}).get('latch'): print('ad resume',call('/qazbot_d1_map/motion_adapter/resume'))
if st.get('goal_coordinator',{}).get('latched'): print('gc resume',call('/qazbot_d1_map/goal_coordinator/resume'))
spin(2);g=st.get('goal_coordinator',{});a=st.get('motion_adapter',{});print('gc',g.get('latched'),g.get('ready'),'ad',a.get('state'),a.get('latch'),a.get('registration'),'trust',st.get('localization_trust',{}).get('ready'))
