import rclpy,time
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init();n=Node('orch_gc_status');d={}
n.create_subscription(String,'/qazbot_d1_map/goal_coordinator/status',lambda m:d.__setitem__('s',m.data),10)
t=time.time()
while time.time()-t<8 and 's' not in d: rclpy.spin_once(n,timeout_sec=0.2)
print(d.get('s',''))
