"""Hardware observation only: no command publishers, no services."""
import json,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from aimdk_msgs.msg import McCommonState
from sensor_msgs.msg import Imu,PointCloud2
rclpy.init();n=Node('qazbot_passive_hardware');data={};mc=[]
def recv(name,m):
 now=n.get_clock().now().nanoseconds*1e-9
 h=m.header;stamp=h.stamp.sec+h.stamp.nanosec*1e-9
 data.setdefault(name,[]).append((stamp,now-stamp))
def state(m):
 mc.append(dict(body=m.body_status.value,fsm=m.fsm_state.current_state,source=m.input_source.name))
n.create_subscription(McCommonState,'/aima/mc/common/state',state,qos_profile_sensor_data)
for name,cls in [('imu',Imu),('lidar_pointcloud',PointCloud2)]:
 n.create_subscription(cls,'/aima/hal/sensor/lidar_chest_front/'+name,lambda m,k=name:recv(k,m),qos_profile_sensor_data)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
r={'mc_last':mc[-1] if mc else None,'sensors':{},'mc_publishers':[(x.node_namespace,x.node_name) for x in n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity')]}
for name,rows in data.items():
 ages=sorted(x[1] for x in rows);r['sensors'][name]=dict(count=len(rows),hz=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else None,age_min=ages[0],age_max=ages[-1])
p=Path(sys.argv[1]) if len(sys.argv)>1 else Path.home()/'control_backend/tests/integration_results/hardware_before.json';p.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
n.destroy_node();rclpy.try_shutdown()
