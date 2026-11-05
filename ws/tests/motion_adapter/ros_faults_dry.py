#!/usr/bin/env python3
"""Complete adapter node with synthetic SDK state, isolated domain 89, live=false.
No MC velocity messages are published. A foreign endpoint is advertised only to
test graph diagnosis. No SetMcInputSource server is present or invoked.
"""
import os,json,subprocess,signal,time,sys
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='89':raise RuntimeError('requires isolated domain 89')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist,TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool,String
from aimdk_msgs.msg import McCommonState,McLocomotionVelocity
root=Path.home()/'control_backend';out=root/'tests/integration_results'/sys.argv[1];out.mkdir()
rclpy.init();n=Node('adapter_fault_fixture');status={};dry=[];checks=[]
def on_status(m):
 global status
 status=json.loads(m.data)
n.create_subscription(String,'/adapter_test_node/status',on_status,10)
n.create_subscription(Twist,'/adapter_test_node/dry_run_cmd',dry.append,10)
cmd=n.create_publisher(Twist,'/adapter_test/cmd',1)
tr=n.create_publisher(Bool,'/adapter_test/trust',1);tf=n.create_publisher(TFMessage,'/adapter_test/tf',1)
state=n.create_publisher(McCommonState,'/aima/mc/common/state',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
m=McCommonState();m.body_status.value=1;m.fsm_state.current_state=4;m.input_source.name='rc';sp=m.speed_status;sp.speed_mode.value=1
sp.forward_bounds.min_value=-1.;sp.forward_bounds.max_value=.8;sp.lateral_bounds.min_value=-1.;sp.lateral_bounds.max_value=1.;sp.angular_bounds.min_value=-1.;sp.angular_bounds.max_value=1.
allow=True
v=Twist();v.linear.x=.3
f=(out/'node.log').open('w');p=subprocess.Popen(['python3',str(root/'ws/src/navigation/x2_motion_adapter.py'),'--ros-args','-r','__node:=adapter_test_node','-p','live:=false','-p','cmd_vel_topic:=/adapter_test/cmd','-p','tf_topic:=/adapter_test/tf','-p','trust_topic:=/adapter_test/trust'],stdout=f,stderr=subprocess.STDOUT)
def pump(pred,limit=8):
 end=time.monotonic()+limit
 while not pred():
  if p.poll() is not None:raise RuntimeError('adapter exited')
  if time.monotonic()>end:raise RuntimeError('diagnostic deadline: '+str(status))
  t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.header.stamp=n.get_clock().now().to_msg();t.transform.rotation.w=1.
  tf.publish(TFMessage(transforms=[t]));tr.publish(Bool(data=allow));state.publish(m);cmd.publish(v)
  rclpy.spin_once(n,timeout_sec=.02)
def check(name,ok):
 checks.append(dict(name=name,ok=bool(ok)));print(name,ok,flush=True)
 if not ok:raise AssertionError(name)
try:
 pump(lambda:status.get('reason')=='dry_run' and dry)
 check('dry_node_passes_valid_command',abs(dry[-1].linear.x-.3)<1e-9)
 check('no_MC_publisher',not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
 clients=n.get_client_names_and_types_by_node('adapter_test_node','/')
 check('no_registration_client',not any('SetMcInputSource' in name for name,_ in clients))
 allow=False;pump(lambda:status.get('reason')=='localization_untrusted')
 check('untrusted_refused',status['state']=='dry_run')
 allow=True;pump(lambda:status.get('reason')=='dry_run')
 m.input_source.name='pnc';pump(lambda:status.get('reason')=='vendor_autonomy_active')
 check('vendor_autonomy_refused',status['state']=='dry_run')
 m.input_source.name='rc';pump(lambda:status.get('reason')=='dry_run')
 foreign=n.create_publisher(McLocomotionVelocity,'/aima/mc/locomotion/velocity',1)
 pump(lambda:status.get('reason')=='foreign_velocity_publisher')
 check('foreign_endpoint_refused',status['state']=='dry_run')
 n.destroy_publisher(foreign);pump(lambda:status.get('reason')=='dry_run')
 check('no_MC_publisher_after_test',not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
finally:
 if p.poll() is None:p.send_signal(signal.SIGINT)
 code=p.wait(timeout=10);f.close();n.destroy_node();rclpy.try_shutdown()
 (out/'result.json').write_text(json.dumps(dict(checks=checks,node_exit=code,domain=89,live=False),indent=2))
 if code:raise RuntimeError('unclean adapter shutdown')
