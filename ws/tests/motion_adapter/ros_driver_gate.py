#!/usr/bin/env python3
"""Actual adapter + fake SDK/state on loopback 89. MC packets stay in this isolated model."""
import os,sys,time,json,signal,subprocess
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('requires isolated domain 89')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String,Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist,TransformStamped
from aimdk_msgs.msg import McCommonState,McLocomotionVelocity
from aimdk_msgs.srv import SetMcInputSource
root=Path.home()/'control_backend';out=root/'tests/integration_results'/sys.argv[1];out.mkdir()
rclpy.init();n=Node('driver_gate_fixture');status={};packets=[];checks=[];registered=[]
def sdk(q,r):
 registered.append(q.input_source.name);r.response.header.code=0;return r
n.create_service(SetMcInputSource,'/aimdk_5Fmsgs/srv/SetMcInputSource',sdk)
def report(m):
 global status
 status=json.loads(m.data)
n.create_subscription(String,'/gate_driver/status',report,10)
n.create_subscription(McLocomotionVelocity,'/aima/mc/locomotion/velocity',lambda m:packets.append((time.monotonic(),m.forward_velocity,m.lateral_velocity,m.angular_velocity)),QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
mc=n.create_publisher(McCommonState,'/aima/mc/common/state',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
tr=n.create_publisher(Bool,'/driver_fixture/trust',1);tf=n.create_publisher(TFMessage,'/driver_fixture/tf',1)
cmd=n.create_publisher(Twist,'/qazbot_d1_map/cmd_vel_authorized',1)
m=McCommonState();m.body_status.value=1;m.fsm_state.current_state=4;m.input_source.name='rc'
m.speed_status.speed_mode.value=1
for bounds in (m.speed_status.forward_bounds,m.speed_status.lateral_bounds,m.speed_status.angular_bounds):
 bounds.min_value=-.8;bounds.max_value=.8
send=False
def tick():
 mc.publish(m);tr.publish(Bool(data=True))
 t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.header.stamp=n.get_clock().now().to_msg();t.transform.rotation.w=1.
 tf.publish(TFMessage(transforms=[t]))
 if send:
  v=Twist();v.linear.x=.3;cmd.publish(v)
def wait(pred,limit=12):
 end=time.monotonic()+limit
 while not pred():
  if p.poll() is not None:raise RuntimeError('adapter exited')
  if time.monotonic()>end:raise RuntimeError('diagnostic deadline: '+str(status))
  tick();rclpy.spin_once(n,timeout_sec=.02)
def observe(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:tick();rclpy.spin_once(n,timeout_sec=.02)
def check(name,ok):
 checks.append(dict(name=name,ok=bool(ok)));print(name,bool(ok),flush=True)
 if not ok:raise AssertionError(name)
def call(name):
 c=n.create_client(Trigger,'/gate_driver/'+name);wait(c.service_is_ready)
 f=c.call_async(Trigger.Request());wait(f.done);r=f.result();n.destroy_client(c);return r
with (out/'node.log').open('w') as log:
 p=subprocess.Popen(['python3',str(root/'ws/src/navigation/x2_motion_adapter.py'),'--ros-args','-r','__node:=gate_driver','-p','live:=true',
   '-p','tf_topic:=/driver_fixture/tf','-p','trust_topic:=/driver_fixture/trust','-p','lock_path:='+str(out/'owner.lock')],stdout=log,stderr=subprocess.STDOUT)
 try:
  wait(lambda:status.get('registration')=='not_requested')
  send=True;observe(.3)
  check('commands_before_registration_create_no_MC_endpoint',not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
  check('explicit_registration_requested',call('register').success)
  wait(lambda:status.get('registration')=='registered' and status.get('ready'))
  observe(.3)
  check('registered_but_unarmed_drops_old_goal_stream',not packets and not status['motion_armed'])
  send=False;observe(.2)
  check('new_goal_arm_accepted',call('arm_motion').success)
  observe(.2);check('arm_alone_does_not_move',not packets)
  m.input_source.name='sdk_qazbot_nav2';send=True
  wait(lambda:any(v>.29 for _,v,_,_ in packets));check('fresh_command_after_arm_passes',True)
  m.input_source.name='rc'
  wait(lambda:status.get('state')=='latched')
  check('operator_takeover_latches',status.get('latch')=='operator_takeover')
  m.input_source.name='sdk_qazbot_nav2';observe(.2)
  check('takeover_closes_motion_arm',not status['motion_armed'])
  check('explicit_resume_accepted',call('resume').success)
  wait(lambda:status.get('state')=='idle' and not status.get('motion_armed'))
  mark=len(packets);observe(.4)
  check('resume_does_not_replay_stream',not any(abs(v)+abs(y)+abs(w)>1e-9 for _,v,y,w in packets[mark:]))
  send=False;check('new_goal_rearm_accepted',call('arm_motion').success)
  observe(.2);mark=len(packets);send=True
  wait(lambda:any(v>.29 for _,v,_,_ in packets[mark:]))
  check('fresh_new_goal_after_resume_passes',True)
  check('only_one_registration_request',registered==['sdk_qazbot_nav2'])
 finally:
  send=False
  if p.poll() is None:p.send_signal(signal.SIGINT)
  code=p.wait(timeout=12)
check('clean_shutdown',code==0)
(out/'result.json').write_text(json.dumps(dict(checks=checks,node_exit=code,registration_requests=registered,
 scope='Actual ROS adapter, fake SDK/state; loopback domain 89 only, no hardware commands'),indent=2))
n.destroy_node();rclpy.try_shutdown()
