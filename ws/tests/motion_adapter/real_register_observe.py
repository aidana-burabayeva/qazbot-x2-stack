#!/usr/bin/env python3
"""Authorized S1 only: one real source-registration request; never arms or publishes velocity."""
import argparse,os,sys,time,json,signal,subprocess
from pathlib import Path
a=argparse.ArgumentParser();a.add_argument('--register-once',action='store_true',required=True);a.add_argument('result_name');args=a.parse_args()
if os.environ.get('ROS_DOMAIN_ID','0') not in ('','0') or os.environ.get('ROS_LOCALHOST_ONLY','0')=='1':
 raise RuntimeError('S1 requires the actual domain 0; do not confuse with model tests')
profile=Path(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']).read_text()
if '10.0.1.41' not in profile or '192.168.1.66' not in profile:raise RuntimeError('expected X2 hardware profile')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from aimdk_msgs.msg import McCommonState,McLocomotionVelocity
root=Path.home()/'control_backend';out=root/'tests/integration_results'/args.result_name;out.mkdir()
rclpy.init();n=Node('qazbot_S1_registration_observer')
states=[];packets=[];status={};checks=[];p=None;exit_code=None;error=None
def state(m):states.append(dict(source=m.input_source.name,body=m.body_status.value,fsm=m.fsm_state.current_state,received_s=time.time()))
def velocity(m):packets.append(dict(source=m.source,forward=m.forward_velocity,lateral=m.lateral_velocity,angular=m.angular_velocity))
def adapter(m):
 global status
 status=json.loads(m.data)
q=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(McCommonState,'/aima/mc/common/state',state,q)
n.create_subscription(McLocomotionVelocity,'/aima/mc/locomotion/velocity',velocity,q)
n.create_subscription(String,'/qazbot_d1_map/motion_adapter/status',adapter,10)
def wait(pred,seconds=15):
 end=time.monotonic()+seconds
 while not pred():
  if p is not None and p.poll() is not None:raise RuntimeError('adapter exited')
  if time.monotonic()>end:raise RuntimeError('S1 diagnostic observation deadline; no retry: '+str(status))
  rclpy.spin_once(n,timeout_sec=.05)
def check(name,ok,detail=None):
 checks.append(dict(name=name,ok=bool(ok),detail=detail));print(json.dumps(checks[-1]),flush=True)
 if not ok:raise AssertionError(name)
def own_endpoint():
 return [x for x in n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity') if x.node_name=='motion_adapter' and x.node_namespace=='/qazbot_d1_map']
log=(out/'adapter.log').open('w')
try:
 wait(lambda:len(states)>=5)
 check('standing_stable_RC_before',all(x['body']==1 and x['fsm']==4 and x['source']=='rc' for x in states),states[-1])
 check('no_authorized_command_source',not n.get_publishers_info_by_topic('/qazbot_d1_map/cmd_vel_authorized'))
 check('no_existing_adapter_endpoint',not own_endpoint())
 check('no_existing_sdk_velocity',not any(x['source']=='sdk_qazbot_nav2' for x in packets))
 p=subprocess.Popen(['bash',str(root/'ws/src/navigation/run_x2_driver.sh'),'-p','live:=true','-p','priority:=30'],stdout=log,stderr=subprocess.STDOUT)
 wait(lambda:status.get('registration')=='not_requested' and status.get('sdk_service_ready'))
 check('waiting_unarmed_without_MC_endpoint',not status.get('motion_armed') and not own_endpoint())
 client=n.create_client(Trigger,'/qazbot_d1_map/motion_adapter/register');wait(client.service_is_ready)
 future=client.call_async(Trigger.Request())  # The sole explicit request to the adapter.
 wait(future.done);response=future.result()
 check('single_registration_request_submitted',response.success,response.message)
 wait(lambda:status.get('registration') in ('registered','failed'))
 check('SDK_registration_ack_success',status.get('registration')=='registered',status)
 count=len(states);wait(lambda:len(states)>=count+20 and bool(own_endpoint()))
 check('motion_arm_remains_closed',not status.get('motion_armed'))
 check('MC_source_remains_RC',all(x['source']=='rc' and x['body']==1 and x['fsm']==4 for x in states[count:]),states[-1])
 check('no_sdk_velocity_messages',not any(x['source']=='sdk_qazbot_nav2' for x in packets))
 check('no_command_publisher_appeared',not n.get_publishers_info_by_topic('/qazbot_d1_map/cmd_vel_authorized'))
except BaseException as exc:
 error=str(exc)
 raise
finally:
 if p is not None:
  if p.poll() is None:p.send_signal(signal.SIGINT)
  exit_code=p.wait(timeout=15)
  # Continue observing after the adapter exits. No new actor is started.
  count=len(states);end=time.monotonic()+10
  while time.monotonic()<end and (own_endpoint() or len(states)<count+5):
   rclpy.spin_once(n,timeout_sec=.05)
 log.close()
 result=dict(checks=checks,adapter_exit=exit_code,last_status=status,error=error,
  SDK_velocity_messages=sum(x['source']=='sdk_qazbot_nav2' for x in packets),
  total_velocity_messages_observed=len(packets),mc_states=states,
  sdk_endpoint_after_stop=bool(own_endpoint()),registration_action='ADD',priority=30,
  motion_arm_service_called=False,velocity_publisher_created_by_observer=False)
 (out/'result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps({k:result[k] for k in ['adapter_exit','last_status','error','SDK_velocity_messages','sdk_endpoint_after_stop']}),flush=True)
 n.destroy_node();rclpy.try_shutdown()
 if exit_code not in (None,0):raise RuntimeError('S1 adapter unclean shutdown')
 if result['SDK_velocity_messages']:raise RuntimeError('Unexpected SDK velocity packet observed')
