#!/usr/bin/env python3
"""Fake SDK on loopback domain 89. No hardware RPC, no velocity commands."""
import json, os, signal, subprocess, sys, time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('isolated domain 89 required')
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from aimdk_msgs.srv import SetMcInputSource
from aimdk_msgs.msg import McLocomotionVelocity
root=Path.home()/'control_backend';out=root/'tests/integration_results'/sys.argv[1];out.mkdir()
rclpy.init();n=Node('fake_sdk_registration_fixture')
requests=[];states={};velocities=[];checks=[];reply_code=99
async def sdk(req,resp):
 requests.append(dict(action=req.action.value,name=req.input_source.name,priority=req.input_source.priority))
 await reply_gate
 resp.response.header.code=reply_code
 return resp
srv=n.create_service(SetMcInputSource,'/aimdk_5Fmsgs/srv/SetMcInputSource',sdk,callback_group=ReentrantCallbackGroup())
n.create_subscription(McLocomotionVelocity,'/aima/mc/locomotion/velocity',velocities.append,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
def wait(pred,limit=12):
 end=time.monotonic()+limit
 while not pred():
  if time.monotonic()>end:raise RuntimeError('diagnostic observation deadline: '+str(states))
  rclpy.spin_once(n,timeout_sec=.02)
def check(name,ok):
 checks.append(dict(name=name,ok=bool(ok)));print(name,bool(ok),flush=True)
 if not ok:raise AssertionError(name)
def invoke(client):
 wait(client.service_is_ready);f=client.call_async(Trigger.Request());wait(f.done);return f.result()
codes=[]
for case,reply_code in [('failure',99),('success',0)]:
 reply_gate=Future()
 name='adapter_registration_'+case
 sub=n.create_subscription(String,'/'+name+'/status',lambda m,k=case:states.__setitem__(k,json.loads(m.data)),10)
 reg=n.create_client(Trigger,'/'+name+'/register')
 with (out/(case+'.log')).open('w') as log:
  p=subprocess.Popen(['python3',str(root/'ws/src/navigation/x2_motion_adapter.py'),'--ros-args',
    '-r','__node:='+name,'-p','live:=true','-p','lock_path:='+str(out/(case+'.lock'))],stdout=log,stderr=subprocess.STDOUT)
  start_count=len(requests)
  try:
   wait(lambda:states.get(case,{}).get('registration')=='not_requested')
   check(case+'_no_automatic_SDK_request',len(requests)==start_count)
   check(case+'_no_MC_endpoint_before_registration',not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
   wait(lambda:(name,'/') in n.get_node_names_and_namespaces())
   topics=n.get_subscriber_names_and_types_by_node(name,'/')
   check(case+'_uses_authorized_input',any(t=='/qazbot_d1_map/cmd_vel_authorized' for t,_ in topics))
   check(case+'_explicit_request_accepted',invoke(reg).success)
   wait(lambda:len(requests)==start_count+1 and states.get(case,{}).get('registration')=='pending')
   check(case+'_pending_has_no_MC_endpoint',not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
   check(case+'_pending_duplicate_refused',not invoke(reg).success)
   check(case+'_pending_has_single_request',len(requests)==start_count+1)
   reply_gate.set_result(None)
   expected='failed' if reply_code else 'registered'
   wait(lambda:states.get(case,{}).get('registration')==expected)
   check(case+'_one_SDK_request',len(requests)==start_count+1)
   check(case+'_second_request_refused',not invoke(reg).success)
   check(case+'_no_retry_or_fallback',len(requests)==start_count+1)
   endpoints=n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity')
   check(case+'_MC_endpoint_policy',len(endpoints)==(0 if reply_code else 1))
   check(case+'_idle_sends_no_MC_messages',not velocities)
  finally:
   if p.poll() is None:p.send_signal(signal.SIGINT)
   code=p.wait(timeout=12);codes.append(code)
  check(case+'_clean_exit',code==0)
  check(case+'_shutdown_idle_sends_no_MC_messages',not velocities)
 n.destroy_subscription(sub);n.destroy_client(reg)
 wait(lambda:not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
(out/'result.json').write_text(json.dumps(dict(checks=checks,node_exits=codes,requests=requests,MC_messages=len(velocities),scope='fake SDK only; loopback domain 89'),indent=2))
n.destroy_node();rclpy.try_shutdown()
