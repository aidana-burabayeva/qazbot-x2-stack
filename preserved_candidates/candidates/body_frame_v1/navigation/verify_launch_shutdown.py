#!/usr/bin/env python3
"""Verify that loss of one critical candidate child stops the complete stack.

Runs only on an isolated DDS domain; no physical input topics are available.
Observation deadlines bound this test, not the runtime failure policy.
"""
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState

if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('Requires isolated domain87 and localhost-only transport')
root=Path(__file__).resolve().parent.parent
out=Path(sys.argv[1]);out.mkdir(exist_ok=False)
rclpy.init();observer=rclpy.create_node('qazbot_launch_shutdown_observer')
active=set();clients={};requests={}
def transition(msg,name):
    if msg.goal_state.id==3:active.add(name)
for name in ['map_server','amcl']:
    observer.create_subscription(TransitionEvent,'/qazbot_d1_map/'+name+'/transition_event',lambda m,n=name:transition(m,n),100)
    clients[name]=observer.create_client(GetState,'/qazbot_d1_map/'+name+'/get_state')
def command(pid):
    try:return Path('/proc/'+str(pid)+'/cmdline').read_bytes().split(b'\0')
    except FileNotFoundError:return None
def children(pid):
    # This Tegra kernel does not expose /proc/.../children. PPID is available
    # through procps; a missing optional proc file is not process termination.
    rows=subprocess.check_output(['ps','-eo','pid=,ppid='],text=True).splitlines()
    return [int(child) for child,parent in (row.split() for row in rows) if int(parent)==pid]
log=(out/'launch.log').open('w')
proc=subprocess.Popen(['ros2','launch',str(root/'navigation/localization.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
tracked={}
try:
    end=time.monotonic()+30
    while True:
        if proc.poll() is not None:raise RuntimeError('Candidate launch exited before failure injection')
        tracked={pid:command(pid) for pid in children(proc.pid)}
        adapter=[pid for pid,args in tracked.items() if args and str(root/'navigation/body_pose_adapter.py').encode() in args]
        rclpy.spin_once(observer,timeout_sec=.05)
        for name,client in clients.items():
            if name not in requests and client.service_is_ready():requests[name]=client.call_async(GetState.Request())
            future=requests.get(name)
            if future is not None and future.done() and future.result().current_state.id==3:active.add(name)
        initialized_adapter=('qazbot_body_pose_adapter','/') in observer.get_node_names_and_namespaces()
        if adapter and len(tracked)>=6 and active=={'map_server','amcl'} and initialized_adapter:break
        if time.monotonic()>end:raise RuntimeError('Candidate children did not become ready')
    os.kill(adapter[0],signal.SIGINT)
    rc=proc.wait(timeout=20)
    remaining=[pid for pid,args in tracked.items() if args and command(pid)==args]
    abnormal=[{'pid':int(pid),'exit_code':int(code)} for pid,code in
              re.findall(r'process has died \[pid (\d+), exit code (-?\d+)',(out/'launch.log').read_text())]
    result={'injected_exit_child':adapter[0],'tracked_child_count':len(tracked),
            'active_before_injection':sorted(active),
            'launch_exit_code':rc,'remaining_candidate_children':remaining,
            'abnormal_child_exits':abnormal,
            'passed':not remaining and rc==0 and not abnormal,
            'qualification':'Process-dependency test only; no hardware sensor-loss or stopping-distance claim.'}
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise RuntimeError('Coherent shutdown test failed')
finally:
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=20)
    log.close()
    observer.destroy_node();rclpy.shutdown()
