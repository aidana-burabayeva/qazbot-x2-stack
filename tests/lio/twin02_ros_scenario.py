#!/usr/bin/env python3
"""Isolated DDS TWIN-02 replay: compiled source/recovery nodes, real TrustEvaluator/AdapterCore.
No AMCL, Nav2 action or physical robot. All poses belong to the simulated test fixture.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from aimdk_msgs.msg import McCommonState
from qazbot_odom_recovery_msgs.srv import RecoverOdometry
import yaml

ROOT=Path('/backend')
sys.path.insert(0,str(ROOT/'tests/loc_trust'))
from test_loc_trust import L, make_inputs, room_grid, simulate_scan
from x2_motion_adapter import AdapterCore, Config, McState, LATCHED


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--jump',type=float,required=True);ap.add_argument('--gap',type=float,required=True)
    ap.add_argument('--announced',choices=['true','false'],required=True);ap.add_argument('--out',required=True);ap.add_argument('--invalid-reset',action='store_true');a=ap.parse_args()
    if os.environ.get('TWIN_SANDBOX')!='1' or os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('SIM only')
    out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    params={'twin02_replay_source':{'ros__parameters':{'jump_m':a.jump,'gap_s':a.gap,'announced_reset':a.announced=='true','invalid_after_reset':a.invalid_reset}}}
    (out/'source.yaml').write_text(yaml.safe_dump(params))
    rclpy.init();n=rclpy.create_node('twin02_scenario');procs=[];logs=[];trace=[];events=[]
    obs={'odom':None,'raw':None,'status':None,'mc':None,'raw_count':0,'odom_count':0,'max_odom_step':0.}
    trust=L.TrustEvaluator(room_grid(),L.TrustConfig(admission_policy='strict'))
    adapter=AdapterCore(Config());active=False;released=False;baseline=None;scan=None;last_verdict=None
    def spawn(exe,extra=()):
        f=(out/(exe+'.log')).open('w');logs.append(f)
        p=subprocess.Popen(['/work/install/qazbot_odom_recovery/lib/qazbot_odom_recovery/'+exe,*extra],stdout=f,stderr=f);procs.append(p)
    def pose(m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        return p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def odom(m):
        p=pose(m)
        if obs['odom']:
            before=obs['odom']['pose'];obs['max_odom_step']=max(obs['max_odom_step'],math.hypot(p[0]-before[0],p[1]-before[1]))
        obs['odom']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':p};obs['odom_count']+=1
    def raw(m):obs.update(raw=m,raw_count=obs['raw_count']+1)
    def status(m):
        d=m.status[0];obs['status']={v.key:v.value for v in d.values};obs['status']['receipt']=time.monotonic()
    def mc(m):
        action=m.action_info.action_desc
        if not obs['mc'] or obs['mc']['action']!=action:events.append({'event':'MC','action':action,'at':time.monotonic()})
        obs['mc']={'action':action,'body':m.body_status.value,'fsm':m.fsm_state.current_state,'receipt':time.monotonic()}
    n.create_subscription(Odometry,'/twin02/odom',odom,10)
    n.create_subscription(Odometry,'/twin02/leg_source',raw,qos_profile_sensor_data)
    n.create_subscription(DiagnosticArray,'/twin02/status',status,10)
    n.create_subscription(McCommonState,'/twin02/mc_state',mc,qos_profile_sensor_data)
    client=n.create_client(RecoverOdometry,'/twin02/recover')
    def inputs(t):
        o=obs['odom'];return make_inputs(t,(0.,0.,0.),o['pose'],scan,odom_stamp=o['stamp'],odom_msg_stamp=o['stamp'])
    def tick():
        nonlocal last_verdict
        if not active:return
        t=n.get_clock().now().nanoseconds*1e-9
        last_verdict=trust.evaluate(inputs(t))
        st=obs['status'] or {};m=obs['mc'] or {}
        gate=st.get('recovery_gate')=='open' and time.monotonic()-st.get('receipt',0)<.5
        allowed=bool(last_verdict['motion_ok'] and gate)
        state=McState(stamp=t if time.monotonic()-m.get('receipt',0)<.5 else t-1,source='rc',body=m.get('body',0),fsm=m.get('fsm',0))
        d=adapter.step(t,(.3,0.,0.),t,state,localized=allowed)
        if not allowed:assert d.publish in (None,(0.,0.,0.)),d
        trace.append({'t':t,'state':st.get('state'),'gate':gate,'trust':last_verdict['motion_ok'],
                      'reasons':last_verdict['reasons'],'adapter':d.state,'output':d.publish,'pose':obs['odom']['pose']})
    def wait(pred,timeout=15):
        end=time.monotonic()+timeout
        while not pred():
            if time.monotonic()>end:raise RuntimeError('condition timeout '+repr(obs['status'])+repr(obs['mc']))
            if any(p.poll() is not None for p in procs):raise RuntimeError('owned C++ node exited')
            rclpy.spin_once(n,timeout_sec=.02);tick()
    def request(command,proof=False,newtrust=False):
        req=RecoverOdometry.Request();req.command=command;req.epoch=1;req.motion_inhibited=True
        req.source_reset_confirmed=proof;req.newly_qualified_trust=newtrust
        if obs['raw'] and baseline:
            req.verified_pose.header.stamp=copy.copy(obs['raw'].header.stamp);req.verified_pose.header.frame_id='odom'
            req.verified_pose.pose.position.x,req.verified_pose.pose.position.y=baseline[:2]
            req.verified_pose.pose.orientation.z=math.sin(baseline[2]/2);req.verified_pose.pose.orientation.w=math.cos(baseline[2]/2)
        fut=client.call_async(req);rclpy.spin_until_future_complete(n,fut,timeout_sec=5)
        assert fut.done() and fut.result() is not None
        result=fut.result();events.append({'event':'service','command':command,'accepted':result.success,'state':result.state})
        return result.success
    summary={'jump':a.jump,'gap':a.gap,'announced':a.announced,'invalid_reset':a.invalid_reset,'scope':__doc__}
    try:
        spawn('recovery_node')
        assert client.wait_for_service(timeout_sec=15)
        spawn('replay_source',('--ros-args','--params-file',str(out/'source.yaml')))
        wait(lambda:obs['odom_count']>=3 and obs['status'] and obs['status']['recovery_gate']=='open')
        baseline=obs['odom']['pose'];scan=simulate_scan(room_grid(),baseline)
        trust.confirm_anchor(inputs(n.get_clock().now().nanoseconds*1e-9));active=True;tick()
        assert last_verdict['motion_ok']
        if a.announced=='true':
            wait(lambda:obs['mc']['action']=='DAMPING_DEFAULT')
            trust.relocalize();tick()
            assert request(RecoverOdometry.Request.BEGIN)
            wait(lambda:obs['status']['state']=='1')
        else:
            initial_count=obs['raw_count']
            wait(lambda:obs['raw_count']>initial_count and (math.isnan(obs['raw'].pose.pose.position.x) or obs['raw'].pose.pose.position.x>.4),timeout=a.gap+8)
        wait(lambda:obs['raw'] and (math.isnan(obs['raw'].pose.pose.position.x) or obs['raw'].pose.pose.position.x>.4) and obs['mc']['action']=='STAND_DEFAULT',timeout=a.gap+8)
        if a.announced=='true':
            assert not request(RecoverOdometry.Request.CONFIRM,proof=False)
            assert request(RecoverOdometry.Request.CONFIRM,proof=True)
            count=obs['odom_count'];wait(lambda:obs['odom_count']>=count+3 and obs['status']['state']=='2')
            assert tuple(obs['odom']['pose'])==__import__('pytest').approx(baseline,abs=1e-8)
            assert not last_verdict['motion_ok']
            assert not request(RecoverOdometry.Request.RELEASE,newtrust=False)
            trust.confirm_anchor(inputs(n.get_clock().now().nanoseconds*1e-9));tick();assert last_verdict['motion_ok']
            assert request(RecoverOdometry.Request.RELEASE,newtrust=True)
            wait(lambda:obs['status']['recovery_gate']=='open')
            assert adapter.state==LATCHED
            t=n.get_clock().now().nanoseconds*1e-9;assert adapter.resume(now=t)
            wait(lambda:adapter.state=='active');released=True
        else:
            wait(lambda:obs['status']['state']=='3')
            assert not request(RecoverOdometry.Request.CONFIRM,proof=True)
            assert obs['status']['recovery_gate']=='closed'
            assert adapter.state==LATCHED
        assert obs['max_odom_step']<1e-8
        summary.update(result='PASS',recovered=released,max_output_step_m=obs['max_odom_step'],raw_samples=obs['raw_count'],odom_samples=obs['odom_count'])
    except Exception as e:
        summary.update(result='FAIL',error=repr(e),observations={k:v for k,v in obs.items() if k!='raw'})
    finally:
        for p in reversed(procs):p.terminate()
        for p in reversed(procs):
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        (out/'result.json').write_text(json.dumps(summary,indent=2));(out/'trace.json').write_text(json.dumps(trace));(out/'events.json').write_text(json.dumps(events,indent=2))
        print(json.dumps(summary,indent=2),flush=True);n.destroy_node();rclpy.shutdown()
    return 0 if summary['result']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
