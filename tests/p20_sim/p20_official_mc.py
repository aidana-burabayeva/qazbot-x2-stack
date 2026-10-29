#!/usr/bin/env python3
"""SIM ONLY: actual P20 ROS adapter -> official MC -> MuJoCo -> torso FK yaw surrogate (native leg stream is absent).
Calibration scope: no AMCL/Nav2 goals; localisation is deliberately excluded to isolate P20.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

if os.environ.get('TWIN_SANDBOX') != '1' or os.environ.get('ROS_DOMAIN_ID') != '87':
    raise RuntimeError('isolated twin only')
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from aimdk_msgs.msg import McCommonState, McInputAction
from aimdk_msgs.srv import SetMcInputSource


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--loop', choices=['true', 'false'], required=True)
    ap.add_argument('--direction', type=int, choices=[-1, 1], required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    source = 'sdk_p20_sim_probe'
    rclpy.init()
    n = rclpy.create_node('p20_official_probe')
    obs = {'odom': [], 'mc': {}, 'status': {}}
    processes, logs = [], []
    def spawn(script, args, log):
        f = (out / log).open('w'); logs.append(f)
        p = subprocess.Popen(['python3', '/backend/ws/src/' + script, '--ros-args', *args], stdout=f, stderr=f)
        processes.append(p)
    relay_log = (out / 'torso_odom.log').open('w'); logs.append(relay_log)
    processes.append(subprocess.Popen(['/tools/torso_odom'], stdout=relay_log, stderr=relay_log))
    spawn('navigation/x2_motion_adapter.py', ['-p', 'live:=true', '-p', 'source_name:=' + source,
          '-p', 'require_localization:=false', '-p', 'require_trust:=false',
          '-p', 'yaw_loop:=' + a.loop, '-p', 'odom_topic:=/twin/p20_odom', '-p', 'lock_path:=/tmp/p20-sim.lock'], 'adapter.log')
    start = time.monotonic()
    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        obs['odom'].append([time.monotonic(), p.x, p.y,
          math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
    def on_mc(m):
        obs['mc'] = dict(body=m.body_status.value, fsm=m.fsm_state.current_state,
                         source=m.input_source.name, action=m.action_info.action_desc, received=time.monotonic())
    n.create_subscription(Odometry, '/twin/p20_odom', on_odom, qos_profile_sensor_data)
    n.create_subscription(McCommonState, '/aima/mc/common/state', on_mc, qos_profile_sensor_data)
    n.create_subscription(String, '/motion_adapter/status', lambda m: obs.update(status=json.loads(m.data)), 10)
    pub = n.create_publisher(Twist, '/qazbot_d1_map/cmd_vel_authorized', 10)
    commands = []
    def wait(pred, seconds=15):
        end = time.monotonic() + seconds
        while not pred():
            if time.monotonic() > end:
                raise RuntimeError('condition timeout ' + repr(obs['status']) + repr(obs['mc']))
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('owned child exited; inspect logs')
            rclpy.spin_once(n, timeout_sec=.02)
    def service(name):
        c = n.create_client(Trigger, name)
        if not c.wait_for_service(timeout_sec=10): raise RuntimeError(name + ' unavailable')
        f = c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        if not f.done() or not f.result().success: raise RuntimeError(name + ' refused ' + str(f.result()))
        n.destroy_client(c)
    def send(v, w):
        now = time.monotonic()
        if not obs['odom'] or now - obs['odom'][-1][0] > .5:
            raise RuntimeError('native odometry stale')
        if obs['mc'].get('body') != 1 or obs['mc'].get('fsm') not in (4, 5):
            raise RuntimeError('MC not standing ' + str(obs['mc']))
        if obs['status'].get('latch'): raise RuntimeError('adapter latched ' + str(obs['status']))
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); pub.publish(m)
        commands.append([now, v, w])
    def hold(seconds, v=0., w=0.):
        begin = time.monotonic(); next_tick = begin
        while time.monotonic() - begin < seconds:
            now = time.monotonic()
            if now >= next_tick:
                send(v, w); next_tick = now + .02
            rclpy.spin_once(n, timeout_sec=.005)
        return begin, time.monotonic()
    result = {'loop': a.loop, 'direction': a.direction, 'scope': __doc__, 'source': source}
    try:
        wait(lambda: len(obs['odom']) > 5 and obs['status'].get('foreign_publishers') == [] and obs['mc'].get('body') == 1)
        service('/motion_adapter/register')
        wait(lambda: obs['status'].get('registration') == 'registered')
        service('/motion_adapter/arm_motion')
        hold(1., v=.3)
        hold(.8)
        begin, end = hold(6., w=a.direction * .5)
        hold(2.)
        rows = [r for r in obs['odom'] if begin + 1.5 <= r[0] <= end]
        signed = sum(math.remainder(b[3]-x[3], 2*math.pi) for x,b in zip(rows,rows[1:]))
        result['sustained_rate'] = signed / (rows[-1][0]-rows[0][0])
        hold(1., v=.3); hold(.8)
        yaw0 = obs['odom'][-1][3]; goal = a.direction * math.pi/2
        begin = time.monotonic(); aligned = None
        while time.monotonic()-begin < 10:
            angle = math.remainder(obs['odom'][-1][3]-yaw0, 2*math.pi)
            err = math.remainder(goal-angle, 2*math.pi)
            if abs(err) <= .17:
                aligned = time.monotonic()-begin; break
            hold(.04, w=math.copysign(min(.5,max(.1,abs(err))),err))
        hold(2.)
        result.update(alignment_time_s=aligned, final_error=math.remainder(goal-math.remainder(obs['odom'][-1][3]-yaw0,2*math.pi),2*math.pi),
                      adapter_end=obs['status'], mc_end=obs['mc'], abort=None)
    except Exception as exc:
        result['abort'] = str(exc)
    finally:
        pub.publish(Twist())
        for p in reversed(processes): p.terminate()
        for p in reversed(processes):
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired: p.kill(); p.wait()
        c = n.create_client(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
        if c.wait_for_service(timeout_sec=2):
            req=SetMcInputSource.Request(); req.action.value=McInputAction.INPUTACTION_DELETE
            req.input_source.name=source; req.request.header.stamp=n.get_clock().now().to_msg()
            f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=3)
            result['delete_code'] = f.result().response.header.code if f.done() and f.result() else None
        for f in logs: f.close()
        result['duration_s']=time.monotonic()-start
        (out/'result.json').write_text(json.dumps(result,indent=2))
        (out/'trace.json').write_text(json.dumps({'odom':obs['odom'],'commands':commands}))
        print(json.dumps(result,indent=2),flush=True)
        n.destroy_node();rclpy.shutdown()
    return 1 if result.get('abort') else 0

if __name__ == '__main__':
    raise SystemExit(main())
