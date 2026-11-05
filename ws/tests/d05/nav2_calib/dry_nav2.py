#!/usr/bin/env python3
"""Dry integration of nav2_calib.py against the REAL Humble 1.1.18 controller_server + velocity_smoother binaries
(live parameter file) in an isolated domain. No robot topics, no GC/adapter (stub status JSON only), robot never moves
(static odom->base_footprint). The HARNESS (not the tool) sends FollowPath goals to emulate the operator goal.
Usage: dry_nav2.py LIVE_PARAMS_YAML OUT_DIR
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

PARAMS, OUT = sys.argv[1], sys.argv[2]
NS = '/qazbot_d1_map'
TOOL = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'd05', 'nav2_calib', 'nav2_calib.py')
assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'refusing to run on robot domain'
procs = []
results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'} {name} {detail}", flush=True)


def start(cmd, log):
    p = subprocess.Popen(cmd, stdout=open(os.path.join(OUT, log), 'w'), stderr=subprocess.STDOUT,
                         start_new_session=True)
    procs.append(p)
    return p


remap = ['-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static']
start(['ros2', 'run', 'nav2_controller', 'controller_server', '--ros-args', '-r', '__ns:=/qazbot_d1_map/nav2',
       '-r', '__node:=controller_server', *remap, '-r', 'cmd_vel:=cmd_vel_nav', '--params-file', PARAMS], 'ctl.log')
start(['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args', '-r', '__ns:=/qazbot_d1_map/nav2',
       '-r', '__node:=velocity_smoother', *remap, '-r', 'cmd_vel:=cmd_vel_nav', '-r', f'cmd_vel_smoothed:={NS}/cmd_vel',
       '--params-file', PARAMS], 'sm.log')
start(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', '__ns:=/qazbot_d1_map/nav2',
       '-r', '__node:=lifecycle_manager_control', '-p', 'autostart:=true',
       '-p', 'node_names:=[controller_server,velocity_smoother]'], 'lm.log')
start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'odom', '--child-frame-id',
       'base_footprint', '--ros-args', *remap], 'tf.log')

rclpy.init()
n = Node('d05_dry_harness')
gc = {'busy': False, 'latched': False, 'fault': '', 'ready': True}
ad = {'live': True, 'mode': 'live', 'registration': 'registered', 'latch': '', 'foreign_publishers': [],
      'state': 'idle', 'ready': True}
gc_pub = n.create_publisher(String, f'{NS}/goal_coordinator/status', 10)
ad_pub = n.create_publisher(String, f'{NS}/motion_adapter/status', 10)
od_pub = n.create_publisher(Odometry, '/qazbot_native/odom', 10)
lim_pub = n.create_publisher(SpeedLimit, f'{NS}/nav2/speed_limit', 10)
nav, sm, lim = [], [], []
n.create_subscription(Twist, f'{NS}/nav2/cmd_vel_nav', lambda m: nav.append((time.monotonic(), m.linear.x)), 10)
n.create_subscription(Twist, f'{NS}/cmd_vel', lambda m: sm.append((time.monotonic(), m.linear.x)), 10)
n.create_subscription(SpeedLimit, f'{NS}/nav2/speed_limit', lambda m: lim.append((m.percentage, m.speed_limit)), 10)


def tick():
    gc_pub.publish(String(data=json.dumps(gc)))
    ad_pub.publish(String(data=json.dumps(ad)))
    o = Odometry()
    o.header.stamp = n.get_clock().now().to_msg()
    o.header.frame_id, o.child_frame_id = 'odom', 'base_footprint'
    od_pub.publish(o)


n.create_timer(0.1, tick)
fp = ActionClient(n, FollowPath, f'{NS}/nav2/follow_path')


def spin(sec, until=None):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        if until and until():
            return True
    return bool(until and until())


def goal():
    g = FollowPath.Goal()
    g.path = Path()
    g.path.header.frame_id = 'odom'
    for i in range(121):   # 6 m straight, 5 cm spacing
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose.position.x = i * 0.05
        p.pose.orientation.w = 1.0
        g.path.poses.append(p)
    g.controller_id, g.goal_checker_id = 'FollowPath', 'goal_checker'
    f = fp.send_goal_async(g)
    spin(3.0, f.done)
    return f.result()


def window(buf, t0, t1):
    return [v for t, v in buf if t0 <= t <= t1]


try:
    check('controller action up', fp.wait_for_server(timeout_sec=30.0))
    spin(3.0)
    # 1) baseline: no limit -> RPP desired 0.30
    h = goal()
    t0 = time.monotonic()
    spin(3.0)
    base = window(nav, t0 + 1.5, time.monotonic())
    check('baseline RPP 0.30', bool(base) and abs(max(base) - 0.30) < 1e-3, f'max={max(base) if base else None}')
    # 2) absolute mode probe (NOT used by the tool): does an absolute limit above the base raise RPP?
    m = SpeedLimit()
    m.percentage, m.speed_limit = False, 0.5
    lim_pub.publish(m)
    t1 = time.monotonic()
    spin(2.0)
    ab_nav, ab_sm = window(nav, t1 + 0.5, time.monotonic()), window(sm, t1 + 0.5, time.monotonic())
    print(f'INFO absolute 0.5: RPP max {max(ab_nav):.3f}, smoother max {max(ab_sm):.3f}', flush=True)
    check('smoother caps at live max_velocity 0.30', max(ab_sm) <= 0.3 + 1e-6)
    m.percentage, m.speed_limit = True, 0.0
    lim_pub.publish(m)
    spin(1.0)
    n.destroy_publisher(lim_pub)   # the tool refuses while any other speed_limit publisher exists
    h.cancel_goal_async()
    spin(2.0)

    # 3) tool run at 0.15 (50 %) with recording
    env = dict(os.environ, DDS_PROFILE='none')
    tool = subprocess.Popen([sys.executable, TOOL, '--run-id', 'dry81_015', '--target-vx', '0.15', '--goal-wait-s', '60',
                             '--max-goal-s', '30', '--post-roll-s', '1.5', '--out-root', OUT, '--path-len-m', '6'],
                            stdout=open(os.path.join(OUT, 'tool.log'), 'w'), stderr=subprocess.STDOUT, env=env)
    procs.append(tool)
    check('tool published 50 % limit', spin(30.0, lambda: (True, 50.0) in [(p, round(v, 6)) for p, v in lim]))
    gc['busy'] = True
    h = goal()
    t2 = time.monotonic()
    spin(4.0)
    run_nav = window(nav, t2 + 1.0, time.monotonic())
    check('RPP output = 0.15 under limit', bool(run_nav) and abs(max(run_nav) - 0.15) < 1e-3,
          f'max={max(run_nav) if run_nav else None}')
    h.cancel_goal_async()
    spin(1.0)
    gc['busy'] = False
    rc = None
    end = time.monotonic() + 20
    while time.monotonic() < end and rc is None:
        spin(0.2)
        rc = tool.poll()
    check('tool exit 0', rc == 0, f'rc={rc}')
    check('tool restored NO_SPEED_LIMIT', (True, 0.0) in lim)
    meta = [f for f in os.listdir(OUT) if f.startswith('calib_dry81_015') and f.endswith('.meta.json')]
    md = json.load(open(os.path.join(OUT, sorted(meta)[-1]))) if meta else {}
    cmd = md.get('commanded', {})
    check('meta verdict goal_ended', md.get('verdict') == 'goal_ended', str(md.get('verdict')))
    check('meta limit_honored', cmd.get('limit_honored') is True, f"nav max {cmd.get('nav_cmd_max_vx')}")
    check('meta steady >= 2 s at 0.15', (cmd.get('steady_cmd_s') or 0) >= 2.0, f"{cmd.get('steady_cmd_s')}")
    check('meta ramp within live limits', cmd.get('ramp_within_limits') is True, json.dumps(cmd.get('ramp')))
    check('meta live params read', md.get('live_params', {}).get('controller_server', {})
          .get('FollowPath.desired_linear_vel') == 0.3)
    bag = md.get('bag')
    spin(3.0)
    check('bag written', bool(bag) and os.path.isdir(bag) and any(f.endswith('.mcap') for f in os.listdir(bag)),
          str(bag))
    # 4) after restore: a new goal is back at 0.30
    h = goal()
    t3 = time.monotonic()
    spin(3.0)
    after = window(nav, t3 + 1.5, time.monotonic())
    check('after restore RPP back to 0.30', bool(after) and abs(max(after) - 0.30) < 1e-3)
    h.cancel_goal_async()
    spin(1.0)
    # 5) refusal: GC busy at start -> exit 3, no limit published
    gc['busy'] = True
    before = len(lim)
    rc = subprocess.run([sys.executable, TOOL, '--run-id', 'dry81_busy', '--target-vx', '0.10', '--no-record',
                         '--out-root', OUT], env=env, capture_output=True, timeout=60).returncode
    spin(0.5)
    check('refused when GC busy (exit 3, no limit sent)', rc == 3 and len(lim) == before, f'rc={rc}')
    gc['busy'] = False
    # 6) envelope: 0.40 refused (exit 2)
    rc = subprocess.run([sys.executable, TOOL, '--run-id', 'dry81_040', '--target-vx', '0.40', '--no-record',
                         '--out-root', OUT], env=env, capture_output=True, timeout=60).returncode
    check('0.40 above envelope refused (exit 2)', rc == 2, f'rc={rc}')
    # 7) below regulated_linear_scaling_min_speed (0.15): 33.3 % must give 0.10, not the 0.15 floor
    lp = n.create_publisher(SpeedLimit, f'{NS}/nav2/speed_limit', 10)
    spin(1.0, lambda: lp.get_subscription_count() >= 1)
    m = SpeedLimit()
    m.percentage, m.speed_limit = True, 100.0 / 3.0
    lp.publish(m)
    h = goal()
    t4 = time.monotonic()
    spin(3.0)
    low = window(nav, t4 + 1.0, time.monotonic())
    check('33.3 % -> RPP 0.10 (min_speed floor does not lift it)', bool(low) and abs(max(low) - 0.10) < 1e-3,
          f'max={max(low) if low else None}')
    h.cancel_goal_async()
    m.speed_limit = 0.0
    lp.publish(m)
    spin(1.0)
finally:
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT) if p is not procs[-1] else p.send_signal(signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
    time.sleep(2.0)   # teardown only (not a test wait)
    for p in procs:
        if p.poll() is None:
            p.kill()
    n.destroy_node()
    rclpy.try_shutdown()
print(f'SUMMARY {sum(results)}/{len(results)} PASS', flush=True)
sys.exit(0 if results and all(results) else 1)
