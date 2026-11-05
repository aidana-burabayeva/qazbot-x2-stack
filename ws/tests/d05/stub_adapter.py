#!/usr/bin/env python3
"""TEST STUB (not the adapter): publishes a motion_adapter/status-like JSON and records Twists received on the
command topic. Used only in an isolated ROS_DOMAIN_ID for the D-05 dry-run integration. Refuses domain 0."""
import argparse
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from aimdk_msgs.msg import McLocomotionVelocity

ap = argparse.ArgumentParser()
ap.add_argument('--scenario', default='ok', choices=['ok', 'not_armed', 'rc_active', 'latch_mid', 'dry_adapter'])
ap.add_argument('--out', required=True, help='JSONL of received commands')
ap.add_argument('--latch-after', type=float, default=1.5, help='latch_mid: seconds after the first non-zero cmd')
ap.add_argument('--ns', default='/qazbot_d1_map')
a = ap.parse_args()
if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', ''):
    sys.exit('stub refuses to run in ROS_DOMAIN_ID 0')

rclpy.init()
n = Node('motion_adapter_stub')
st = {'first_nz': None, 'latched': False}
f = open(a.out, 'w')


def status():
    s = {'state': 'idle', 'reason': 'no_command', 'latch': '', 'live': True, 'mode': 'live', 'dry_run': False,
         'mc_source': 'rc', 'trusted': True, 'registration': 'registered', 'registration_error': '',
         'motion_armed': True, 'ready': True, 'foreign_publishers': [], 't1_pause': False}
    if a.scenario == 'not_armed':
        s['motion_armed'] = False
        s['reason'] = 'driver_not_armed'
    if a.scenario == 'dry_adapter':
        s.update(live=False, mode='dry_run', dry_run=True, registration='dry_run')
    if st['first_nz'] is not None:
        s.update(state='active', reason='driving', mc_source='sdk_stub')
    if st['latched']:
        s.update(state='latched', latch='operator_takeover', reason='operator_takeover', mc_source='rc')
    return s


def on_cmd(m):
    t = time.monotonic()
    f.write(json.dumps({'t': t, 'vx': m.linear.x, 'vy': m.linear.y, 'wz': m.angular.z}) + '\n')
    f.flush()
    if abs(m.linear.x) > 1e-6 and st['first_nz'] is None:
        st['first_nz'] = t


def tick():
    if a.scenario == 'latch_mid' and st['first_nz'] and time.monotonic() - st['first_nz'] > a.latch_after:
        st['latched'] = True
    pub_status.publish(String(data=json.dumps(status())))


pub_status = n.create_publisher(String, f'{a.ns}/motion_adapter/status', 10)
n.create_subscription(Twist, f'{a.ns}/cmd_vel_authorized', on_cmd, 10)
n.create_publisher(String, f'{a.ns}/goal_coordinator/status', 10).publish(
    String(data=json.dumps({'busy': False, 'latched': False})))
n.create_timer(0.5, tick)
rc_pub = n.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity',
                            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))


def rc_tick():
    m = McLocomotionVelocity()
    m.source = 'rc'
    m.forward_velocity = 0.5 if a.scenario == 'rc_active' else 0.0     # real rc streams zeros at ~49 Hz when idle
    rc_pub.publish(m)


n.create_timer(0.02, rc_tick)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
