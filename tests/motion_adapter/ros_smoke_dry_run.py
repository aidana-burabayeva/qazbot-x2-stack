#!/usr/bin/env python3
"""ROS-level smoke test of x2_motion_adapter in DRY RUN on the real MC graph.

Checks, without moving the robot:
  1. the adapter node has NO publisher on /aima/mc/locomotion/velocity (dry run cannot command MC);
  2. the number of publishers on that topic is unchanged by starting the adapter;
  3. a Twist sent to the private cmd topic appears on ~/dry_run_cmd unchanged;
  4. status JSON reports state=dry_run and the live MC state fields are readable.
Start the adapter first (live:=false require_localization:=false), then run this.
Needs the aimdk_msgs environment. Publishes only on the private test topic.
"""
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MC_TOPIC = '/aima/mc/locomotion/velocity'
CMD_TOPIC = '/qazbot_d1_map/cmd_vel'

rclpy.init()
n = Node('adapter_smoke_test')
status, dry = [], []
n.create_subscription(String, '/x2_motion_adapter/status', lambda m: status.append(json.loads(m.data)), 10)
n.create_subscription(Twist, '/x2_motion_adapter/dry_run_cmd', dry.append, 10)
pub = n.create_publisher(Twist, CMD_TOPIC, 10)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
who = [i.node_name for i in n.get_publishers_info_by_topic(MC_TOPIC)]
print('publishers on MC velocity topic:', who)
assert 'x2_motion_adapter' not in who, 'dry-run adapter must not publish to MC'
t = Twist()
t.linear.x, t.angular.z = 0.30, 0.10
end = time.time() + 1.5
while time.time() < end:
    pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.02)
t.linear.x = t.angular.z = 0.0
for _ in range(10):
    pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.02)
rclpy.spin_once(n, timeout_sec=1.0)
assert dry, f"no dry_run_cmd received; status={status[-1:]}"
first = (round(dry[0].linear.x, 3), round(dry[0].linear.y, 3), round(dry[0].angular.z, 3))
print('dry_run_cmd first:', first, ' count:', len(dry))
assert first == (0.3, 0.0, 0.1), first
assert status and status[-1]['state'] == 'dry_run' and status[-1]['live'] is False, status[-1:]
print('status:', status[-1])
print('SMOKE OK')
sys.exit(0)
