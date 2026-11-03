#!/usr/bin/env python3
"""Publish ONE goal_pose to the goal coordinator after the subscription is matched. argv: x y yaw_rad"""
import sys, math, time, rclpy
from geometry_msgs.msg import PoseStamped
x, y, yw = map(float, sys.argv[1:4])
rclpy.init(); n = rclpy.create_node('send_goal_1102'); p = n.create_publisher(PoseStamped, '/qazbot_d1_map/goal_pose', 1)
end = time.time() + 15
while p.get_subscription_count() < 1 and time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
if p.get_subscription_count() < 1: sys.exit('no subscriber; nothing sent')
t = time.time() + 1.0
while time.time() < t: rclpy.spin_once(n, timeout_sec=0.1)
m = PoseStamped(); m.header.frame_id = 'map'; m.header.stamp = n.get_clock().now().to_msg()
m.pose.position.x, m.pose.position.y = x, y; m.pose.orientation.z, m.pose.orientation.w = math.sin(yw / 2), math.cos(yw / 2)
p.publish(m); t = time.time() + 1.0
while time.time() < t: rclpy.spin_once(n, timeout_sec=0.1)
print('sent', x, y, yw, 'subs', p.get_subscription_count(), time.strftime('%H:%M:%S'))
