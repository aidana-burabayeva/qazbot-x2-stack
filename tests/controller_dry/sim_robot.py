#!/usr/bin/env python3
"""Kinematic stand-in for MC + sensors, for dry controller tests (no hardware, own namespace, own TF).

Reads <ns>/cmd_vel (Twist), applies MC-like behaviour measured/documented for X2:
  * command older than 0.2 s counts as zero (docs 1.1.0),
  * a start from rest needs |vx| >= START_VX (~0.1 m/s) or |wz| >= START_WZ (~0.05 rad/s) (approximate, docs),
  * first-order velocity lag TAU.
Publishes <ns>/odom, TF map->odom (identity) and odom->base_footprint to <ns>/tf, and a LaserScan
(<ns>/scan) from one straight wall x = wall_x, |y| <= 3 (wall_x < 0 disables it).
<ns>/sim_reset (Float64MultiArray [x, y, yaw, wall_x(, wall_half_length)]) repositions the robot.
Test hooks (default = previous behaviour): map_odom (x, y, yaw) published as map->odom (emulated
localization offset), publish_map_odom (False = localization source dead), wall_half (default 3 m).
Usage: sim_robot.py NS
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

START_VX, START_WZ, TAU, CMD_TIMEOUT = 0.1, 0.05, 0.25, 0.2


class Sim(Node):
    def __init__(self, ns):
        super().__init__('sim_robot', namespace=ns)
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.wall_x = -1.0
        self.wall_y = 0.0
        self.wall_half = 3.0
        self.map_odom = (0.0, 0.0, 0.0)
        self.publish_map_odom = True
        self.cmd = (0.0, 0.0)
        self.cmd_t = 0.0
        self.moving = False
        self.tau, self.start_vx, self.start_wz = TAU, START_VX, START_WZ
        self.tf_pub = self.create_publisher(TFMessage, f'/{ns}/tf', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 5)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, 'sim_reset', self.on_reset, 10)
        self.last = time.monotonic()
        self.create_timer(0.02, self.step)
        self.scan_timer = self.create_timer(0.1, self.pub_scan)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.monotonic()

    def on_reset(self, m):
        self.x, self.y, self.yaw, self.wall_x = m.data[:4]
        self.wall_half = m.data[4] if len(m.data) > 4 else 3.0
        self.v = self.w = 0.0
        self.moving = False
        self.cmd = (0.0, 0.0)

    def step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        cv, cw = self.cmd if now - self.cmd_t <= CMD_TIMEOUT else (0.0, 0.0)
        if not self.moving and abs(cv) < self.start_vx and abs(cw) < self.start_wz:
            cv = cw = 0.0
        self.moving = abs(cv) > 1e-6 or abs(cw) > 1e-6 or abs(self.v) > 0.01 or abs(self.w) > 0.01
        a = 1.0 if self.tau <= 0 else 1.0 - math.exp(-dt / self.tau)
        self.v += (cv - self.v) * a
        self.w += (cw - self.w) * a
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        stamp = self.get_clock().now().to_msg()
        tfs = []
        frames = [('odom', 'base_footprint', self.x, self.y, self.yaw)]
        if self.publish_map_odom:
            frames.insert(0, ('map', 'odom') + tuple(self.map_odom))
        for parent, child, x, y, yaw in frames:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            tfs.append(t)
        self.tf_pub.publish(TFMessage(transforms=tfs))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)

    def pub_scan(self):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        n = 360
        s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 2 * math.pi / n
        s.range_min, s.range_max = 0.05, 8.0
        ranges = []
        for i in range(n):
            a = self.yaw + s.angle_min + i * s.angle_increment
            r = float('inf')
            if self.wall_x >= 0 and math.cos(a) > 1e-6:
                t = (self.wall_x - self.x) / math.cos(a)
                if t > 0 and abs(self.y - self.wall_y + t * math.sin(a)) <= self.wall_half:
                    r = t
            ranges.append(r if r <= s.range_max else float('inf'))
        s.ranges = ranges
        self.scan_pub.publish(s)


def main():
    rclpy.init()
    rclpy.spin(Sim(sys.argv[1]))


if __name__ == '__main__':
    main()
