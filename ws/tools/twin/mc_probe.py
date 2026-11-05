#!/usr/bin/env python3
"""Record the official sim's odometry sources while a velocity command runs (SIM container only).
Usage: mc_probe.py OUT.csv SECONDS. Columns: t, source, x, y, yaw, vx, wz (wz = twist.angular.z as published)."""
import csv, math, os, sys, time
if os.environ.get('QAZBOT_SIM') != '1':
    raise RuntimeError('SIM only')
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

rclpy.init(); n = Node('mc_probe'); t0 = time.monotonic(); rows = []
def cb(src):
    def f(m):
        q = m.pose.pose.orientation
        rows.append((round(time.monotonic() - t0, 4), src, m.pose.pose.position.x, m.pose.pose.position.y,
                     2 * math.atan2(q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))
    return f
n.create_subscription(Odometry, '/aima/mc/leg_odometry', cb('leg_odometry'), qos_profile_sensor_data)
n.create_subscription(Odometry, '/aima/hal/odom/state', cb('hal_odom'), qos_profile_sensor_data)
end = time.monotonic() + float(sys.argv[2])
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=.01)
with open(sys.argv[1], 'w', newline='') as f:
    csv.writer(f).writerows([('t', 'source', 'x', 'y', 'yaw', 'vx', 'wz')] + rows)
print({s: sum(1 for r in rows if r[1] == s) for s in ('leg_odometry', 'hal_odom')})
