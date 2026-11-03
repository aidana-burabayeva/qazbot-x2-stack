#!/usr/bin/env python3
"""Log wall time of receive gaps > 0.3 s on /qazbot_lio/scan and /qazbot_native/odom for N s (read-only)."""
import sys, time, rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('gap_probe_1102')
last = {}
def cb(k):
    def f(_):
        t = time.time()
        if k in last and t - last[k] > 0.3: print(f'{t:.2f} {k} gap {t-last[k]:.2f}', flush=True)
        last[k] = t
    return f
n.create_subscription(LaserScan, '/qazbot_lio/scan', cb('scan'), qos_profile_sensor_data)
n.create_subscription(Odometry, '/qazbot_native/odom', cb('odom'), 10)
end = time.time() + float(sys.argv[1])
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
