#!/usr/bin/env python3
"""SIM only. Read /aima/hal/odom/state for SECONDS (default 1) and print: n z_mean z_min (0 msgs -> n=0)."""
import os, sys, time
if os.environ.get('QAZBOT_SIM') != '1':
    raise RuntimeError('SIM only')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('sim_odom_peek'); z = []
n.create_subscription(Odometry, '/aima/hal/odom/state', lambda m: z.append(m.pose.pose.position.z), qos_profile_sensor_data)
end = time.monotonic() + (float(sys.argv[1]) if len(sys.argv) > 1 else 1.0)
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.01)
print('n=%d z_mean=%.3f z_min=%.3f' % (len(z), sum(z) / len(z) if z else -1, min(z) if z else -1))
