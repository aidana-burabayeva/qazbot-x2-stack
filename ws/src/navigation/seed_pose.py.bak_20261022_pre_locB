#!/usr/bin/env python3
"""Publish one operator-provided commissioning hypothesis to the private AMCL."""
import argparse
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

p = argparse.ArgumentParser()
for name in ['x', 'y', 'yaw-deg', 'std-x', 'std-y', 'std-yaw-deg']:
    p.add_argument('--' + name, required=True, type=float)
a = p.parse_args()
if not all(math.isfinite(v) for v in vars(a).values()):
    p.error('All values must be finite')
if min(a.std_x, a.std_y, a.std_yaw_deg) <= 0:
    p.error('Standard deviations must be positive')
rclpy.init()
n = rclpy.create_node('qazbot_operator_initial_pose')
pub = n.create_publisher(PoseWithCovarianceStamped, '/qazbot_d1_map/initialpose', 1)
latest_local_stamp = None
def local_tf(msg):
    global latest_local_stamp
    for tf in msg.transforms:
        if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
            latest_local_stamp = tf.header.stamp
sub = n.create_subscription(TFMessage, '/qazbot_d1_map/tf', local_tf, qos_profile_sensor_data)
try:
    # Bounded diagnostic discovery, not a motion timeout or retry policy.
    deadline = time.monotonic() + 10
    while pub.get_subscription_count() == 0 or latest_local_stamp is None:
        if time.monotonic() >= deadline:
            raise RuntimeError('AMCL subscriber or local odometry TF unavailable; nothing published')
        rclpy.spin_once(n, timeout_sec=0.1)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    # Commissioning requires a stationary robot. Reference the observed local
    # odometry sample. AMCL may still request callback-time TF internally;
    # this stamp alone does not eliminate its future-extrapolation warning.
    m.header.stamp = latest_local_stamp
    m.pose.pose.position.x, m.pose.pose.position.y = a.x, a.y
    yaw = math.radians(a.yaw_deg)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    for index, value in [(0,a.std_x), (7,a.std_y), (35,math.radians(a.std_yaw_deg))]:
        m.pose.covariance[index] = value * value
    pub.publish(m)
    if not pub.wait_for_all_acked(rclpy.duration.Duration(seconds=5)):
        raise RuntimeError('Initialpose transport acknowledgement not received; do not repeat automatically')
    print(json.dumps({'status': 'one hypothesis published; localization not accepted', **vars(a)}))
finally:
    n.destroy_node()
    rclpy.shutdown()
