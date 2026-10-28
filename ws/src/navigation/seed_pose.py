#!/usr/bin/env python3
"""Publish one operator-provided commissioning hypothesis to the private AMCL.

--from-file PATH --map-yaml YAML: re-seed the last CONFIRMED pose saved by the trust node (pose_store) ONCE, with its
saved covariance, if the file exists and its map sha256 equals the image of YAML; otherwise publish nothing and exit 0
(the operator poses as before). It is only a hypothesis: the trust node clears READY on every initialpose, and READY
needs confirm_anchor + a clean gate (T2/T3/T4 ...) on the seeded pose."""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pose_store  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--from-file', default='')
p.add_argument('--map-yaml', default='')
for name in ['x', 'y', 'yaw-deg', 'std-x', 'std-y', 'std-yaw-deg']:
    p.add_argument('--' + name, type=float)
a = p.parse_args()
manual = {k: v for k, v in vars(a).items() if k not in ('from_file', 'map_yaml')}
if a.from_file:
    if not a.map_yaml:
        p.error('--from-file needs --map-yaml')
    if any(v is not None for v in manual.values()):
        p.error('--from-file excludes --x/--y/--yaw-deg/--std-*')
    record, why = pose_store.load(a.from_file, pose_store.map_image_sha256(a.map_yaml))
    if record is None:
        print(json.dumps({'status': 'nothing seeded; operator initial pose required', 'reason': why, 'file': a.from_file}))
        sys.exit(0)
    x, y, yaw, cov = record['x'], record['y'], record['yaw'], record['covariance']
else:
    if any(v is None for v in manual.values()):
        p.error('--x --y --yaw-deg --std-x --std-y --std-yaw-deg are required (or --from-file)')
    if not all(math.isfinite(v) for v in manual.values()):
        p.error('All values must be finite')
    if min(a.std_x, a.std_y, a.std_yaw_deg) <= 0:
        p.error('Standard deviations must be positive')
    x, y, yaw, cov = a.x, a.y, math.radians(a.yaw_deg), [0.0] * 36
    for index, value in [(0, a.std_x), (7, a.std_y), (35, math.radians(a.std_yaw_deg))]:
        cov[index] = value * value

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

TOPIC = '/qazbot_d1_map/initialpose'
rclpy.init()
n = rclpy.create_node('qazbot_operator_initial_pose')
pub = n.create_publisher(PoseWithCovarianceStamped, TOPIC, 1)
latest_local_stamp = None
def local_tf(msg):
    global latest_local_stamp
    for tf in msg.transforms:
        if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
            latest_local_stamp = tf.header.stamp
sub = n.create_subscription(TFMessage, '/qazbot_d1_map/tf', local_tf, qos_profile_sensor_data)


def amcl_matched():
    """AMCL's initialpose subscription is in the graph and every discovered subscriber is matched to our publisher.
    (The trust node also subscribes; a subscriber count alone could publish before AMCL exists.)"""
    subs = n.get_subscriptions_info_by_topic(TOPIC)
    return (any(s.node_name == 'amcl' and s.node_namespace == '/qazbot_d1_map' for s in subs)
            and pub.get_subscription_count() >= len(subs))


try:
    if a.from_file:
        # Relaunch: AMCL and FAST-LIO come up with this launch, so wait on the graph (no deadline); launch shutdown
        # (SIGINT) ends the wait. AMCL keeps an initialpose received while inactive and applies it on activation.
        n.get_logger().info('waiting for AMCL initialpose subscriber and odom->base_footprint TF')
        try:
            while not amcl_matched() or latest_local_stamp is None:
                rclpy.spin_once(n, timeout_sec=0.1)
        except (KeyboardInterrupt, ExternalShutdownException):
            print(json.dumps({'status': 'nothing seeded; shut down while waiting for AMCL', 'file': a.from_file}))
            sys.exit(0)
    else:
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
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    m.pose.covariance = [float(v) for v in cov]
    pub.publish(m)
    if not pub.wait_for_all_acked(rclpy.duration.Duration(seconds=5)):
        raise RuntimeError('Initialpose transport acknowledgement not received; do not repeat automatically')
    extra = dict(file=a.from_file, x=x, y=y, yaw=yaw, covariance=list(cov),
                 map_sha256=record['map_sha256'], saved_unix=record['saved_unix']) if a.from_file else manual
    print(json.dumps({'status': 'one hypothesis published; localization not accepted', **extra}))
finally:
    n.destroy_node()
    rclpy.try_shutdown()
