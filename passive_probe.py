#!/usr/bin/env python3
"""Observe standard ROS messages and endpoint metadata; never publish or call services."""
import json
import time
from collections import defaultdict

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init()
node = rclpy.create_node('qazbot_passive_navigation_probe')
stats = defaultdict(lambda: {'count': 0})
edges = {}
subscriptions = []

def callback(topic, msg):
    entry = stats[topic]
    entry['count'] += 1
    if isinstance(msg, TFMessage):
        for tf in msg.transforms:
            key = tf.header.frame_id + ' -> ' + tf.child_frame_id
            edges[key] = {'stamp': tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9,
                          'translation': [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z],
                          'rotation': [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w],
                          'topic': topic}
        return
    entry['frame_id'] = msg.header.frame_id
    entry['age_s'] = node.get_clock().now().nanoseconds / 1e9 - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
    if isinstance(msg, Odometry):
        entry['child_frame_id'] = msg.child_frame_id
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        entry['pose'] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        entry['covariance_diagonal'] = [msg.pose.covariance[i] for i in (0, 7, 14, 21, 28, 35)]
    if isinstance(msg, PointCloud2):
        entry.update(width=msg.width, height=msg.height, fields=[f.name for f in msg.fields])
    if isinstance(msg, OccupancyGrid):
        p = msg.info.origin.position
        entry.update(width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution, origin=[p.x, p.y, p.z])

topics = {'/tf': TFMessage, '/tf_static': TFMessage, '/map': OccupancyGrid,
          '/odom': Odometry, '/aima/mc/leg_odometry': Odometry,
          '/slam/lidar_odom': Odometry, '/slam/localization/odometry': Odometry,
          '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': PointCloud2,
          '/scan': LaserScan}
for topic, cls in topics.items():
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    if topic in ('/tf_static', '/map'):
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscriptions.append(node.create_subscription(cls, topic, lambda msg, t=topic: callback(t, msg), qos))
start = time.monotonic()
while time.monotonic() - start < 12:
    rclpy.spin_once(node, timeout_sec=0.1)
endpoints = {}
for topic in topics:
    endpoints[topic] = [{'node': i.node_namespace + '/' + i.node_name, 'type': i.topic_type,
                         'reliability': str(i.qos_profile.reliability), 'durability': str(i.qos_profile.durability)}
                        for i in node.get_publishers_info_by_topic(topic)]
print(json.dumps({'duration_s': time.monotonic()-start, 'topics': dict(stats), 'tf_edges': edges,
                  'publishers': endpoints, 'nodes': node.get_node_names_and_namespaces()}, indent=2))
node.destroy_node()
rclpy.shutdown()
