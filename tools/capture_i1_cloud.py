#!/usr/bin/env python3
"""Passive evidence capture; does not start sensors or publish commands.

Run only after operator approval and the single-owner process/topic check.
Stops on a quota of unique, nonempty clouds with same-stamp LIO odometry,
not on a runtime timeout. Output must be outside the Git checkout.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--matched-clouds', type=int, default=100)
    args = parser.parse_args()
    if args.matched_clouds < 1 or args.output.exists():
        parser.error('quota must be positive; output must not exist')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rclpy.signals import SignalHandlerOptions
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('i1_passive_cloud_capture')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    counts = collections.Counter()
    odom_stamps, cloud_stamps = set(), set()
    ages = []
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def receive(topic, message):
        now = time.time_ns()
        writer.write(topic, serialize_message(message), now)
        counts[topic] += 1
        if topic in ('/qazbot_lio/cloud_registered', '/qazbot_lio/odometry'):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if topic.endswith('/odometry'):
                odom_stamps.add(stamp)
            elif message.width * message.height:
                cloud_stamps.add(stamp)
                ages.append((now - stamp) * 1e-9)

    topics = [
        ('/qazbot_lio/cloud_registered', PointCloud2, 'sensor_msgs/msg/PointCloud2'),
        ('/qazbot_lio/odometry', Odometry, 'nav_msgs/msg/Odometry'),
        ('/aima/hal/sensor/lidar_chest_front/imu', Imu, 'sensor_msgs/msg/Imu'),
        ('/aima/mc/leg_odometry', Odometry, 'nav_msgs/msg/Odometry'),
        ('/tf_static', TFMessage, 'tf2_msgs/msg/TFMessage'),
    ]
    for topic, cls, type_name in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name,
                                                   serialization_format='cdr'))
        static = topic == '/tf_static'
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE)
        node.create_subscription(cls, topic, lambda m, t=topic: receive(t, m), qos)
    print(json.dumps({'event': 'capture_started', 'path': str(args.output),
                      'quota': args.matched_clouds}), flush=True)
    try:
        while not interrupted and len(odom_stamps & cloud_stamps) < args.matched_clouds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Close storage before potentially slow DDS teardown.
        del writer
        matched = len(odom_stamps & cloud_stamps)
        ordered = sorted(ages)
        summary = {'counts': dict(counts), 'matched_clouds': matched,
                   'quota': args.matched_clouds, 'interrupted': interrupted,
                   'quota_met': matched >= args.matched_clouds,
                   'cloud_receipt_age_s': {'max': max(ordered) if ordered else None,
                       'p95': ordered[int(.95 * (len(ordered)-1))] if ordered else None},
                   'files_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in args.output.iterdir() if p.is_file()}}
        (args.output / 'capture_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if matched >= args.matched_clouds else 2


if __name__ == '__main__':
    raise SystemExit(main())
