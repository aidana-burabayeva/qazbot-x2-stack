#!/usr/bin/env python3
"""Copy only the existing local LIO TF edge into an independent TF graph.

No new estimator, timestamp change, extrapolation, or global correction.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


class LocalTfRelay(Node):
    def __init__(self):
        super().__init__('local_tf_relay')
        source = self.declare_parameter('source_tf', '/lio/tf').value
        target = self.declare_parameter('target_tf', '/qazbot_d1_map/tf').value
        if source == target:
            raise ValueError('source_tf and target_tf must differ')
        self.pub = self.create_publisher(TFMessage, target, 100)
        self.sub = self.create_subscription(TFMessage, source, self.on_tf, qos_profile_sensor_data)

    def on_tf(self, msg):
        transforms = [t for t in msg.transforms
                      if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint']
        if transforms:
            self.pub.publish(TFMessage(transforms=transforms))


def main():
    rclpy.init()
    node = LocalTfRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
