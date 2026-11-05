#!/usr/bin/env python3
"""Relays the replayed LiDAR cloud and TF, dropping them in configured sim-time windows (fault injection).

Windows are relative to the first message the gate sees. Relaying keeps the bag's QoS semantics
(SensorDataQoS for the cloud, TF default QoS)."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def _windows(flat):
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2) if flat[i + 1] > flat[i]]


class TopicGate(Node):

    def __init__(self):
        super().__init__('cm_topic_gate')
        for name, default in (('cloud_in', '/cm_test/lidar_pointcloud_raw'),
                              ('cloud_out', '/cm_test/lidar_pointcloud'),
                              ('tf_in', '/cm_test/tf_raw'), ('tf_out', '/cm_test/tf'),
                              ('tf_drop_child', 'base_footprint')):
            self.declare_parameter(name, default)
        self.declare_parameter('cloud_drop', [0.0, 0.0])
        self.declare_parameter('tf_drop', [0.0, 0.0])
        gp = self.get_parameter
        self._cloud_w = _windows(list(gp('cloud_drop').value))
        self._tf_w = _windows(list(gp('tf_drop').value))
        self._tf_child = gp('tf_drop_child').value
        self._t0 = None
        self._state = {'cloud': None, 'tf': None}
        self._cloud_pub = self.create_publisher(PointCloud2, gp('cloud_out').value, qos_profile_sensor_data)
        self._tf_pub = self.create_publisher(TFMessage, gp('tf_out').value, QoSProfile(depth=100))
        self.create_subscription(PointCloud2, gp('cloud_in').value, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(TFMessage, gp('tf_in').value, self._on_tf, QoSProfile(depth=100))
        self.get_logger().info(f'cloud drop windows {self._cloud_w}, tf drop windows {self._tf_w} '
                               f'(child "{self._tf_child}")')

    def _rel(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._t0 = self._t0 or now
        return now - self._t0

    def _dropping(self, key, windows):
        t = self._rel()
        drop = any(a <= t < b for a, b in windows)
        if drop != self._state[key]:
            self._state[key] = drop
            self.get_logger().info(f'{key}: {"DROPPING" if drop else "relaying"} at t={t:.2f} s')
        return drop

    def _on_cloud(self, msg):
        if not self._dropping('cloud', self._cloud_w):
            self._cloud_pub.publish(msg)

    def _on_tf(self, msg):
        if self._dropping('tf', self._tf_w):
            kept = [t for t in msg.transforms if self._tf_child and t.child_frame_id != self._tf_child]
            if not kept:
                return
            msg.transforms = kept
        self._tf_pub.publish(msg)


def main():
    rclpy.init()
    node = TopicGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
