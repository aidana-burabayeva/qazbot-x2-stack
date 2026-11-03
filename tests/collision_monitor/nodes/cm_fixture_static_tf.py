#!/usr/bin/env python3
"""Publishes the test fixture static TF base_footprint -> lidar_chest_front from YAML parameters."""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class FixtureStaticTf(Node):

    def __init__(self):
        super().__init__('cm_fixture_static_tf')
        self.declare_parameter('parent_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'lidar_chest_front')
        self.declare_parameter('translation', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation_xyzw', [0.0, 0.0, 0.0, 1.0])
        gp = self.get_parameter
        t = TransformStamped()
        t.header.frame_id = gp('parent_frame').value
        t.child_frame_id = gp('child_frame').value
        tr = gp('translation').value
        q = gp('rotation_xyzw').value
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = tr
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self._bc = StaticTransformBroadcaster(self)
        self._bc.sendTransform(t)
        self.get_logger().info(f'static TF {t.header.frame_id} -> {t.child_frame_id} published')


def main():
    rclpy.init()
    node = FixtureStaticTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
