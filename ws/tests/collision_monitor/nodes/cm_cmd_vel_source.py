#!/usr/bin/env python3
"""Synthetic constant cmd_vel_in for the offline collision_monitor test (communication only)."""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelSource(Node):

    def __init__(self):
        super().__init__('cm_cmd_vel_source')
        self.declare_parameter('topic', '/cm_test/cmd_vel_in')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('linear_x', 0.30)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('stop_after_s', -1.0)
        gp = self.get_parameter
        self._msg = Twist()
        self._msg.linear.x = gp('linear_x').value
        self._msg.angular.z = gp('angular_z').value
        self._stop_after = gp('stop_after_s').value
        self._t0 = None
        self._stopped = False
        self._pub = self.create_publisher(Twist, gp('topic').value, 1)
        self.create_timer(1.0 / gp('rate_hz').value, self._tick)
        self.get_logger().info(
            f'constant cmd_vel_in vx={self._msg.linear.x} wz={self._msg.angular.z} '
            f'at {gp("rate_hz").value} Hz, stop_after_s={self._stop_after}')

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return  # no /clock yet
        self._t0 = self._t0 or now
        if self._stop_after > 0.0 and now - self._t0 > self._stop_after:
            if not self._stopped:
                self._stopped = True
                self.get_logger().info('cmd_vel_in publishing stopped (scenario)')
            return
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = CmdVelSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
