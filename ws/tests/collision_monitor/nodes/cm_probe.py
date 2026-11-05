#!/usr/bin/env python3
"""Records cloud / cmd_vel_in / cmd_vel_out arrivals (sim + wall time) to CSV for offline analysis.

The cloud is subscribed raw (serialized CDR) and only its header stamp is decoded, so the probe adds
negligible load. Analysis lives in tools/analyze_run.py (logic in lib/cm_eval_lib.py)."""
import csv
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def cdr_header_stamp_ns(raw):
    """CDR (little endian, 4-byte encapsulation header) -> header.stamp of a message starting with Header."""
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec * 1_000_000_000 + nsec


class Probe(Node):

    def __init__(self):
        super().__init__('cm_probe')
        for name, default in (('cloud_topic', '/cm_test/lidar_pointcloud'),
                              ('cmd_in_topic', '/cm_test/cmd_vel_in'),
                              ('cmd_out_topic', '/cm_test/cmd_vel_out'),
                              ('output_csv', '/tmp/cm_probe.csv')):
            self.declare_parameter(name, default)
        gp = self.get_parameter
        self._rows = []
        self._path = gp('output_csv').value
        self.create_subscription(PointCloud2, gp('cloud_topic').value, self._on_cloud,
                                 qos_profile_sensor_data, raw=True)
        self.create_subscription(Twist, gp('cmd_in_topic').value, lambda m: self._on_cmd('in', m), 10)
        self.create_subscription(Twist, gp('cmd_out_topic').value, lambda m: self._on_cmd('out', m), 10)
        self.create_timer(2.0, self.flush)
        self.get_logger().info(f'recording to {self._path}')

    def _now(self):
        return time.monotonic_ns(), self.get_clock().now().nanoseconds

    def _on_cloud(self, raw):
        w, s = self._now()
        self._rows.append((w, s, 'cloud', cdr_header_stamp_ns(raw), len(raw), 0.0, 0.0, 0.0))

    def _on_cmd(self, kind, m):
        w, s = self._now()
        self._rows.append((w, s, kind, 0, 0, m.linear.x, m.linear.y, m.angular.z))

    def flush(self):
        with open(self._path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(('wall_ns', 'sim_ns', 'kind', 'stamp_ns', 'nbytes', 'vx', 'vy', 'wz'))
            wr.writerows(self._rows)


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.flush()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
