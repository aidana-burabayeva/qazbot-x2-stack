#!/usr/bin/env python3
"""Live operator aid for P-D04: prints the nearest /qazbot_lio/scan return per bearing sector once per second."""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# (label, min_deg, max_deg); bearing in base_footprint, +left
SECTORS = [('R-edge', -62, -44), ('R', -44, -15), ('F', -15, 15), ('L', 15, 44), ('L-edge', 44, 62), ('L-side', 62, 90)]


class ScanSectors(Node):
    def __init__(self, max_range):
        super().__init__('d04_scan_sectors')
        self.max_range = max_range
        self.last = None
        self.create_subscription(LaserScan, '/qazbot_lio/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_scan(self, msg):
        self.last = msg

    def report(self):
        m = self.last
        if m is None:
            print('no scan yet', flush=True)
            return
        out = []
        for label, lo, hi in SECTORS:
            best = None
            for i, r in enumerate(m.ranges):
                a = math.degrees(m.angle_min + i * m.angle_increment)
                if lo <= a < hi and math.isfinite(r) and m.range_min < r < self.max_range:
                    if best is None or r < best[0]:
                        best = (r, a)
            out.append(f'{label}:{best[0]:.2f}m@{best[1]:+.0f}' if best else f'{label}:--')
        print(' '.join(out), flush=True)


def main():
    rclpy.init()
    node = ScanSectors(float(sys.argv[1]) if len(sys.argv) > 1 else 2.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
