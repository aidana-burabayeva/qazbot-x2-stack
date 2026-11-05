"""Vendor footprint radius about the navigation point (base_footprint).

Inputs are copied from the robot (2026-10-21): app2.yaml robot.footprint (12 vertices, base_link frame),
TF base_link -> lidar_imu_chest_front translation, and the lidar-forward heading offset computed from
that TF rotation and level_bridge.LIDAR_FWD_IN_IMU (0.22 deg, axes parallel).
"""
import math
import unittest

import numpy as np

POLY = np.array([[0.20, 0.0], [0.20, 0.225], [0.075, 0.336], [0.0, 0.336], [-0.075, 0.336], [-0.20, 0.225],
                 [-0.20, 0.0], [-0.20, -0.225], [-0.075, -0.336], [0.0, -0.336], [0.075, -0.336], [0.20, -0.225]])
RECT = np.array([[sx * 0.20, sy * 0.336] for sx in (1, -1) for sy in (1, -1)])
NAV_POINT = np.array([0.0866, 0.0161])      # live TF 2026-10-21; static value 0.0867, 0.0173
HEADING_OFFSET = math.radians(0.22)


def radius(points, nav=NAV_POINT, yaw=HEADING_OFFSET):
    c, s = math.cos(-yaw), math.sin(-yaw)
    rel = (points - nav) @ np.array([[c, -s], [s, c]]).T
    return float(np.hypot(rel[:, 0], rel[:, 1]).max())


class VendorPolygon(unittest.TestCase):
    def test_polygon_about_nav_point(self):
        self.assertAlmostEqual(radius(POLY), 0.387, delta=0.003)

    def test_bounding_rectangle_about_nav_point(self):
        # circumscribed rectangle is NOT the vendor shape; its radius is the conservative bound
        self.assertAlmostEqual(radius(RECT), 0.454, delta=0.003)

    def test_vendor_circle_about_base_link_is_not_nav_point_radius(self):
        self.assertAlmostEqual(float(np.hypot(*RECT.T).max()), 0.391, delta=0.001)
        self.assertGreater(radius(RECT), 0.391 + 0.05)

    def test_heading_offset_negligible(self):
        self.assertLess(abs(radius(RECT, yaw=0.0) - radius(RECT)), 0.002)


if __name__ == "__main__":
    unittest.main()
