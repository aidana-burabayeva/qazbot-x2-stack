#!/usr/bin/env python3
"""Unit tests of level_bridge.scan_ranges (bin logic of /scan) and of the staged nav files.
Task 20261027-rpp-local-collision (I1). No robot, no DDS traffic.

Run (candidate): LB_SRC=<dir with level_bridge.py> NAV_SRC=<navigation dir> python3 -m unittest -v test_level_bridge_scan.py
Default dirs: ws/src/{lio,navigation} relative to the control_backend root two levels above this file.
"""
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LB = Path(os.environ.get('LB_SRC', ROOT / 'ws' / 'src' / 'lio'))
NAV = Path(os.environ.get('NAV_SRC', ROOT / 'ws' / 'src' / 'navigation'))
sys.path.insert(0, str(LB))
import level_bridge as lb  # noqa: E402

N = lb.N_BINS


def bin_of(deg):
    return int((math.radians(deg) + math.pi) / (2 * math.pi) * N) % N


def scan(hits, **kw):
    """hits: list of (bearing deg, range m)."""
    a = np.radians([h[0] for h in hits]) if hits else np.array([])
    r = np.array([h[1] for h in hits], dtype=float)
    return lb.scan_ranges(a, r, **kw)


class ScanRanges(unittest.TestCase):
    def test_nan_outside_mask(self):
        s = scan([])
        for deg in (-180, -120, -60, -46.5, 45, 90, 179):
            self.assertTrue(np.isnan(s[bin_of(deg)]), deg)

    def test_inf_for_in_fov_no_return(self):
        s = scan([])
        for deg in (-45.5, -20, 0, 20, 44):
            self.assertTrue(np.isposinf(s[bin_of(deg)]), deg)

    def test_fov_edges_are_parameters(self):
        s = scan([], fov_min_deg=-10, fov_max_deg=10)
        self.assertTrue(np.isposinf(s[bin_of(0)]))
        self.assertTrue(np.isnan(s[bin_of(-20)]))
        self.assertTrue(np.isnan(s[bin_of(20)]))

    def test_real_hit_preserved_min_range(self):
        s = scan([(10.0, 2.0), (10.0, 1.2), (10.0, 3.0)])
        self.assertAlmostEqual(float(s[bin_of(10.0)]), 1.2, places=5)

    def test_hit_outside_fov_kept(self):
        s = scan([(120.0, 1.5)])
        self.assertAlmostEqual(float(s[bin_of(120.0)]), 1.5, places=5)

    def test_guard_around_hit(self):
        b = bin_of(0.0)
        s = scan([(0.0, 1.0)], guard_bins=2)
        self.assertTrue(np.isfinite(s[b]))
        for k in (-2, -1, 1, 2):
            self.assertTrue(np.isnan(s[b + k]), k)
        for k in (-3, 3):
            self.assertTrue(np.isposinf(s[b + k]), k)

    def test_guard_zero_disables(self):
        b = bin_of(0.0)
        s = scan([(0.0, 1.0)], guard_bins=0)
        self.assertTrue(np.isposinf(s[b + 1]))

    def test_guard_does_not_touch_finite_neighbours(self):
        b = bin_of(0.0)
        s = scan([(0.0, 1.0), (0.0 + 0.25 * 1.0, 2.0)], guard_bins=2)
        self.assertTrue(np.isfinite(s[b]) and np.isfinite(s[b + 1]))

    def test_fmin_nan_does_not_poison_hit(self):
        # outside FOV the bin starts as NaN; np.minimum.at would keep NaN, fmin must take the hit
        s = scan([(-120.0, 2.5)])
        self.assertAlmostEqual(float(s[bin_of(-120.0)]), 2.5, places=5)
        r = np.full(4, np.nan, np.float32)
        np.fmin.at(r, [1, 1, 2], np.array([3.0, 2.0, 5.0], np.float32))
        self.assertEqual(float(r[1]), 2.0)
        self.assertTrue(np.isnan(r[0]))
        m = np.full(2, np.nan, np.float32)
        np.minimum.at(m, [0], np.array([1.0], np.float32))
        self.assertTrue(np.isnan(m[0]))   # documents why minimum is wrong

    def test_guard_wraps_at_seam(self):
        s = scan([(179.9, 1.0)], guard_bins=2)
        self.assertTrue(np.isfinite(s[bin_of(179.9)]))

    def test_dtype_and_len(self):
        s = scan([(0.0, 1.0)])
        self.assertEqual(s.shape, (N,))
        self.assertEqual(s.dtype, np.float32)

    def test_empty_slice_in_fov_all_inf_no_hits(self):
        s = scan([])
        self.assertEqual(int(np.isfinite(s).sum()), 0)
        self.assertGreater(int(np.isposinf(s).sum()), 300)


class StagedNav(unittest.TestCase):
    def test_bt_parses_and_has_recovery(self):
        t = ET.parse(NAV / 'bt' / 'navigate_to_pose.xml').getroot()
        rn = t.find('.//RecoveryNode')
        self.assertIsNotNone(rn)
        self.assertIn(int(rn.get('number_of_retries')), (1, 2))
        kids = [c.tag for c in rn]
        self.assertEqual(kids, ['FollowPath', 'ClearEntireCostmap'])
        self.assertEqual(rn.find('ClearEntireCostmap').get('service_name'),
                         'local_costmap/clear_entirely_local_costmap')
        self.assertIsNotNone(t.find('.//ComputePathToPose'))
        # planner is outside the recovery loop
        self.assertIsNone(rn.find('.//ComputePathToPose'))

    def test_launch_files_compile_and_mention_params(self):
        c = (NAV / 'nav_control.launch.py').read_text()
        compile(c, 'nav_control.launch.py', 'exec')
        self.assertIn("'obstacle_layer.scan.inf_is_valid': True", c)
        for k in ("'obstacle_layer.scan.obstacle_max_range': 3.0", "'obstacle_layer.scan.raytrace_max_range': 3.5",
                  "'FollowPath.use_collision_detection': True", "'failure_tolerance': 0.3",
                  "'footprint_padding': 0.01"):
            self.assertIn(k, c)
        f = (NAV / 'nav_full.launch.py').read_text()
        compile(f, 'nav_full.launch.py', 'exec')
        self.assertIn('nav2_recovery_node_bt_node', f)
        self.assertIn('nav2_clear_costmap_service_bt_node', f)


if __name__ == '__main__':
    unittest.main()
