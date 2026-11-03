"""Admission-policy comparisons; no ROS, hardware, goals, or velocity publishers."""
import unittest
from dataclasses import replace
from test_loc_trust import L, room_grid, make_inputs, simulate_scan


class Go2Policy(unittest.TestCase):
    def setUp(self):
        self.grid = room_grid()
        self.ev = L.TrustEvaluator(self.grid, L.TrustConfig(admission_policy='go2'))
        self.inp = make_inputs(10., (0., 0., 0.), (0., 0., 0.),
                               simulate_scan(self.grid, (0., 0., 0.)))

    def test_requires_initialization_not_just_fresh_tf(self):
        self.assertFalse(self.ev.evaluate(self.inp)['trusted'])
        self.ev.relocalize()
        self.assertFalse(self.ev.evaluate(self.inp)['trusted'])
        self.ev.accept_initialization()
        self.assertTrue(self.ev.evaluate(self.inp)['trusted'])

    def test_rviz_covariance_does_not_require_convergence(self):
        self.ev.accept_initialization()
        cov = self.inp.amcl_cov.copy()
        cov[0] = cov[7] = .25
        cov[35] = .0685
        inp = replace(self.inp, amcl_cov=cov, cloud_xy=None, cloud_w=None)
        result = self.ev.evaluate(inp)
        self.assertTrue(result['trusted'])
        self.assertIn('T2_covariance', result['diagnostics'])
        strict = L.TrustEvaluator(self.grid)
        strict.confirm_anchor(inp)
        self.assertFalse(strict.evaluate(inp)['trusted'])

    def test_quality_and_custom_due_rules_are_diagnostic(self):
        self.ev.accept_initialization()
        flags = ['T2_covariance', 'T3_particles', 'T4_scan_score', 'T5_map_odom_jump',
                 'T6_no_anchor', 'T7_ambiguous', 'T1_amcl_pose_stale',
                 'T1_particle_cloud_stale', 'T1_tf_frozen']
        result = self.ev._verdict(flags, {})
        self.assertTrue(result['trusted'])
        self.assertEqual(result['diagnostics'], flags)

    def test_each_retained_fault_blocks_without_threshold_changes(self):
        for fault in ['T0_invalid_pose', 'T0_invalid_scan', 'T1_tf_stale',
                      'T1_scan_stale', 'T1_odom_stale', 'T1_odom_message_stale']:
            with self.subTest(fault=fault):
                self.ev.accept_initialization()
                result = self.ev._verdict([fault], {})
                self.assertFalse(result['trusted'])
                self.assertIn(fault, result['reasons'])
                self.ev.goal_request()
                self.assertTrue(self.ev._verdict([], {})['trusted'])

    def test_map_fault_requires_new_initialization(self):
        for fault in ['T8_map_mismatch', 'T8_map_unseen']:
            self.ev.accept_initialization()
            self.assertFalse(self.ev._verdict([fault], {})['trusted'])
            self.assertFalse(self.ev._verdict([], {})['ready'])

    def test_new_initialpose_revokes_previous_initialization(self):
        self.ev.accept_initialization()
        self.assertTrue(self.ev.evaluate(self.inp)['trusted'])
        self.ev.relocalize()
        self.assertFalse(self.ev.evaluate(self.inp)['trusted'])

    def test_real_stale_scan_and_odom_block(self):
        self.ev.accept_initialization()
        result = self.ev.evaluate(replace(self.inp, scan_stamp=8., odom_stamp=8.))
        self.assertFalse(result['trusted'])
        self.assertIn('T1_scan_stale', result['reasons'])
        self.assertIn('T1_odom_stale', result['reasons'])

    def test_policy_validation_and_default(self):
        self.assertEqual(L.TrustConfig().admission_policy, 'strict')
        with self.assertRaises(ValueError):
            L.TrustEvaluator(self.grid, L.TrustConfig(admission_policy='typo'))


class Go2Callbacks(unittest.TestCase):
    """Execute the actual callback bodies with message-shaped objects, without loading ROS."""
    def setUp(self):
        import ast
        import math
        import threading
        from pathlib import Path
        from types import SimpleNamespace as NS
        import numpy as np
        source = Path(L.__file__).with_name('localization_trust_node.py').read_text()
        tree = ast.parse(source)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TrustNode')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('_initialpose', '_amcl')]
        namespace = {'np': np, 'math': math}
        exec(compile(ast.Module(body=methods, type_ignores=[]), source, 'exec'), namespace)
        self.NS = NS
        self.node = NS(cfg=L.TrustConfig(admission_policy='go2'), seed_stamp=None,
                       ev=L.TrustEvaluator(room_grid(), L.TrustConfig(admission_policy='go2')),
                       lock=threading.Lock(), cov=None, mo=None, odom=(0., 0., 0.))
        self.node._stamp = lambda *args: 100.
        self.node._odom_at = lambda *args: None
        self.node.get_logger = lambda: NS(warn=lambda *args: None)
        self.initialpose = lambda msg: namespace['_initialpose'](self.node, msg)
        self.amcl = lambda msg: namespace['_amcl'](self.node, msg)

    def message(self, stamp=10, x=0., w=1.):
        N = self.NS
        return N(header=N(frame_id='map', stamp=N(sec=stamp, nanosec=0)),
                 pose=N(pose=N(position=N(x=x, y=0.), orientation=N(x=0., y=0., z=0., w=w)),
                        covariance=[.25] * 36))

    def test_cached_pose_and_old_amcl_cannot_initialize(self):
        self.amcl(self.message(9))
        self.assertFalse(self.node.ev.initialized)
        self.initialpose(self.message(10))
        self.amcl(self.message(9))
        self.assertFalse(self.node.ev.initialized)
        self.amcl(self.message(10))
        self.assertTrue(self.node.ev.initialized)
        # Later AMCL updates must continue to be consumed after the seed is cleared.
        self.amcl(self.message(11))
        self.assertEqual(self.node.amcl_src, 11.)

    def test_bad_pose_never_initializes(self):
        self.initialpose(self.message(10))
        self.amcl(self.message(11, x=float('nan')))
        self.assertFalse(self.node.ev.initialized)
        self.amcl(self.message(12, w=0.))
        self.assertFalse(self.node.ev.initialized)

    def test_invalid_reinitialization_revokes_ready(self):
        self.initialpose(self.message(10))
        self.amcl(self.message(11))
        self.initialpose(self.message(12, w=0.))
        self.assertFalse(self.node.ev.initialized)
        self.amcl(self.message(13))
        self.assertFalse(self.node.ev.initialized)


if __name__ == '__main__':
    unittest.main()
