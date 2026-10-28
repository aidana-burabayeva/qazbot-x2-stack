"""READY (latched) / MOTION_OK (continuous) split, user decision 2026-10-22 ("Simplify Localization Trust").
Pure evaluator, no ROS. Run: cd tests/loc_trust && python3 -m unittest -v test_ready_motion"""
import unittest

import numpy as np

from test_loc_trust import COV, L, make_inputs, room_grid, simulate_scan


class ReadyMotion(unittest.TestCase):
    def setUp(self):
        self.g = room_grid()
        self.pose = (0.0, 0.0, 0.0)
        self.scan = simulate_scan(self.g, self.pose)
        self.ev = L.TrustEvaluator(self.g, L.TrustConfig(admission_policy='strict'))
        self.ev.confirm_anchor(make_inputs(1, self.pose, self.pose, self.scan))
        r = self.ev.evaluate(make_inputs(1.2, self.pose, self.pose, self.scan))
        self.assertTrue(r['ready'] and r['motion_ok'] and r['trusted'], r)

    def at(self, t, odom=None, mo=None, **kw):
        odom = self.pose if odom is None else odom
        mo = self.pose if mo is None else mo
        kw.setdefault('scan', simulate_scan(self.g, L.compose(mo, odom)))
        return self.ev.evaluate(make_inputs(t, mo, odom, kw.pop('scan'), **kw))

    def test_stationary_nine_minute_old_amcl_stays_ready_and_motion_ok(self):
        for t in np.arange(2.0, 542.0, 30.0):            # amcl_pose / particle_cloud last at t=1, robot standing
            r = self.at(t, amcl_stamp=1, cloud_stamp=1, odom_at_amcl=self.pose, odom_at_cloud=self.pose)
            self.assertTrue(r['ready'] and r['motion_ok'], (t, r))

    def test_partial_lidar_occlusion_while_moving_is_diagnostic(self):
        for k in range(10):
            self.at(2 + k * 0.2, odom=(0.02 * k, 0, 0))
        odom = (0.2, 0, 0)
        occluded = simulate_scan(self.g, odom)
        occluded = occluded[: len(occluded) // 3] + 0.4      # 1/3 of the endpoints, pushed off the walls
        r = self.at(4.2, odom=odom, scan=occluded)
        self.assertTrue({'T4_scan_score', 'T4_scan_drop'} & set(r['diagnostics']), r)
        self.assertTrue(r['ready'] and r['motion_ok'], r)

    def test_covariance_growth_while_moving_is_diagnostic(self):
        for k in range(10):
            cov = COV * (1 + 30 * k)                        # sigma_xy 0.1 -> 0.52 m
            r = self.at(2 + k * 0.2, odom=(0.02 * k, 0, 0), amcl_cov=cov)
        self.assertIn('T2_covariance', r['diagnostics'])
        self.assertTrue(r['ready'] and r['motion_ok'], r)

    def test_pose_jump_clears_ready_until_operator(self):
        r = self.at(2.0, mo=(0.3, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'])
        self.assertIn('not_ready:T5_map_odom_jump', r['reasons'])
        for t in (4.0, 20.0):                              # jump over, everything fresh: still not READY
            self.assertFalse(self.at(t, mo=(0.3, 0, 0))['ready'])
        self.ev.confirm_anchor(make_inputs(20.0, (0.3, 0, 0), self.pose, self.scan))
        self.assertTrue(self.at(20.2, mo=(0.3, 0, 0))['ready'])

    def test_gradual_amcl_corrections_keep_ready(self):
        # T5 step semantics (bag short_nav_01 evidence): 6 x 0.06 m corrections in 3 s = 0.36 m cumulative, each < 0.10
        for k in range(1, 7):
            r = self.at(2.0 + 0.5 * k, mo=(0.06 * k, 0, 0))
            self.assertTrue(r['ready'] and r['motion_ok'], (k, r))
            self.assertNotIn('not_ready:T5_map_odom_jump', r['reasons'])

    def test_single_step_above_limit_clears_ready(self):
        r = self.at(2.0, mo=(0.15, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'], r)
        self.assertIn('not_ready:T5_map_odom_jump', r['reasons'])

    def gap_then_fresh(self, gap_kw, mo_after):
        for k in range(8):                                 # 1.6 s gap: the jump must still be caught (first fresh vs last fresh sample)
            r = self.at(2.0 + 0.2 * k, **gap_kw(2.0 + 0.2 * k))
            self.assertFalse(r['motion_ok'], r)
        return self.at(3.8, mo=mo_after)

    def test_jump_during_stale_tf_gap_clears_ready_at_first_fresh_tick(self):
        jumped = (0.3, 0, 0)
        r = self.gap_then_fresh(lambda t: dict(mo=jumped, map_odom_stamp=0.0), jumped)
        self.assertFalse(r['ready'] or r['motion_ok'], r)
        self.assertTrue(any(x.startswith('not_ready:T5_map_odom_jump') for x in r['reasons']), r)   # no stale exemption

    def test_jump_during_nan_tf_gap_clears_ready_at_first_fresh_tick(self):
        r = self.gap_then_fresh(lambda t: dict(mo=(float('nan'), 0, 0)), (0.3, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'], r)
        self.assertIn('not_ready:T5_map_odom_jump_across_gap', r['reasons'])

    def test_gap_without_jump_keeps_ready(self):
        r = self.gap_then_fresh(lambda t: dict(map_odom_stamp=0.0), self.pose)
        self.assertTrue(r['ready'] and r['motion_ok'], r)
        r = self.gap_then_fresh(lambda t: dict(mo=(float('nan'), 0, 0)), self.pose)
        self.assertTrue(r['ready'] and r['motion_ok'], r)

    def test_critical_input_loss_stops_and_recovers_without_operator(self):
        for k, stale in enumerate((dict(odom_stamp=0), dict(odom_msg_stamp=0), dict(scan_stamp=0),
                                   dict(map_odom_stamp=0))):
            t = 2 + k
            r = self.at(t, **stale)
            self.assertFalse(r['motion_ok'] or r['trusted'], (stale, r))
            self.assertTrue(r['ready'], (stale, r))
            r = self.at(t + 0.2)
            self.assertTrue(r['motion_ok'] and r['trusted'], (stale, r))

    def test_map_mismatch_clears_ready(self):
        r = self.at(2.0, map_ok='occupied_cells')
        self.assertFalse(r['ready'] or r['motion_ok'])
        self.assertFalse(self.at(2.2)['ready'])

    def test_initialpose_clears_ready_until_confirm(self):
        self.ev.relocalize()
        r = self.at(2.0)
        self.assertFalse(r['ready'] or r['motion_ok'])
        self.assertIn('not_ready:initialpose', r['reasons'])
        self.ev.confirm_anchor(make_inputs(2.0, self.pose, self.pose, self.scan))
        self.assertTrue(self.at(2.2)['motion_ok'])

    def test_ready_gate_refuses_unconverged_amcl(self):
        bad = COV * 30
        self.ev.confirm_anchor(make_inputs(2.0, self.pose, self.pose, self.scan))
        r = self.at(2.2, amcl_cov=bad)
        self.assertFalse(r['ready'])
        self.assertIn('T2_covariance', r['reasons'])
        self.assertTrue(self.at(2.4)['ready'])


class PerUpdateLatch(unittest.TestCase):
    setUp = ReadyMotion.setUp
    at = ReadyMotion.at
    gap_then_fresh = ReadyMotion.gap_then_fresh
    """T5 per accepted map->odom update (observe_map_odom), latched until the next evaluation."""
    def obs(self, x, yaw=0.0):
        self.ev.observe_map_odom((x, 0.0, yaw))

    def test_smooth_correction_between_ticks_keeps_ready(self):
        for k in range(1, 7):                               # 6 x 0.06 m over 3 s, 2 updates per tick
            self.obs(0.06 * k - 0.03 if k > 1 else 0.0)
            self.obs(0.06 * k)
            r = self.at(2.0 + 0.5 * k, mo=(0.06 * k, 0, 0))
            self.assertTrue(r['ready'] and r['motion_ok'], (k, r))

    def test_single_015_step_clears_ready(self):
        self.obs(0.15)
        r = self.at(2.0, mo=(0.15, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'], r)

    def test_out_and_back_between_ticks_clears_ready(self):
        self.obs(0.0); self.obs(0.5); self.obs(0.0)          # tick sees 0 -> 0: only the latch can catch it
        r = self.at(2.0, mo=(0.0, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'], r)
        self.assertIn('not_ready:T5_map_odom_jump', r['reasons'])

    def test_yaw_out_and_back_clears_ready(self):
        self.obs(0.0, 0.3); self.obs(0.0, 0.0)
        self.assertFalse(self.at(2.0)['ready'])

    def test_republished_identical_values_are_step_zero(self):
        for _ in range(5):
            self.obs(0.0)
        self.assertTrue(self.at(2.0)['ready'])

    def test_several_small_updates_between_ticks_keep_ready(self):
        for x in (0.09, 0.0, 0.09, 0.0, 0.09):               # each step <= 0.10 (net 0.09 at the tick)
            self.obs(x)
        r = self.at(2.0, mo=(0.09, 0, 0))
        self.assertTrue(r['ready'] and r['motion_ok'], r)

    def test_latch_cleared_after_evaluation_but_ready_stays_lost(self):
        self.obs(0.5); self.obs(0.0)
        self.assertFalse(self.at(2.0)['ready'])
        self.assertIsNone(self.ev.step_latch)
        r = self.at(2.2)
        self.assertFalse(r['ready'])
        self.assertNotIn('T5_map_odom_jump', r['reasons'])

    def test_data_gap_with_jump_clears_ready(self):
        self.obs(0.3)
        r = self.gap_then_fresh(lambda t: dict(mo=(0.3, 0, 0), map_odom_stamp=0.0), (0.3, 0, 0))
        self.assertFalse(r['ready'] or r['motion_ok'], r)

    def test_data_gap_without_jump_keeps_ready(self):
        r = self.gap_then_fresh(lambda t: dict(mo=self.pose, map_odom_stamp=0.0), self.pose)
        self.assertTrue(r['ready'] and r['motion_ok'], r)


if __name__ == '__main__':
    unittest.main()
