"""Trust gaps closed for task 20261021-x2-nav2-e2e-dryrun: AMCL loss (standing / moving), particle-cloud staleness,
frozen-content TF, served-map identity, EXPERIMENTAL ambiguity budget, anchor vs FAST-LIO drift, no silent recovery.
Pure evaluator, no ROS. Run: cd tests/loc_trust && python3 -m unittest -v test_trust_gaps"""
import math
import os
import unittest

import numpy as np

from test_loc_trust import L, make_inputs, room_grid, simulate_scan, wall_grid

HERE = os.path.dirname(os.path.abspath(__file__))
D1 = os.path.join(HERE, '..', '..', 'ws', 'src', 'maps', 'first_map_nav2_v2', 'map.yaml')


class Base(unittest.TestCase):
    def setUp(self):
        self.g = room_grid()
        self.pose = (0.0, 0.0, 0.0)
        self.scan = simulate_scan(self.g, self.pose)
        self.ev = L.TrustEvaluator(self.g, L.TrustConfig(stable_s=0))
        self.ev.set_baseline([L.soft_score(self.scan, self.g, self.pose, .05)])
        self.ev.confirm_anchor(make_inputs(1, self.pose, self.pose, self.scan))
        self.assertTrue(self.ev.evaluate(make_inputs(1.2, self.pose, self.pose, self.scan))['trusted'])


class AmclLoss(Base):
    def test_amcl_death_while_standing_is_seen_through_map_odom_tf(self):
        # Standing AMCL publishes neither amcl_pose nor particle_cloud; it re-sends map->odom per scan.
        r = self.ev.evaluate(make_inputs(10, self.pose, self.pose, self.scan, amcl_stamp=1, cloud_stamp=1, map_odom_stamp=8.9))
        self.assertFalse(r['trusted'])
        self.assertEqual(r['reasons'], ['T1_tf_stale'])

    def test_standing_without_amcl_updates_is_trusted(self):
        r = self.ev.evaluate(make_inputs(10, self.pose, self.pose, self.scan, amcl_stamp=1, cloud_stamp=1))
        self.assertTrue(r['trusted'], r)

    # Mapping of the former "while moving" tests: the rule is no longer "age > 3 s and moving" but "AMCL was due
    # (odom moved >= 0.25 m / 0.2 rad since its last output) for > pose_max_age_moving_s". `moving=True` is
    # now status only, so the old inputs (old stamps, moving=True, no displacement) must be trusted, and the
    # stale cases are reproduced through a real displacement of the odom since odom_at_amcl / odom_at_cloud.

    def test_standing_jitter_with_old_amcl_is_trusted(self):
        # 60 s standing span measured on X2: x 24 mm, y 18 mm, yaw 4 mrad; moving flag true from jitter
        odom = (0.024, 0.018, 0.004)
        r = self.ev.evaluate(make_inputs(10, self.pose, odom, self.scan, amcl_stamp=1, cloud_stamp=1, moving=True,
                                         odom_at_amcl=self.pose, odom_at_cloud=self.pose, odom_at_scan=self.pose))
        self.assertNotIn('T1_amcl_pose_stale', r['reasons'])
        self.assertNotIn('T1_particle_cloud_stale', r['reasons'])

    def test_old_cloud_alone_while_moving_flag_is_not_stale(self):
        r = self.ev.evaluate(make_inputs(10, self.pose, self.pose, self.scan, cloud_stamp=1, moving=True,
                                         odom_at_cloud=self.pose))
        self.assertEqual(r['reasons'], [])

    def test_slow_motion_below_update_min_d_is_not_stale(self):
        for k in range(1, 51):                     # 0.20 m over 10 s, no AMCL update
            t = 1.2 + k * 0.2
            r = self.ev.evaluate(make_inputs(t, self.pose, (0.2 * k / 50, 0, 0), self.scan, amcl_stamp=1, cloud_stamp=1,
                                             moving=True, odom_at_amcl=self.pose, odom_at_cloud=self.pose,
                                             odom_at_scan=(0.2 * k / 50, 0, 0)))
            self.assertNotIn('T1_amcl_pose_stale', r['reasons'])
            self.assertNotIn('T1_particle_cloud_stale', r['reasons'])

    def check_due_then_stale(self, **disp):
        lag = self.ev.cfg.pose_max_age_moving_s
        kw = dict(amcl_stamp=1, cloud_stamp=1, moving=True)
        odom = (disp.get('dx', 0.0), 0.0, disp.get('dyaw', 0.0))
        def at(t):
            return self.ev.evaluate(make_inputs(t, self.pose, odom, self.scan, odom_at_amcl=self.pose,
                                                odom_at_cloud=self.pose, odom_at_scan=odom, **kw))
        r = at(10.0)                                # just became due: AMCL still has its lag
        self.assertNotIn('T1_amcl_pose_stale', r['reasons'])
        self.assertNotIn('T1_particle_cloud_stale', r['reasons'])
        r = at(10.0 + lag - 0.1)
        self.assertNotIn('T1_amcl_pose_stale', r['reasons'])
        r = at(10.0 + lag + 0.1)
        self.assertIn('T1_amcl_pose_stale', r['reasons'])
        self.assertIn('T1_particle_cloud_stale', r['reasons'])
        return odom, lag

    def test_amcl_silent_after_due_displacement_is_stale(self):
        self.check_due_then_stale(dx=0.30)

    def test_amcl_silent_after_due_rotation_is_stale(self):
        self.check_due_then_stale(dyaw=0.25)

    def test_new_amcl_update_clears_due(self):
        odom, lag = self.check_due_then_stale(dx=0.30)
        t = 10.0 + lag + 0.3
        r = self.ev.evaluate(make_inputs(t, self.pose, odom, self.scan, amcl_stamp=t - 0.2, cloud_stamp=t - 0.2,
                                         odom_at_amcl=odom, odom_at_cloud=odom, odom_at_scan=odom, moving=True))
        self.assertNotIn('T1_amcl_pose_stale', r['reasons'])
        self.assertNotIn('T1_particle_cloud_stale', r['reasons'])

    def due_after(self, odom):
        for t in (10.0, 13.2):
            r = self.ev.evaluate(make_inputs(t, self.pose, odom, self.scan, amcl_stamp=1, cloud_stamp=1, moving=True,
                                             odom_at_amcl=self.pose, odom_at_cloud=self.pose, odom_at_scan=odom))
        return 'T1_amcl_pose_stale' in r['reasons']

    def test_due_is_per_axis_like_nav2_amcl_not_euclidean(self):
        # nav2_amcl 1.1.18 shouldUpdateFilter: |dx|>0.25 || |dy|>0.25 (odom frame). 0.30 m at 45 deg = 0.212/axis.
        k = math.sqrt(0.5)
        self.assertFalse(self.due_after((0.30 * k, 0.30 * k, 0.0)))

    def test_due_diagonal_36cm_and_strict_threshold(self):
        k = math.sqrt(0.5)
        self.assertTrue(self.due_after((0.36 * k, 0.36 * k, 0.0)))       # 0.2546 per axis > 0.25

    def test_due_exactly_at_threshold_is_not_due(self):
        self.assertFalse(self.due_after((0.25, 0.0, 0.0)))
        self.assertFalse(self.due_after((0.0, 0.0, 0.199)))

    def test_amcl_thresholds_match_nav2_defaults(self):
        c = L.TrustConfig()      # nav2_amcl humble defaults, launch does not override: keep in step with AMCL
        self.assertEqual((c.amcl_update_min_d, c.amcl_update_min_a, c.pose_max_age_moving_s), (0.25, 0.2, 3.0))


class NodeStampReject(unittest.TestCase):
    """Node-level, no rclpy.init / no DDS: the node object is built with __new__ and fed fake messages."""
    def make(self):
        import sys, types
        sys.path.insert(0, os.path.join(HERE, '..', '..', 'ws', 'src', 'navigation'))
        try:
            import localization_trust_node as N
        except ImportError as e:
            self.skipTest(str(e))
        n = N.TrustNode.__new__(N.TrustNode)
        n.stamps = {k: N.SourceStamp() for k in ('amcl', 'cloud')}
        n.cov = n.cloud = None
        n.amcl_t = n.cloud_t = float('-inf')
        n.odom_src, n.odom_at_amcl, n.odom_at_cloud, n.amcl_src = [], None, None, None
        n.odom = (0.0, 0.0, 0.0)
        clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(100e9)))
        n.get_clock = lambda: clock
        return n, N

    def stamp(self, sec):
        import types
        return types.SimpleNamespace(sec=sec, nanosec=0)

    def test_duplicate_and_old_amcl_stamp_keep_reference(self):
        n, N = self.make()
        msg = lambda sec: types_msg(sec)
        import types
        def types_msg(sec):
            return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='map', stamp=self.stamp(sec)),
                                         pose=types.SimpleNamespace(covariance=[0.0] * 36))
        n._amcl(types_msg(90))
        self.assertEqual(n.odom_at_amcl, (0.0, 0.0, 0.0))
        t0 = n.amcl_t
        n.odom = (0.5, 0.0, 0.0)
        n._amcl(types_msg(90))                  # duplicate
        n._amcl(types_msg(80))                  # older
        self.assertEqual(n.odom_at_amcl, (0.0, 0.0, 0.0))
        self.assertEqual(n.amcl_t, t0)
        n._amcl(types_msg(95))                  # newer: reference moves
        self.assertEqual(n.odom_at_amcl, (0.5, 0.0, 0.0))

    def test_duplicate_cloud_stamp_keeps_reference(self):
        n, N = self.make()
        import types
        P = types.SimpleNamespace
        def msg(sec):
            return P(header=P(frame_id='map', stamp=self.stamp(sec)),
                     particles=[P(pose=P(position=P(x=0.0, y=0.0)), weight=1.0)])
        n._cloud(msg(90))
        n.odom = (0.5, 0.0, 0.0)
        n._cloud(msg(90))
        self.assertEqual(n.odom_at_cloud, (0.0, 0.0, 0.0))
        n._cloud(msg(91))
        self.assertEqual(n.odom_at_cloud, (0.5, 0.0, 0.0))


class FrozenTf(Base):
    def run_frozen(self, msg_fn, tf_fn, secs=2.0):
        r = None
        for k in range(int(secs * 5)):
            t = 2 + k / 5
            r = self.ev.evaluate(make_inputs(t, self.pose, tf_fn(t), self.scan, odom_msg_pose=msg_fn(t)))
        return r

    def test_identical_tf_with_fresh_stamps_while_odom_moves(self):
        r = self.run_frozen(lambda t: (0.3 * (t - 2), 0.0, 0.0), lambda t: (0.0, 0.0, 0.0))
        self.assertIn('T1_tf_frozen', r['reasons'])

    def test_frozen_tf_while_turning(self):
        r = self.run_frozen(lambda t: (0.0, 0.0, 0.4 * (t - 2)), lambda t: (0.0, 0.0, 0.0))
        self.assertIn('T1_tf_frozen', r['reasons'])

    def test_standing_identical_tf_is_not_frozen(self):
        r = self.run_frozen(lambda t: (0.0, 0.0, 0.0), lambda t: (0.0, 0.0, 0.0))
        self.assertNotIn('T1_tf_frozen', r['reasons'])
        self.assertTrue(r['trusted'], r)

    def test_moving_tf_following_odom_is_not_frozen(self):
        f = lambda t: (0.02 * (t - 2), 0.0, 0.0)        # small, within the anchor tolerance
        r = self.run_frozen(f, f)
        self.assertNotIn('T1_tf_frozen', r['reasons'])


class MapIdentity(unittest.TestCase):
    def setUp(self):
        self.g = L.OccGrid.from_pgm(D1)
        self.data = np.where(self.g.occ, 100, 0).astype(np.int8).ravel()

    def test_identical_served_map(self):
        self.assertEqual(L.map_identity(self.g, self.g.res, (self.g.ox, self.g.oy), self.g.w, self.g.h, self.data), '')

    def test_mismatches(self):
        g = self.g
        self.assertEqual(L.map_identity(g, g.res * 2, (g.ox, g.oy), g.w, g.h, self.data), 'resolution')
        self.assertEqual(L.map_identity(g, g.res, (g.ox + 0.05, g.oy), g.w, g.h, self.data), 'origin')
        other = self.data.copy()
        other[np.flatnonzero(other == 100)[:3]] = 0
        self.assertEqual(L.map_identity(g, g.res, (g.ox, g.oy), g.w, g.h, other), 'occupied_cells')

    def test_evaluator_blocks_unseen_and_mismatched_map(self):
        g, pose = room_grid(), (0.0, 0.0, 0.0)
        scan = simulate_scan(g, pose)
        ev = L.TrustEvaluator(g, L.TrustConfig(stable_s=0))
        ev.set_baseline([1.0])
        ev.confirm_anchor(make_inputs(1, pose, pose, scan))
        r = ev.evaluate(make_inputs(2, pose, pose, scan, map_ok=None))     # pending gate: unseen map blocks READY
        self.assertIn('T8_map_unseen', r['reasons']); self.assertFalse(r['ready'])
        self.assertTrue(ev.evaluate(make_inputs(3, pose, pose, scan))['trusted'])
        r = ev.evaluate(make_inputs(4, pose, pose, scan, map_ok='origin'))   # mismatch after READY: READY lost
        self.assertIn('not_ready:T8_map_mismatch', r['reasons']); self.assertFalse(r['ready'] or r['trusted'])
        self.assertFalse(ev.evaluate(make_inputs(5, pose, pose, scan))['trusted'])   # map fixed: still needs operator
        ev.confirm_anchor(make_inputs(5, pose, pose, scan))
        self.assertTrue(ev.evaluate(make_inputs(6, pose, pose, scan))['trusted'])

    def test_inputs_default_is_unseen(self):
        self.assertIsNone(L.Inputs(now=0).map_ok)
        self.assertTrue(L.TrustConfig().require_map_match)


class AmbiguityBudget(unittest.TestCase):
    def test_default_is_conservative_and_2m_is_explicit_experimental(self):
        self.assertEqual(L.TrustConfig().degenerate_budget_m, 0.0)
        self.assertEqual(L.EXPERIMENTAL_DEGENERATE_BUDGET_M, 2.0)

    def test_experimental_budget_allows_two_metres_only(self):
        g, pose = wall_grid(), (-1.5, 0.0, 0.0)
        ev = L.TrustEvaluator(g, L.TrustConfig(stable_s=0.0, degenerate_budget_m=L.EXPERIMENTAL_DEGENERATE_BUDGET_M))
        sc = simulate_scan(g, pose)
        ev.set_baseline([L.soft_score(sc, g, pose, 0.05)])
        i0 = make_inputs(0.0, pose, (0, 0, 0), sc)
        ev.confirm_anchor(i0)
        self.assertTrue(ev.arm_check(i0)['ambiguous'])
        used = []
        for k in range(1, 30):
            odom = (0.0, 0.1 * k, 0.0)
            r = ev.evaluate(make_inputs(k * 0.2, pose, odom, simulate_scan(g, L.compose(pose, odom))))
            used.append((r['metrics']['degenerate_used_m'], 'T7_degenerate_budget' in r['diagnostics']))
        self.assertTrue(all(not tripped for u, tripped in used if u <= 2.0 - 1e-9))
        self.assertTrue(all(tripped for u, tripped in used if u > 2.0 + 1e-9))


class AnchorLioDrift(unittest.TestCase):
    def test_lio_scale_drift_against_correct_map_pose_is_caught(self):
        """AMCL stays right; FAST-LIO under-reports travel by 10 %. T6 trips once the gap exceeds anchor_xy_base."""
        g = room_grid()
        ev = L.TrustEvaluator(g, L.TrustConfig(stable_s=0))
        start = (-2.0, 0.5, 0.0)
        sc = simulate_scan(g, start)
        ev.set_baseline([L.soft_score(sc, g, start, .05)])
        ev.confirm_anchor(make_inputs(0, start, (0, 0, 0), sc))
        trip = None
        for k in range(1, 60):
            true_x = 0.05 * k
            odom = (0.9 * true_x, 0.0, 0.0)                     # drifting LIO
            mo = (start[0] + true_x - odom[0], start[1], 0.0)   # AMCL keeps the map pose right via map->odom
            r = ev.evaluate(make_inputs(k * 0.2, mo, odom, simulate_scan(g, L.compose(mo, odom))))
            if 'T6_anchor_disagreement' in r['diagnostics']:
                trip = true_x
                break
        self.assertIsNotNone(trip)
        self.assertAlmostEqual(trip, 1.55, delta=0.06)        # 0.1 * 1.55 > 0.15 m


class ScanTimeAlignment(Base):
    def test_scan_scored_at_its_own_pose_while_turning(self):
        # Scan taken at yaw 0; TF already at yaw 0.05 (0.5 rad/s, 0.1 s later). Scoring at the latest TF degrades
        # the score; scoring at the scan-time pose (odom_at_scan) does not.
        late = (0.0, 0.0, 0.05)
        r1 = self.ev.evaluate(make_inputs(3.0, self.pose, late, self.scan))
        r2 = self.ev.evaluate(make_inputs(3.2, self.pose, late, self.scan, odom_at_scan=(0.0, 0.0, 0.0)))
        self.assertGreater(r2['metrics']['scan_score'], 0.99)
        self.assertLess(r1['metrics']['scan_score'], r2['metrics']['scan_score'] - 0.1)


class NoSilentRecovery(Base):
    def test_transient_loss_recovers_without_settling_but_prolonged_loss_needs_operator(self):
        # was test_recovery_needs_stable_window (5 s settling removed 2026-10-22: the aborted goal is never resumed,
        # a NEW goal is required). Transient TF loss: MOTION_OK false, READY kept, MOTION_OK back with fresh data.
        r = self.ev.evaluate(make_inputs(6.2, self.pose, self.pose, self.scan, map_odom_stamp=0))
        self.assertFalse(r['trusted']); self.assertTrue(r['ready'])
        r = self.ev.evaluate(make_inputs(6.4, self.pose, self.pose, self.scan))
        self.assertTrue(r['trusted'], r)
        # loss still present when a new goal is requested = prolonged: READY lost until confirm_anchor
        self.ev.evaluate(make_inputs(6.6, self.pose, self.pose, self.scan, map_odom_stamp=0))
        self.assertEqual(self.ev.goal_request(), ['T1_tf_stale'])
        r = self.ev.evaluate(make_inputs(6.8, self.pose, self.pose, self.scan))
        self.assertFalse(r['ready'] or r['trusted'])
        self.assertTrue(r['reasons'][0].startswith('not_ready:input_loss_at_goal_request'))
        self.ev.confirm_anchor(make_inputs(7, self.pose, self.pose, self.scan))
        self.assertTrue(self.ev.evaluate(make_inputs(7.2, self.pose, self.pose, self.scan))['trusted'])
        self.assertEqual(self.ev.goal_request(), [])


if __name__ == '__main__':
    unittest.main()
