"""T5 map->odom continuity must be a ROBOT displacement, not the translation of the transform.

The same recorded / constructed TF sequence is fed to the baseline (C3 as installed on the X2) and to the candidate
(T5_VARIANT=baseline|candidate). Ground truth is the pose the robot really has in the map (simulator-style truth):
a localizer error is the difference between the estimated and the true pose, independent of the metric under test.
Outcomes are written to /results/t5/outcome_<variant>.json for the before/after table.
"""
import inspect
import json
import math
import os
import random
import unittest
from dataclasses import replace

from test_loc_trust import L, room_grid, make_inputs, simulate_scan

VARIANT = os.environ.get('T5_VARIANT', 'candidate')
CAND = VARIANT != 'baseline'      # 'baseline' = C3 without the T5 geometry; anything else = tree under test
OUT = {}
# physical X2, bag S4_takeover_b_1132, 1793100629.517 (evidence/S4_takeover_b_1132/map_correction_geometry.json)
S4_OLD = (-7.776306876357339, 3.738193736449905, 2.676965024739606)
S4_NEW = (-7.688323971454602, 3.686754797041766, 2.700739923156257)
S4_ODOM = (-0.5683582834746513, -2.6257122306211644, 2.5385096373230955)


def mk(policy='go2'):
    return L.TrustEvaluator(room_grid(), L.TrustConfig(admission_policy=policy))


def feed(ev, t, mo, odom, scan=None):
    """What the node does for one accepted map->odom update followed by an evaluation tick."""
    if 'odom' in inspect.signature(ev.observe_map_odom).parameters:
        ev.observe_map_odom(mo, odom)
    else:
        ev.observe_map_odom(mo)
    scan = simulate_scan(room_grid(), L.compose(mo, odom)) if scan is None else scan
    return ev.evaluate(make_inputs(t, mo, odom, scan))


def seeded(ev, mo, odom, t=10.0):
    ev.relocalize()
    ev.accept_initialization()
    r = feed(ev, t, mo, odom)
    assert ev.initialized, r
    return r


def robot_move(prev, new, odom):
    a, b = L.compose(prev, odom), L.compose(new, odom)
    return math.hypot(b[0] - a[0], b[1] - a[1])


def rec(name, r, ev):
    OUT[name] = dict(variant=VARIANT, motion_ok=r['motion_ok'], initialized=ev.initialized,
                     reasons=r['reasons'], jump_xy=r['metrics'].get('jump_xy'), jump_yaw=r['metrics'].get('jump_yaw'))


class CaseA(unittest.TestCase):
    """map->odom changed, the robot position moved little: no false reset."""

    def test_A1_recorded_physical_s4_correction(self):
        ev = mk()
        seeded(ev, S4_OLD, S4_ODOM)
        r = feed(ev, 10.2, S4_NEW, S4_ODOM)
        rec('A1_s4_recorded', r, ev)
        self.assertAlmostEqual(robot_move(S4_OLD, S4_NEW, S4_ODOM), 0.0396, places=3)   # independent oracle
        if CAND:
            self.assertTrue(r['motion_ok'], r['reasons'])
            self.assertAlmostEqual(r['metrics']['jump_xy'], robot_move(S4_OLD, S4_NEW, S4_ODOM), places=6)
        else:
            self.assertIn('T5_map_odom_jump', r['reasons'])      # the confirmed false reset of the baseline
            self.assertFalse(ev.initialized)

    def test_A2_normal_amcl_corrections_with_truth(self):
        """Robot walks a path; AMCL error random-walks by <= 5 cm / 1 deg per update: never a reset."""
        rng = random.Random(5)
        ev = mk()
        truth = lambda k: (-4.0 + 0.02 * k, 1.0 + 0.5 * math.sin(k / 40), 0.4 * math.sin(k / 60))   # simulator truth in map
        odom0 = lambda k: (0.02 * k, 0.5 * math.sin(k / 40), 0.4 * math.sin(k / 60))             # same motion, odom frame
        err = [0.0, 0.0, 0.0]
        mo = L.compose(truth(0), L.inverse(odom0(0)))
        seeded(ev, mo, odom0(0))
        worst = 0.0
        for k in range(1, 200):
            if k % 5 == 0:   # AMCL update
                err = [err[0] + rng.uniform(-.03, .03), err[1] + rng.uniform(-.03, .03), err[2] + rng.uniform(-.015, .015)]
                mo = L.compose(L.compose(truth(k), tuple(err)), L.inverse(odom0(k)))
            r = feed(ev, 10.0 + 0.2 * k, mo, odom0(k))
            worst = max(worst, r['metrics']['jump_xy'])
            self.assertTrue(r['motion_ok'], (k, r['reasons']))
        rec('A2_amcl_walk', r, ev)
        self.assertLess(worst, 0.10)

    def test_A3_translation_cancelled_by_rotation(self):
        """Large transform translation, tiny robot displacement (a pure rotation about the robot)."""
        odom = (3.0, -2.0, 0.5)
        old = (1.0, 1.0, 0.3)
        pose = L.compose(old, odom)
        dth = 0.03
        new_pose = (pose[0], pose[1], pose[2] + dth)
        new = L.compose(new_pose, L.inverse(odom))
        self.assertGreater(math.hypot(new[0] - old[0], new[1] - old[1]), 0.10)    # old metric sees > 0.10 m
        self.assertLess(robot_move(old, new, odom), 1e-9)
        ev = mk()
        seeded(ev, old, odom)
        r = feed(ev, 10.2, new, odom)
        rec('A3_rotation_about_robot', r, ev)
        self.assertEqual('T5_map_odom_jump' in r['reasons'], not CAND)

    def test_A4_odom_origin_invariance(self):
        """The same physical correction expressed in differently chosen odom frames gives the same T5 metric."""
        rng = random.Random(9)
        vals = []
        for _ in range(12):
            d = (rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(-3, 3))       # arbitrary odom-origin shift
            old, new, odom = L.compose(S4_OLD, d), L.compose(S4_NEW, d), L.compose(L.inverse(d), S4_ODOM)
            ev = mk()
            seeded(ev, old, odom)
            vals.append(feed(ev, 10.2, new, odom)['metrics']['jump_xy'])
        OUT['A4_invariance_spread'] = dict(variant=VARIANT, spread=max(vals) - min(vals), values=[round(v, 4) for v in vals])
        if CAND:
            self.assertLess(max(vals) - min(vals), 1e-6)
        else:
            self.assertGreater(max(vals) - min(vals), 0.01)    # baseline depends on the origin


class CaseB(unittest.TestCase):
    """Real global jumps must still block motion."""

    def test_B1_wrong_corridor(self):
        odom = (2.0, 0.5, 0.2)
        truth = (-1.0, 0.0, 0.0)
        mo_true = L.compose(truth, L.inverse(odom))
        wrong = L.compose(L.compose(truth, (0.0, 2.6, 0.0)), L.inverse(odom))      # next corridor, 2.6 m aside
        ev = mk()
        seeded(ev, mo_true, odom)
        r = feed(ev, 10.2, wrong, odom)
        rec('B1_wrong_corridor', r, ev)
        self.assertGreater(robot_move(mo_true, wrong, odom), 2.0)
        self.assertFalse(r['motion_ok'])
        self.assertIn('T5_map_odom_jump', r['reasons'])
        self.assertFalse(ev.initialized)

    def test_B2_small_translation_big_lever_arm(self):
        """Yaw error 0.06 rad (under 0.10) at 6 m from the odom origin moves the robot 0.36 m: a real jump."""
        odom = (6.0, 0.0, 0.0)
        old = (0.0, 0.0, 0.0)
        new = (0.0, 0.0, 0.06)
        self.assertLess(math.hypot(new[0] - old[0], new[1] - old[1]), 1e-9)
        ev = mk()
        seeded(ev, old, odom)
        r = feed(ev, 10.2, new, odom)
        rec('B2_lever_arm', r, ev)
        self.assertGreater(robot_move(old, new, odom), 0.3)
        if CAND:
            self.assertFalse(r['motion_ok'])
            self.assertIn('T5_map_odom_jump', r['reasons'])

    def test_B3_yaw_flip(self):
        odom = (1.0, 1.0, 0.0)
        ev = mk()
        seeded(ev, (0.0, 0.0, 0.0), odom)
        r = feed(ev, 10.2, (0.0, 0.0, 0.5), odom)
        rec('B3_yaw_0p5', r, ev)
        self.assertFalse(r['motion_ok'])
        self.assertIn('T5_map_odom_jump', r['reasons'])

    def test_B4_jump_across_stale_gap(self):
        odom = (2.0, 0.5, 0.2)
        old = (0.0, 0.0, 0.0)
        new = L.compose(L.compose(L.compose(old, odom), (0.0, 1.5, 0.0)), L.inverse(odom))
        ev = mk()
        seeded(ev, old, odom)
        stale = make_inputs(10.2, old, odom, simulate_scan(room_grid(), L.compose(old, odom)), map_odom_stamp=8.0)
        ev.evaluate(stale)
        r = feed(ev, 10.4, new, odom)
        rec('B4_across_gap', r, ev)
        self.assertFalse(r['motion_ok'])
        self.assertTrue(any(x.startswith('T5') for x in r['reasons']), r['reasons'])

    def test_B5_latched_between_ticks(self):
        """0 -> far -> back to 0 between two ticks: the far step is not lost."""
        odom = (1.0, 0.0, 0.0)
        ev = mk()
        seeded(ev, (0.0, 0.0, 0.0), odom)
        far = L.compose(L.compose((0.0, 0.0, 0.0), L.compose(odom, (0.0, 3.0, 0.0))), L.inverse(odom))
        for mo in (far, (0.0, 0.0, 0.0)):
            if 'odom' in inspect.signature(ev.observe_map_odom).parameters:
                ev.observe_map_odom(mo, odom)
            else:
                ev.observe_map_odom(mo)
        r = ev.evaluate(make_inputs(10.2, (0.0, 0.0, 0.0), odom, simulate_scan(room_grid(), odom)))
        rec('B5_latched', r, ev)
        self.assertIn('T5_map_odom_jump', r['reasons'])

    def test_B6_missing_odom_is_not_weaker(self):
        """No odom pose known at the update: fall back to the old translation rule (never weaker)."""
        if not CAND:
            self.skipTest('baseline has no odom argument')
        ev = mk()
        ev.observe_map_odom((0.0, 0.0, 0.0), None)
        ev.observe_map_odom((0.2, 0.0, 0.0), None)
        self.assertEqual(ev.step_latch[0], 0.2)


class CaseC(unittest.TestCase):
    """New initialpose: history of the previous localization must not cause a false reject; a new real jump still does."""

    def test_C1_reseed_far_from_previous_epoch(self):
        odom = (2.0, 0.5, 0.2)
        ev = mk()
        seeded(ev, (0.0, 0.0, 0.0), odom)
        ev.observe_map_odom(*(((0.05, 0.0, 0.0), odom) if CAND else (((0.05, 0.0, 0.0)),)))
        ev.mo_gap = True
        ev.relocalize()
        ev.accept_initialization()
        newm = L.compose(L.compose((5.0, 3.0, 1.0), L.inverse(odom)), (0, 0, 0))     # operator put the pose 5 m away
        r = feed(ev, 20.0, newm, odom)
        rec('C1_reseed_far', r, ev)
        self.assertTrue(r['motion_ok'], r['reasons'])
        self.assertTrue(ev.initialized)

    def test_C2_real_jump_after_reseed_blocks(self):
        odom = (2.0, 0.5, 0.2)
        ev = mk()
        ev.relocalize()
        ev.accept_initialization()
        m0 = L.compose((5.0, 3.0, 1.0), L.inverse(odom))
        feed(ev, 20.0, m0, odom)
        m1 = L.compose(L.compose((5.0, 3.0, 1.0), (0.0, 1.2, 0.0)), L.inverse(odom))
        r = feed(ev, 20.2, m1, odom)
        rec('C2_jump_after_reseed', r, ev)
        self.assertFalse(r['motion_ok'])
        self.assertIn('T5_map_odom_jump', r['reasons'])
        self.assertFalse(ev.initialized)

    def test_C3_second_reseed_after_reset_by_jump(self):
        """Jump revokes the init; a new initialpose recovers at once and the old pre-jump step does not return."""
        odom = (2.0, 0.5, 0.2)
        ev = mk()
        seeded(ev, (0.0, 0.0, 0.0), odom)
        r = feed(ev, 10.2, L.compose(L.compose(L.compose((0, 0, 0), odom), (0, 2.0, 0)), L.inverse(odom)), odom)
        self.assertFalse(ev.initialized)
        ev.relocalize()
        ev.accept_initialization()
        r = feed(ev, 11.0, (0.1, 0.0, 0.0), odom)
        rec('C3_recover_after_reset', r, ev)
        self.assertTrue(r['motion_ok'], r['reasons'])


def tearDownModule():
    if not os.path.isdir('/results/t5'):
        return
    os.makedirs('/results/t5', exist_ok=True)
    path = '/results/t5/outcome_%s.json' % VARIANT
    old = json.load(open(path)) if os.path.exists(path) else {}
    old.update(OUT)
    json.dump(old, open(path, 'w'), indent=1, sort_keys=True)


if __name__ == '__main__':
    unittest.main()
