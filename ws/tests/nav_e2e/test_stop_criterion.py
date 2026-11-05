"""Unit tests for the stop criterion (no ROS). Run: cd tests/nav_e2e && python3 -m unittest -v test_stop_criterion"""
import unittest
from stop_criterion import stop_check

STILL = [(t / 10, 1.0, 2.0, 0.3, 0.0, 0.0) for t in range(30, 50)]


class StopCriterion(unittest.TestCase):
    def test_silent_smoother_after_terminal_passes(self):
        cmds = [(t / 20, .3, 0, 0) for t in range(40)]            # stream just stops; no trailing zero
        ok, ev = stop_check(cmds, STILL, t_terminal=2.0, period=.05, window_start=3.0)
        self.assertTrue(ok, ev)
        self.assertEqual(ev['last_nonzero_cmd']['value'], (.3, 0, 0))

    def test_terminal_must_be_observed(self):
        self.assertFalse(stop_check([], STILL, None, .05, 3.0)[0])

    def test_nonzero_after_grace_fails(self):
        cmds = [(1.0, .3, 0, 0), (2.2, .3, 0, 0)]
        ok, ev = stop_check(cmds, STILL, 2.0, .05, 3.0)
        self.assertFalse(ok)
        self.assertEqual(len(ev['nonzero_after_terminal']), 1)

    def test_in_flight_command_within_one_period_passes(self):
        self.assertTrue(stop_check([(2.04, .3, 0, 0)], STILL, 2.0, .05, 3.0)[0])

    def test_remotion_in_window_fails(self):
        odoms = STILL[:10] + [(4.0, 1.05, 2.0, .3, .1, 0)]
        self.assertFalse(stop_check([], odoms, 2.0, .05, 3.0)[0])

    def test_trailing_zeros_alone_do_not_hide_motion(self):
        cmds = [(2.5, .3, 0, 0)] + [(2.6 + k / 20, 0, 0, 0) for k in range(5)]
        self.assertFalse(stop_check(cmds, STILL, 2.0, .05, 3.0)[0])

    def test_ungated_monotone_ramp_down_is_allowed_only_with_decel(self):
        cmds = [(2.0, .3, 0, 0), (2.1, .24, 0, 0), (2.2, .18, 0, 0), (2.3, .12, 0, 0), (2.4, .06, 0, 0)]
        self.assertFalse(stop_check(cmds, STILL, 2.0, .05, 3.0)[0])
        self.assertTrue(stop_check(cmds, STILL, 2.0, .05, 3.0, decel=.6)[0])
        bumped = cmds[:3] + [(2.3, .2, 0, 0)]
        self.assertFalse(stop_check(bumped, STILL, 2.0, .05, 3.0, decel=.6)[0])


if __name__ == '__main__':
    unittest.main()
