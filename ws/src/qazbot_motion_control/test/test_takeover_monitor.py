"""TakeoverMonitor (T6 2026-11-06): manual-intervention evidence while a goal owns the driver, in IDLE as well as
ACTIVE. AimDK X2 1.1.0 5.1.3 arbitration (non-zero command takes over; after a timeout the source name stays) and
5.1.2 (the remote controller commands on /aima/mc/locomotion/velocity). No ROS, no robot, no MC."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')))
from x2_motion_adapter import ACTIVE, IDLE, LATCHED, TakeoverMonitor  # noqa: E402
from test_adapter_core import OURS, Sim, published  # noqa: E402

ZERO = (0.0, 0.0, 0.0)


class Evidence(unittest.TestCase):
    def test_idle_rc_source_and_rc_zero_stream_are_not_evidence(self):
        m = TakeoverMonitor(OURS)
        m.arm('rc')                                         # boot / after operator use: MC idle source is rc
        for _ in range(100):
            self.assertEqual(m.on_mc_source(True, 'rc'), '')
            self.assertEqual(m.on_command(True, 'rc', ZERO), '')   # soc0_rc streams zeros at ~45 Hz
        self.assertEqual(m.evidence, '')

    def test_non_zero_rc_command_while_armed_is_takeover(self):
        m = TakeoverMonitor(OURS)
        m.arm('rc')
        ev = m.on_command(True, 'rc', (0.0, 0.0, 0.4))
        self.assertIn("'rc'", ev)
        self.assertEqual(m.evidence, ev)

    def test_any_non_zero_component_counts_like_mc(self):
        for v in ((1e-6, 0.0, 0.0), (0.0, -0.2, 0.0), (0.0, 0.0, -1e-9), (math.nan, 0.0, 0.0)):
            self.assertTrue(TakeoverMonitor(OURS).on_command(True, 'app_proxy', v), v)

    def test_own_commands_and_unarmed_commands_are_ignored(self):
        m = TakeoverMonitor(OURS)
        m.arm(OURS)
        self.assertEqual(m.on_command(True, OURS, (0.3, 0.0, 0.0)), '')      # our own publications echo back
        self.assertEqual(m.on_command(False, 'rc', (0.3, 0.0, 0.0)), '')     # between goals: operator may drive

    def test_source_edge_from_ours_is_takeover_even_without_command_stream(self):
        m = TakeoverMonitor(OURS)
        m.arm('rc')
        self.assertEqual(m.on_mc_source(True, 'rc'), '')
        self.assertEqual(m.on_mc_source(True, OURS), '')                     # we acquired control
        self.assertEqual(m.on_mc_source(True, OURS), '')                     # idle: source stays ours (5.1.3)
        self.assertIn(f'{OURS} -> rc', m.on_mc_source(True, 'rc'))

    def test_source_edge_while_unarmed_is_not_takeover_and_arm_resets(self):
        m = TakeoverMonitor(OURS)
        m.arm(OURS)
        self.assertEqual(m.on_mc_source(False, 'rc'), '')
        self.assertEqual(m.on_mc_source(True, 'rc'), '')                     # edge consumed while unarmed
        m.on_command(True, 'rc', (0.1, 0.0, 0.0))
        m.arm('rc')
        self.assertEqual(m.evidence, '')
        self.assertFalse(m.ours_seen)


class BlockerReproduction(unittest.TestCase):
    """The T5 item-4 sequence at core level with the node glue (_takeover: motion disarmed, core.stop)."""

    def test_takeover_in_idle_gap_latches_and_release_does_not_resume(self):
        s, m = Sim(), TakeoverMonitor(OURS)
        armed = True
        m.arm(s.mc.source)
        r = s.run(1.0, resend=(0.3, 0.0, 0.0), mc_source=OURS)              # driving the goal
        self.assertTrue(published(r))
        m.on_mc_source(armed, OURS)
        r = s.run(1.0, mc_source=OURS)                                      # Nav2 gap / T1 pause: IDLE, source ours
        self.assertEqual(s.core.state, IDLE)
        ev = m.on_command(armed, 'rc', (0.5, 0.0, 0.0))                     # operator moves the stick
        self.assertTrue(ev)
        armed = False                                                       # node glue (_takeover)
        s.core.stop('operator_takeover')
        s.run(2.0, mc_source='rc')                                          # operator drives, then releases
        r = s.run(2.0, resend=(0.3, 0.0, 0.0), mc_source=OURS)              # rc lease expired; Nav2 old goal
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'operator_takeover')
        self.assertTrue(all(p == (0.0, 0.0, 0.0) for p in published(r)))

    def test_without_evidence_the_gap_is_not_a_takeover(self):
        s, m = Sim(), TakeoverMonitor(OURS)
        m.arm(s.mc.source)
        s.run(1.0, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        m.on_mc_source(True, OURS)
        s.run(1.0, mc_source=OURS)
        self.assertEqual(m.on_command(True, 'rc', ZERO), '')
        self.assertEqual(m.on_mc_source(True, OURS), '')
        r = s.run(0.5, resend=(0.3, 0.0, 0.0), mc_source=OURS)              # same goal continues (T1 pause design)
        self.assertEqual(s.core.state, ACTIVE)
        self.assertTrue(published(r))


if __name__ == '__main__':
    unittest.main()
