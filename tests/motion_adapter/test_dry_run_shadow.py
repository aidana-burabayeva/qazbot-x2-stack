"""Dry-run shadow state machine (task 20261021-x2-nav2-e2e-dryrun, G1/G2). Actual AdapterCore, no ROS, no MC."""
import unittest
from test_adapter_core import Sim, OURS, published
from x2_motion_adapter import ACTIVE, IDLE, LATCHED

FAULTS = ('localization_untrusted', 'foreign_velocity_publisher', 'vendor_autonomy_active')


def inject(s, reason):
    s.localized, s.loc_reason, s.foreign_pub = True, '', ''
    if reason == 'localization_untrusted':
        s.localized, s.loc_reason = False, reason
    if reason == 'foreign_velocity_publisher':
        s.foreign_pub = 'continuous_walker'
    if reason == 'vendor_autonomy_active':
        s.mc.source = 'pnc'


class ShadowMatchesLive(unittest.TestCase):
    def scenario(self, live):
        """Drive, fault while driving, recover the fault, keep commanding, resume, fresh command."""
        out = {}
        for reason in FAULTS:
            s = Sim(live=live)
            seq = [d.state for d in s.run(0.4, resend=(.3, 0, 0), mc_source=OURS)]
            inject(s, reason)
            seq += [d.state for d in s.run(0.2, resend=(.3, 0, 0), mc_source='pnc' if reason == 'vendor_autonomy_active' else OURS)]
            inject(s, '')
            stuck = s.run(1.0, resend=(.3, 0, 0), mc_source=OURS)
            seq += [d.state for d in stuck]
            out[reason] = (seq, s.core.latch_reason, [p for p in published(stuck) if p != (0., 0., 0.)])
            s.core.resume(now=s.t)
            out[reason] += ([d.state for d in s.run(0.1)], published(s.run(0.2, resend=(.3, 0, 0), mc_source=OURS))[:1])
        return out

    def test_dry_run_state_sequence_equals_live(self):
        live, dry = self.scenario(True), self.scenario(False)
        for reason in FAULTS:
            with self.subTest(reason=reason):
                self.assertEqual(dry[reason], live[reason])
                seq, latch, motion_while_latched, after_resume, first = dry[reason]
                self.assertEqual(latch, reason)
                self.assertIn(LATCHED, seq)
                self.assertEqual(motion_while_latched, [])
                self.assertEqual(after_resume, [IDLE] * len(after_resume))
                self.assertEqual(first, [(.3, 0., 0.)])


class ShadowTakeover(unittest.TestCase):
    def test_takeover_latches_and_release_does_not_resume(self):
        s = Sim(live=False)
        s.run(0.5, resend=(.3, 0, 0), mc_source=OURS)          # fake MC reports our source (as after registration)
        self.assertEqual(s.core.state, ACTIVE)
        r = s.run(0.1, resend=(.3, 0, 0), mc_source='rc')       # PS5 touched
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'operator_takeover')
        self.assertEqual([p for p in published(r) if p != (0., 0., 0.)], [])
        self.assertEqual(published(r).count((0., 0., 0.)), s.cfg.zero_burst)
        self.assertEqual(published(s.run(3.0, resend=(.3, 0, 0), mc_source=OURS)), [])   # rc released: still latched
        self.assertTrue(s.core.resume(now=s.t))
        self.assertEqual(published(s.run(0.3, mc_source=OURS)), [])                       # old command not replayed
        self.assertEqual(published(s.run(0.1, resend=(.3, 0, 0), mc_source=OURS))[0], (.3, 0., 0.))

    def test_hardware_idle_source_rc_is_accepted_but_any_change_is_takeover(self):
        s = Sim(live=False)
        r = s.run(1.0, resend=(.3, 0, 0), mc_source='rc')      # stock robot: idle source is rc; no registration in dry run
        self.assertEqual(s.core.state, ACTIVE)                  # no control_not_acquired in the shadow
        self.assertTrue(all(p == (.3, 0., 0.) for p in published(r)))
        s.run(0.1, resend=(.3, 0, 0), mc_source='app_proxy')
        self.assertEqual(s.core.latch_reason, 'operator_takeover')

    def test_live_still_requires_acquisition(self):
        s = Sim(live=True)
        s.run(1.0, resend=(.3, 0, 0), mc_source='rc')
        self.assertEqual(s.core.latch_reason, 'control_not_acquired')

    def test_stop_and_resume_in_dry_run(self):
        s = Sim(live=False)
        s.run(0.3, resend=(.3, 0, 0), mc_source=OURS)
        s.core.stop()
        r = s.run(0.3, resend=(.3, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'stop_requested')
        self.assertEqual(published(r), [(0., 0., 0.)] * s.cfg.zero_burst)

    def test_each_fault_refused_from_idle_in_dry_run(self):
        for reason in FAULTS:
            with self.subTest(reason=reason):
                s = Sim(live=False)
                inject(s, reason)
                r = s.run(0.3, resend=(.3, 0, 0), mc_source='pnc' if reason == 'vendor_autonomy_active' else OURS)
                self.assertEqual(published(r), [])
                self.assertEqual((s.core.state, s.core.latch_reason), (LATCHED, reason))


if __name__ == '__main__':
    unittest.main()
