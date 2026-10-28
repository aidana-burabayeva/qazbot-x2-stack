"""Adapter core: T1-only localization loss PAUSES (zero, IDLE, no latch) and continues; > t1_pause_max_s latches;
non-T1 loss and other faults latch immediately (operator decision 2026-10-28). No ROS."""
import unittest
from test_adapter_core import ACTIVE, IDLE, LATCHED, OURS, Sim, published


class T1Sim(Sim):
    t1_ok = False

    def run(self, seconds, resend=None, mc_source=None):
        n, res = int(round(seconds * self.cfg.publish_hz)), []
        for _ in range(n):
            self.t += 1.0 / self.cfg.publish_hz
            self.mc.stamp = self.t
            if mc_source is not None:
                self.mc.source = mc_source
            if resend is not None:
                self.send(*resend)
            res.append(self.core.step(self.t, self.cmd, self.cmd_t, self.mc, self.localized, self.loc_reason,
                                      self.foreign_pub, self.t1_ok))
        return res


def drive():
    s = T1Sim()
    s.loc_reason = 'localization_untrusted'
    r = s.run(0.5, resend=(0.3, 0.0, 0.0), mc_source=OURS)
    assert r[-1].state == ACTIVE
    return s


class T1Pause(unittest.TestCase):
    def test_blip_pauses_with_zero_then_continues(self):
        s = drive()
        s.localized, s.t1_ok = False, True
        r = s.run(2.0, resend=(0.3, 0.0, 0.0), mc_source=OURS)     # Nav2 keeps commanding during the loss
        self.assertTrue(all(d.state == IDLE for d in r))
        self.assertTrue(all(p == (0.0, 0.0, 0.0) for p in published(r)))   # only the zero burst, no motion
        self.assertEqual(len(published(r)), s.cfg.zero_burst)
        self.assertTrue(r[0].reason.startswith('t1_pause:'))
        s.localized = True
        r = s.run(0.5, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        self.assertEqual(r[-1].state, ACTIVE); self.assertEqual(published(r)[-1], (0.3, 0.0, 0.0))
        self.assertIsNone(s.core.pause_since); self.assertEqual(s.core.counters['latched'], 0)

    def test_prolonged_t1_loss_latches(self):
        s = drive()
        s.localized, s.t1_ok = False, True
        r = s.run(2.9, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        self.assertNotEqual(r[-1].state, LATCHED)
        r = s.run(0.3, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        self.assertEqual(r[-1].state, LATCHED); self.assertEqual(s.core.latch_reason, 'localization_untrusted')
        s.localized = True
        self.assertEqual(s.run(0.2, resend=(0.3, 0.0, 0.0), mc_source=OURS)[-1].state, LATCHED)

    def test_prolonged_loss_latches_even_without_commands(self):
        s = drive()
        s.localized, s.t1_ok = False, True
        s.run(0.2, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        r = s.run(3.0, mc_source=OURS)                               # gate closed upstream: no commands
        self.assertEqual(r[-1].state, LATCHED)

    def test_non_t1_loss_latches_immediately(self):
        s = drive()
        s.localized, s.t1_ok = False, False                         # T5 / T8 / not ready / stale trust status
        self.assertEqual(s.run(0.04, resend=(0.3, 0.0, 0.0), mc_source=OURS)[0].state, LATCHED)

    def test_classification_turning_non_t1_during_pause_latches(self):
        s = drive()
        s.localized, s.t1_ok = False, True
        s.run(0.5, resend=(0.3, 0.0, 0.0), mc_source=OURS)
        s.t1_ok = False                                              # e.g. T5 jump across the gap
        self.assertEqual(s.run(0.04, mc_source=OURS)[0].state, LATCHED)

    def test_other_fault_during_t1_loss_latches(self):
        s = drive()
        s.localized, s.t1_ok, s.foreign_pub = False, True, 'ContinuousWalker'
        self.assertEqual(s.run(0.04, resend=(0.3, 0.0, 0.0), mc_source=OURS)[0].state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'localization_untrusted')   # existing _fault() priority order

    def test_without_t1_flag_behaviour_is_unchanged(self):
        s = drive()
        s.localized = False
        self.assertEqual(s.run(0.04, resend=(0.3, 0.0, 0.0), mc_source=OURS)[0].state, LATCHED)


if __name__ == '__main__':
    unittest.main()
