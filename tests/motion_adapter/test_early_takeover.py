"""Regression: observed RC takeover must latch even during initial acquisition grace."""
import unittest
from test_adapter_core import Sim, OURS
from x2_motion_adapter import LATCHED

class EarlyTakeover(unittest.TestCase):
    def test_observed_takeover_during_acquisition_window_latches_immediately(self):
        s=Sim()
        s.run(.06,resend=(.3,0,0),mc_source=OURS)
        decision=s.run(.02,resend=(.3,0,0),mc_source='rc')[0]
        self.assertEqual(decision.state,LATCHED)
        self.assertEqual(decision.reason,'operator_takeover')
        self.assertEqual(decision.publish,(0.,0.,0.))
        for d in s.run(.6,resend=(.3,0,0),mc_source=OURS):
            self.assertEqual(d.state,LATCHED)
            self.assertIn(d.publish,(None,(0.,0.,0.)))

if __name__=='__main__':unittest.main()
