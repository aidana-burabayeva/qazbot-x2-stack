import unittest
from dataclasses import replace
from test_loc_trust import L, room_grid, make_inputs, simulate_scan

class ReseedHistory(unittest.TestCase):
    def setUp(self):
        self.ev=L.TrustEvaluator(room_grid(),L.TrustConfig(admission_policy='go2'))
        self.inp=make_inputs(10.,(0.,0.,0.),(0.,0.,0.),simulate_scan(room_grid(),(0.,0.,0.)))
    def test_explicit_reseed_does_not_compare_to_previous_epoch(self):
        self.ev.accept_initialization();self.ev.evaluate(self.inp)
        self.ev.mo_gap=True
        self.ev.observe_map_odom((.2,0.,0.))
        self.ev.relocalize()
        self.assertFalse(self.ev.initialized)
        self.ev.accept_initialization()
        result=self.ev.evaluate(replace(self.inp,map_odom=(1.,0.,0.)))
        self.assertTrue(result['motion_ok'],result['reasons'])
    def test_new_epoch_jump_still_revokes_initialization(self):
        self.ev.relocalize();self.ev.accept_initialization()
        self.ev.evaluate(self.inp)
        result=self.ev.evaluate(replace(self.inp,map_odom=(.2,0.,0.)))
        self.assertFalse(result['motion_ok'])
        self.assertIn('T5_map_odom_jump',result['reasons'])
        self.assertFalse(self.ev.initialized)

if __name__=='__main__':unittest.main()
