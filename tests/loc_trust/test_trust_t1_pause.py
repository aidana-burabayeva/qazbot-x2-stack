"""T1-only freshness loss keeps READY and is flagged t1_transient (pause-and-continue); T0/T5/T8 are not transient.
Operator decision 2026-10-28. No ROS."""
import unittest
from dataclasses import replace
from test_loc_trust import L, room_grid, make_inputs, simulate_scan


class Go2T1Pause(unittest.TestCase):
    def setUp(self):
        self.grid = room_grid()
        self.ev = L.TrustEvaluator(self.grid, L.TrustConfig(admission_policy='go2'))
        self.inp = make_inputs(10., (0., 0., 0.), (0., 0., 0.), simulate_scan(self.grid, (0., 0., 0.)))
        self.ev.accept_initialization()
        r = self.ev.evaluate(self.inp)
        self.assertTrue(r['trusted'] and r['ready']); self.assertFalse(r['t1_transient'])

    def test_t1_blip_keeps_ready_and_is_transient(self):
        for kw in (dict(scan_stamp=9.), dict(odom_stamp=9.), dict(odom_msg_stamp=9.), dict(map_odom_stamp=8.5)):
            r = self.ev.evaluate(replace(self.inp, **kw))
            self.assertFalse(r['trusted']); self.assertFalse(r['motion_ok'])     # motion stops during the loss
            self.assertTrue(r['ready'], kw); self.assertTrue(r['t1_transient'], kw)
            self.assertTrue(all(x.startswith('T1') for x in r['reasons']), r['reasons'])
        r = self.ev.evaluate(self.inp)
        self.assertTrue(r['trusted'] and r['ready']); self.assertFalse(r['t1_transient'])

    def test_goal_request_during_t1_loss_keeps_ready(self):
        self.ev.evaluate(replace(self.inp, scan_stamp=9.))
        self.assertEqual(self.ev.goal_request(), ['T1_scan_stale'])
        self.assertTrue(self.ev.ready)
        self.assertTrue(self.ev.evaluate(self.inp)['trusted'])                 # no reseed needed

    def test_t0_t5_t8_are_not_transient(self):
        r = self.ev.evaluate(replace(self.inp, map_odom=(float('nan'), 0., 0.)))
        self.assertFalse(r['trusted']); self.assertFalse(r['t1_transient'])
        r = self.ev.evaluate(replace(self.inp, scan_stamp=9., map_ok='size'))
        self.assertFalse(r['t1_transient']); self.assertFalse(r['ready'])
        self.ev.accept_initialization()
        r = self.ev._verdict(['T1_scan_stale', 'T5_map_odom_jump'], {})
        self.assertFalse(r['t1_transient']); self.assertFalse(r['ready'])

    def test_not_ready_is_not_transient(self):
        self.ev.relocalize()
        r = self.ev.evaluate(replace(self.inp, scan_stamp=9.))
        self.assertFalse(r['t1_transient'])


class StrictT1Pause(unittest.TestCase):
    def test_strict_flags_t1_only_and_t5_clears(self):
        ev = L.TrustEvaluator(room_grid(), L.TrustConfig(admission_policy='strict'))
        self.assertFalse(ev._verdict(['T1_scan_stale'], {})['t1_transient'])   # not READY
        ev.ready = True
        self.assertTrue(ev._verdict(['T1_scan_stale', 'T2_covariance'], {})['t1_transient'])
        self.assertFalse(ev._verdict(['T0_invalid_scan'], {})['t1_transient'])
        r = ev._verdict(['T1_scan_stale', 'T5_map_odom_jump'], {})
        self.assertFalse(r['ready']); self.assertFalse(r['t1_transient'])


if __name__ == '__main__':
    unittest.main()
