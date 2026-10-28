import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/lio'))
import native_odom as no

class ClockGapRegression(unittest.TestCase):
    def test_nonpositive_stamp_does_not_replace_valid_pose(self):
        g=no.native_guard();q=(0.,-math.sqrt(.5),0.,math.sqrt(.5))
        self.assertEqual(g.add(100.,no.NATIVE_FRAME,no.NATIVE_CHILD,(0.,0.,1.,*q),100.),'ok')
        saved=g.pose
        self.assertEqual(g.add(0.,no.NATIVE_FRAME,no.NATIVE_CHILD,(0.,0.,1.,*q),100.01),'invalid_stamp')
        self.assertEqual(g.pose,saved)
        self.assertEqual(g.add(-1.,no.NATIVE_FRAME,no.NATIVE_CHILD,(0.,0.,1.,*q),100.02),'invalid_stamp')
        self.assertEqual(g.pose,saved)

    def test_velocity_not_averaged_across_unobserved_gap(self):
        tw=no.Twist0p3()
        for k in range(100):tw.add(10.+k*.006,0.,0.,0.)
        self.assertIsNone(tw.add(13.,.2,0.,0.))
        self.assertIsNone(tw.add(13.12,.2,0.,0.))
        self.assertIsNotNone(tw.add(13.31,.2,0.,0.))
