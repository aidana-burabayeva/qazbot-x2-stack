"""Regression: the measured standing envelope must stay inside the vendor footprint polygon (+ small tolerance).

Needs the STL meshes: set X2_URDF_DIR to a directory containing meshes/ (see README). Skipped otherwise.
Run: X2_URDF_DIR=/path python3 -m unittest -v tests/footprint/test_envelope_rest.py
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.environ.get('X2_URDF_DIR', HERE)
CP = (0.0867, 0.0173)                                   # control point in base_link, vendor /tf_static
VENDOR_HALF_X, VENDOR_HALF_Y = 0.20, 0.336              # a2/app2.yaml footprint extents
TOL = 0.03                                              # metres; arm posture and rounding of the vendor polygon


@unittest.skipUnless(os.path.isdir(os.path.join(MESHES, 'meshes')), 'STL meshes not available (set X2_URDF_DIR)')
class RestEnvelope(unittest.TestCase):
    def test_standing_hull_inside_vendor_footprint(self):
        os.environ['X2_URDF_DIR'] = MESHES
        sys.path.insert(0, HERE)
        import envelope
        rows = envelope.analyse(os.path.join(HERE, 'data', 'rest_standing_2026-10-21.npz'), 1.0)
        r = max(x['r'] for x in rows)
        self.assertLess(r, 0.389 + TOL)                                   # vendor circumradius about the control point
        self.assertLess(max(x['x_front'] for x in rows), VENDOR_HALF_X - CP[0] + TOL)
        self.assertGreater(min(x['x_back'] for x in rows), -VENDOR_HALF_X - CP[0] - TOL)
        self.assertLess(max(x['y_left'] for x in rows), VENDOR_HALF_Y - CP[1] + TOL)
        self.assertGreater(min(x['y_right'] for x in rows), -VENDOR_HALF_Y - CP[1] - TOL)


if __name__ == '__main__':
    unittest.main()
