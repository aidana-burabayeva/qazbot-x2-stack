"""Every recorded map->odom update of the physical X2 (TF dumped by scripts/t5/all_bags.sh to /results/t5/*.json) is fed to the
evaluator exactly as recorded (seeded state, odom_base at the update). Decision per step: block / pass. Safety: every step whose
robot displacement exceeds the threshold blocks; the candidate never passes a step the baseline blocks unless the robot moved
< threshold."""
import bisect
import glob
import inspect
import json
import os
import unittest

from test_loc_trust import L, room_grid, make_inputs, simulate_scan

VARIANT = os.environ.get('T5_VARIANT', 'candidate')
FILES = [f for f in sorted(glob.glob('/results/t5/*.json')) if not os.path.basename(f).startswith(('outcome', 'real_', 'reseed_'))]
ROWS = []


def decide(prev, new, odom):
    ev = L.TrustEvaluator(room_grid(), L.TrustConfig(admission_policy='go2'))
    ev.relocalize()
    ev.accept_initialization()
    scan = simulate_scan(room_grid(), L.compose(prev, odom))
    ev.evaluate(make_inputs(10.0, prev, odom, scan))
    if 'odom' in inspect.signature(ev.observe_map_odom).parameters:
        ev.observe_map_odom(new, odom)
    else:
        ev.observe_map_odom(new)
    r = ev.evaluate(make_inputs(10.2, new, odom, scan))
    return not r['motion_ok'], r['metrics']['jump_xy']


@unittest.skipUnless(FILES, 'no dumped physical TF')
class RealBags(unittest.TestCase):
    def test_recorded_steps(self):
        n = 0
        for f in FILES:
            d = json.load(open(f))
            mo, ob = d['map_odom'], d['odom_base']
            ts = [r[0] for r in ob]
            prev = None
            for r in mo:
                v = tuple(r[2:5])
                if prev is not None and v != prev:
                    i = min(max(bisect.bisect_left(ts, r[0]), 0), len(ob) - 1)
                    odom = tuple(ob[i][2:5])
                    blocked, jump = decide(prev, v, odom)
                    a, b = L.compose(prev, odom), L.compose(v, odom)
                    move = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                    ROWS.append(dict(bag=os.path.basename(f)[:-5], t=r[0], variant=VARIANT, blocked=blocked, jump_xy=jump, robot_move=move))
                    if move > 0.10:
                        self.assertTrue(blocked, (f, r[0], move))      # a real displacement never passes
                    if VARIANT == 'candidate':
                        self.assertEqual(blocked, move > 0.10 or abs(((v[2] - prev[2] + 3.1415926535) % 6.283185307) - 3.1415926535) > 0.10)
                    n += 1
                prev = v
        self.assertGreater(n, 5)


def tearDownModule():
    if ROWS:
        json.dump(ROWS, open('/results/t5/real_%s.json' % VARIANT, 'w'), indent=1)
