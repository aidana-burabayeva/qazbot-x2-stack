"""Re-seed on RECORDED physical TF: the operator's initialpose is replayed at its recorded time into the evaluator, together with
the recorded map->odom / odom->base stream before and after it (bags First_RViz_goal_C3_1205, RViz_goal2_1216). The first
post-seed correction (0.21 m / 3.5 m in the old epoch's terms) must not be rejected; later steps of the new epoch are judged by T5 as usual."""
import bisect
import glob
import inspect
import json
import os
import unittest

from test_loc_trust import L, room_grid, make_inputs, simulate_scan

VARIANT = os.environ.get('T5_VARIANT', 'candidate')
IPS = sorted(glob.glob('/results/t5/ip_*.json.ip'))
OUT = []


def run(bag, ip_t):
    d = json.load(open('/results/t5/%s.json' % bag))
    mo, ob = d['map_odom'], d['odom_base']
    ts = [r[0] for r in ob]
    ev = L.TrustEvaluator(room_grid(), L.TrustConfig(admission_policy='go2'))
    with_odom = 'odom' in inspect.signature(ev.observe_map_odom).parameters
    ev.relocalize(); ev.accept_initialization()          # session was initialized before this reseed
    seeded, prev, post = False, None, []
    for r in mo:
        v, t = tuple(r[2:5]), r[0]
        if v == prev:
            continue
        i = min(max(bisect.bisect_left(ts, t), 0), len(ob) - 1)
        odom = tuple(ob[i][2:5])
        if not seeded and t > ip_t:
            ev.relocalize(); ev.accept_initialization(); seeded = True    # node: initialpose, then first valid AMCL after it
        ev.observe_map_odom(v, odom) if with_odom else ev.observe_map_odom(v)
        res = ev.evaluate(make_inputs(t, v, odom, simulate_scan(room_grid(), L.compose(v, odom))))
        if seeded:
            post.append((t - ip_t, res['motion_ok'], [x for x in res['reasons'] if x.startswith('T5')], res['metrics']['jump_xy']))
        prev = v
    return post


@unittest.skipUnless(IPS, 'no initialpose dumps')
class RealReseed(unittest.TestCase):
    def test_first_post_seed_correction_not_rejected(self):
        for f in IPS:
            bag = os.path.basename(f)[3:-8]
            for k, ip in enumerate(json.load(open(f))):
                post = run(bag, ip[0])
                if not post:
                    continue
                first = post[0]
                OUT.append(dict(bag=bag, seed=k, variant=VARIANT, dt_first=round(first[0], 3), first_ok=first[1], first_jump=round(first[3], 4),
                                later_blocked=[(round(p[0], 2), round(p[3], 3)) for p in post[1:] if not p[1]]))
                if VARIANT != 'preC3':
                    self.assertTrue(first[1], (bag, k, first))


def tearDownModule():
    json.dump(OUT, open('/results/t5/reseed_%s.json' % VARIANT, 'w'), indent=1)
