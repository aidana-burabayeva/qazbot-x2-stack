#!/usr/bin/env python3
"""Offline tests of ws/src/lio/native_odom.py (native_imu footprint, guards, compare, LIO plausibility) and of the
passive native_odom_compare node, on the trial06 fixture (tests/s3/data/trial06_native_series.json.gz: leg odometry
and /qazbot_lio/odom_level of the in-place 67 deg turn whose LIO drifted 1.77 m, lio-drift diagnosis 20261027).

No robot, no MC, no hardware DDS: the node test runs in-process in an isolated domain (88, localhost); its
callbacks are fed directly and its publishers are captured.

Run: bash -c 'source ws/src/navigation/env.sh && ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
              python3 -m unittest -v tests/lio/test_native_odom.py'
"""
import gzip
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LIO_SRC = ROOT / 'ws' / 'src' / 'lio'
sys.path.insert(0, str(LIO_SRC))
import native_odom as no  # noqa: E402

D = json.load(gzip.open(ROOT / 'tests' / 's3' / 'data' / 'trial06_native_series.json.gz', 'rt'))
ON = next(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)          # first nonzero command (receive time)
OFF = max(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)          # last nonzero command
LEG = ('leg_odom', 'lidar_imu_chest_front')
LIO = ('odom', 'base_footprint')


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def axis_q(axis, ang):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(ang / 2) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2))


def lio_t7(r, xform=None):
    """fixture lio row [recv, stamp, x, y, yaw] -> pose7 (optionally moved by an SE2 (tx, ty, th))."""
    x, y, yaw = r[2], r[3], r[4]
    if xform:
        tx, ty, th = xform
        x, y, yaw = tx + math.cos(th) * x - math.sin(th) * y, ty + math.sin(th) * x + math.cos(th) * y, yaw + th
    return (x, y, 0.0, *yaw_q(yaw))


def replay(lio_xform=None, **kw):
    """Feed the fixture in receive order through guards + Compare exactly as the node does; returns rows etc."""
    ev = sorted([(r[0], 'n', r) for r in D['odom']] + [(r[0], 'l', r) for r in D['lio']], key=lambda e: e[0])
    ng, lg, c = no.native_guard(), no.lio_guard(), no.Compare(**kw)
    rows = []
    for recv, k, r in ev:
        if k == 'n':
            if ng.add(r[1], *LEG, r[2:9], recv) in ('ok', 'stamp_back'):
                p = ng.pose
                c.add_native(p[0], p[1], p[2], p[4])
        elif lg.add(r[1], *LIO, lio_t7(r, lio_xform), recv) in ('ok', 'stamp_back'):
            p = lg.pose
            rows += c.add_lio(p[0], p[1], p[2], p[4])
    return rows, ng, lg, c


class Footprint(unittest.TestCase):
    def test_constant_and_formula_match_level_bridge(self):
        """native_imu = level_bridge on_odom with R_lv = I (same LiDAR axis, same projection), every trial06 sample."""
        try:
            import numpy as np
            import level_bridge as lb
        except ImportError as e:          # rclpy missing: env.sh not sourced
            self.skipTest(str(e))
        self.assertEqual(tuple(lb.LIDAR_FWD_IN_IMU.tolist()), no.LIDAR_FWD_IN_IMU)

        class Q:
            pass
        for r in D['odom']:
            q = Q()
            q.x, q.y, q.z, q.w = r[5:9]
            f = np.eye(3) @ lb.quat_to_rot(q) @ lb.LIDAR_FWD_IN_IMU
            self.assertAlmostEqual(no.imu_footprint_yaw(r[5:9])[0], math.atan2(f[1], f[0]), places=12)

    def test_trial06_turn(self):
        """In-place left turn: native IMU footprint 67.5 deg / 0.115 m (design 3.3), LIO 61.8 deg / 1.77 m."""
        rows = replay()[0]
        a, z = [r for r in rows if r['t'] < ON - 0.2][-1], rows[-1]
        self.assertAlmostEqual(math.degrees(z['nyaw'] - a['nyaw']), 67.5, delta=0.3)
        self.assertAlmostEqual(math.hypot(z['nx'] - a['nx'], z['ny'] - a['ny']), 0.115, delta=0.01)
        self.assertAlmostEqual(math.degrees(z['lyaw'] - a['lyaw']), 61.8, delta=0.3)
        self.assertAlmostEqual(math.hypot(z['lx'] - a['lx'], z['ly'] - a['ly']), 1.77, delta=0.02)

    def test_yaw_unwrap_through_pi(self):
        g = no.SourceGuard(*LIO, lambda q: (no.planar_yaw(q), 1.0), 1.0)
        yaws = []
        for k in range(200):
            yaw = 2.9 + 0.01 * k                                  # crosses +pi at k ~ 24
            self.assertEqual(g.add(100 + 0.01 * k, *LIO, (0, 0, 0, *yaw_q(yaw)), 100 + 0.01 * k), 'ok')
            yaws.append(g.pose[4])
        self.assertTrue(all(abs(b - a - 0.01) < 1e-9 for a, b in zip(yaws, yaws[1:])))
        self.assertAlmostEqual(yaws[-1], 2.9 + 1.99, places=9)

    def test_tilt_projection(self):
        """4 deg of pitch (about the horizontal axis normal to the heading) does not change the projected yaw; 4 deg of
        roll or about a generic horizontal axis changes it < 0.2 deg. Base: a real trial06 pose."""
        q0 = tuple(D['odom'][0][5:9])
        yaw0, fwd = no.imu_footprint_yaw(q0)
        self.assertGreater(fwd, 0.99)
        c, s = math.cos(yaw0), math.sin(yaw0)
        for ang in (math.radians(4), -math.radians(4)):
            # pitch: the forward axis stays in its vertical plane -> heading exact
            self.assertAlmostEqual(no.imu_footprint_yaw(qmul(axis_q((-s, c, 0.0), ang), q0))[0], yaw0, places=9)
            # roll / generic horizontal axis: only the ~5 deg down-look of the LiDAR leaks into the heading
            for axis in ((c, s, 0.0), (1.0, 1.0, 0.0)):
                y = no.imu_footprint_yaw(qmul(axis_q(axis, ang), q0))[0]
                self.assertLess(abs(math.degrees(no.wrap(y - yaw0))), 0.2)

    def test_vertical_forward_axis_rejected(self):
        """A forward axis near vertical has no heading: dropped as 'tilt', not latched."""
        g = no.native_guard()
        q0 = tuple(D['odom'][0][5:9])
        yaw0 = no.imu_footprint_yaw(q0)[0]
        q = qmul(axis_q((-math.sin(yaw0), math.cos(yaw0), 0.0), math.radians(80)), q0)
        self.assertEqual(g.add(10.0, *LEG, (0, 0, 0, *q), 10.0), 'tilt')
        self.assertEqual(g.latched, '')
        self.assertEqual(g.add(10.01, *LEG, (0, 0, 0, *q0), 10.01), 'ok')


class TwistDecimation(unittest.TestCase):
    def test_twist_straight_and_turn(self):
        tw = no.Twist0p3()
        heading = 0.7
        out = [tw.add(0.006 * k, 0.3 * 0.006 * k * math.cos(heading), 0.3 * 0.006 * k * math.sin(heading), heading)
               for k in range(200)]
        self.assertTrue(all(o is None for o in out[:50]))         # < 0.3 s: no velocity yet
        vx, vy, wz = out[-1]
        self.assertAlmostEqual(vx, 0.3, places=9)
        self.assertAlmostEqual(vy, 0.0, places=9)
        self.assertAlmostEqual(wz, 0.0, places=9)
        tw.clear()
        self.assertIsNone(tw.add(5.0, 1, 1, 0))                    # cleared: baseline starts again
        tw = no.Twist0p3()
        for k in range(200):
            out = tw.add(0.006 * k, 1.0, 2.0, 0.5 * 0.006 * k)
        self.assertAlmostEqual(out[2], 0.5, places=9)
        self.assertAlmostEqual(math.hypot(out[0], out[1]), 0.0, places=12)

    def test_decimation_trial06(self):
        """~50 Hz, source stamps only, strictly increasing, no interval beyond period + one source interval."""
        d = no.Decimator(50.0)
        src = [r[1] for r in D['odom']]
        pub = [t for t in src if d.due(t)]
        self.assertTrue(set(pub) <= set(src))
        self.assertTrue(all(b > a for a, b in zip(pub, pub[1:])))
        rate = (len(pub) - 1) / (pub[-1] - pub[0])
        self.assertAlmostEqual(rate, 50.0, delta=1.5)
        gaps = [b - a for a, b in zip(pub, pub[1:])]
        self.assertLessEqual(max(gaps), 0.02 + max(b - a for a, b in zip(src, src[1:])) + 1e-9)

    def test_decimation_restarts_after_gap_and_back_step(self):
        d = no.Decimator(50.0)
        self.assertTrue(d.due(10.0))
        self.assertFalse(d.due(10.006))
        self.assertTrue(d.due(11.0))                               # gap: phase restarts at once
        self.assertTrue(d.due(5.0))                                # stamp back: phase restarts
        self.assertFalse(d.due(5.01))


class Guards(unittest.TestCase):
    def setUp(self):
        self.g = no.native_guard()
        self.q = tuple(D['odom'][0][5:9])
        self.assertEqual(self.g.add(100.0, *LEG, (1.0, 2.0, 0.9, *self.q), 100.0), 'ok')

    def add(self, t, xyz=(1.0, 2.0, 0.9), q=None, frames=LEG, now=None):
        return self.g.add(t, *frames, (*xyz, *(q or self.q)), t if now is None else now)

    def test_nan_latches(self):
        for bad in ((float('nan'), 2.0, 0.9), (1.0, float('inf'), 0.9)):
            g = no.native_guard()
            self.assertEqual(g.add(1.0, *LEG, (*bad, *self.q), 1.0), 'non_finite')
            self.assertEqual(g.add(1.006, *LEG, (1.0, 2.0, 0.9, *self.q), 1.006), 'latched')
            self.assertEqual(g.fault(1.006), 'non_finite')
        self.assertEqual(self.add(float('nan')), 'non_finite')     # NaN stamp

    def test_zero_and_non_unit_quaternion_latch(self):
        self.assertEqual(self.add(100.006, q=(0.0, 0.0, 0.0, 0.0)), 'non_finite')
        self.assertEqual(self.add(100.012), 'latched')
        g = no.native_guard()
        self.assertEqual(g.add(1.0, *LEG, (0, 0, 0, 0.0, 0.0, 0.0, 1.05), 1.0), 'non_finite')

    def test_foreign_frame_dropped_not_latched(self):
        for frames in (('odom', 'lidar_imu_chest_front'), ('leg_odom', 'base_link'), ('map', 'x')):
            self.assertEqual(self.add(100.006, frames=frames), 'foreign_frame')
        self.assertEqual(self.g.counts['foreign_frame'], 3)
        self.assertEqual(self.g.latched, '')
        self.assertEqual(self.add(100.006, frames=('/leg_odom', '/lidar_imu_chest_front')), 'ok')   # leading '/' ok
        lg = no.lio_guard()
        self.assertEqual(lg.add(1.0, 'map', 'base_footprint', (0, 0, 0, 0, 0, 0, 1), 1.0), 'foreign_frame')

    def test_origin_reset_latches(self):
        """0.5 m jump with continuous stamps (O2): latched; 1 m/s walking is not."""
        for k in range(1, 50):
            self.assertEqual(self.add(100.0 + 0.006 * k, xyz=(1.0 + 0.006 * k, 2.0, 0.9)), 'ok')
        self.assertEqual(self.add(100.3, xyz=(1.8, 2.0, 0.9)), 'origin_reset')
        self.assertTrue(self.g.latched.startswith('origin_reset:dp='))
        self.assertEqual(self.add(100.306, xyz=(1.8, 2.0, 0.9)), 'latched')
        self.assertTrue(self.g.fault(100.306).startswith('origin_reset'))

    def test_heading_reset_latches(self):
        q = qmul(axis_q((0, 0, 1), 0.5), self.q)
        self.assertEqual(self.add(100.006, q=q), 'origin_reset')
        self.assertTrue(self.g.latched.startswith('origin_reset:dyaw='))

    def test_jump_bound_not_opened_by_stamp_back(self):
        """After a clock step back dt is clamped: 0.2 m is a reset even though |dt| is large."""
        self.assertEqual(self.add(90.0, xyz=(1.2, 2.0, 0.9)), 'origin_reset')

    def test_stamp_back_accepted_and_reported(self):
        self.assertEqual(self.add(99.0), 'stamp_back')
        self.assertEqual(self.g.counts['stamp_back'], 1)
        self.assertEqual(self.add(99.006), 'ok')

    def test_duplicate_dropped(self):
        self.assertEqual(self.add(100.0), 'duplicate')

    def test_future_dropped_and_stale(self):
        self.assertEqual(self.add(100.5, now=100.006), 'future')
        self.assertEqual(self.g.stamp, 100.0)
        self.assertEqual(self.g.fault(100.15), '')
        self.assertEqual(self.g.fault(100.21), 'stale')            # age > 0.2 s: pause publication
        self.assertEqual(self.add(100.21, now=100.215), 'ok')
        self.assertEqual(self.g.fault(100.215), '')
        self.assertEqual(no.lio_guard().fault(0.0), 'no_data')

    def test_lio_age_limit_is_trust_limit(self):
        lg = no.lio_guard()
        lg.add(10.0, *LIO, (0, 0, 0, 0, 0, 0, 1), 10.0)
        self.assertEqual(lg.fault(10.45), '')
        self.assertEqual(lg.fault(10.51), 'stale')


class CompareMath(unittest.TestCase):
    def test_no_extrapolation_and_coverage(self):
        c = no.Compare()
        for k in range(10):
            c.add_native(10.0 + 0.006 * k, 0.0, 0.0, 0.0)
        self.assertEqual(c.add_lio(10.2, 0, 0, 0), [])              # newer than native: waits
        self.assertEqual(len(c.pending), 1)
        c.add_native(10.3, 0.0, 0.0, 0.0)                           # gap 10.054 -> 10.3 > MAX_INTERP_GAP
        self.assertEqual(c.add_lio(10.25, 0, 0, 0), [])
        self.assertEqual(c.counts['lio_native_gap'], 2)
        self.assertEqual(c.add_lio(9.0, 0, 0, 0), [])               # before native history
        self.assertEqual(c.counts['lio_before_native'], 1)
        rows = c.add_lio(10.03, 0, 0, 0)
        self.assertEqual(len(rows), 1)
        for k in range(60):
            c.add_lio(20.0 + k, 0, 0, 0)
        self.assertLessEqual(len(c.pending), no.MAX_PENDING)
        self.assertGreater(c.counts['lio_wait_overflow'], 0)

    def test_interpolation_linear(self):
        c = no.Compare()
        c.add_native(1.0, 0.0, 0.0, 0.0)
        c.add_native(1.02, 0.02, -0.04, 0.01)
        c.add_native(1.04, 0.04, -0.08, 0.02)
        x, y, yaw = c.interp(1.005)
        self.assertAlmostEqual(x, 0.005)
        self.assertAlmostEqual(y, -0.01)
        self.assertAlmostEqual(yaw, 0.0025)

    def test_residuals_frame_independent(self):
        """Increments only: moving/rotating the LIO world leaves every residual unchanged."""
        a = replay()[0]
        b = replay(lio_xform=(3.0, -7.0, 2.2))[0]
        self.assertEqual(len(a), len(b))
        for ra, rb in zip(a, b):
            for k in ('res1_d', 'res1_yaw', 'res10_d', 'res10_yaw', 'lio_speed'):
                if ra[k] is None:
                    self.assertIsNone(rb[k])
                else:
                    self.assertAlmostEqual(ra[k], rb[k], places=6)

    def test_trial06_residuals(self):
        rows, ng, lg, c = replay()
        self.assertEqual(c.counts['pairs'], len(D['lio']))         # every LIO sample matched, none extrapolated
        pre = [r for r in rows if r['t'] < ON and r['res1_d'] is not None]
        self.assertGreater(len(pre), 50)
        self.assertLess(max(r['res1_d'] for r in pre), 0.06)        # standing: LIO sway only
        self.assertLess(max(abs(r['res1_yaw']) for r in pre), 0.01)
        self.assertGreater(rows[-1]['res10_d'], 1.5)                # the standing drift after the turn
        s = c.summary()
        self.assertGreater(s['cumulative']['lio_path'], 5 * s['cumulative']['native_path'])
        self.assertEqual(s['residuals']['1s']['n'], sum(1 for r in rows if r['res1_d'] is not None))

    def test_still_noise(self):
        c = no.Compare()
        for k in range(1000):
            t = 10 + 0.006 * k
            c.add_native(t, 0.001 * math.sin(k), 0.0, 0.0)
        for k in range(50):
            c.add_lio(10.0 + 0.1 * k, 0.01 * (k % 2), 0.0, 0.0)
        n = c.still_noise(14.9)
        self.assertAlmostEqual(n['lio_std_x'], 0.005, places=3)
        self.assertLess(n['native_std_x'], 0.001)
        self.assertIsNone(no.Compare().still_noise(0.0))


class Plausibility(unittest.TestCase):
    """LIO planar speed > 0.05 m/s over 1 s while native is standing (<= 1 cm / 0.02 rad), held 0.3 s."""

    @classmethod
    def setUpClass(cls):
        cls.rows, _, _, cls.c = replay()

    def test_fires_within_2s_after_stop(self):
        first = next(r['t'] for r in self.rows if r['lio_implausible'])
        self.assertGreater(first, OFF)
        self.assertLess(first - OFF, 2.0)
        self.assertEqual(self.c.summary()['plausibility']['first_rise'], first)
        self.assertTrue(self.rows[-1]['lio_implausible'])           # still drifting at the end of the record

    def test_quiet_before_command(self):
        pre = [r for r in self.rows if r['t'] < ON]
        self.assertGreater(sum(1 for r in pre if r['native_still']), 50)   # the check really ran
        self.assertFalse(any(r['plaus_cond'] or r['lio_implausible'] for r in pre))
        self.assertLess(max(r['lio_speed'] for r in pre if r['lio_speed'] is not None), no.PLAUS_LIO_SPEED)

    def test_never_while_native_moves(self):
        self.assertFalse(any(r['lio_implausible'] for r in self.rows if r['native_still'] is not True))
        turn = [r for r in self.rows if ON + 0.5 < r['t'] < ON + 6.5]
        self.assertTrue(turn and not any(r['native_still'] for r in turn))

    def test_hold_filters_a_short_burst(self):
        c = no.Compare(hold_s=0.3)
        rows = []
        for k in range(0, 1001):
            c.add_native(10 + 0.006 * k, 0.0, 0.0, 0.0)
        for k in range(0, 55):
            t = 10.0 + 0.1 * k
            x = 0.0 if not 20 <= k <= 22 else 0.07 * (k - 19)      # 0.2 s of fake LIO motion
            rows += c.add_lio(t, x, 0.0, 0.0)
        self.assertTrue(any(r['plaus_cond'] for r in rows))
        self.assertFalse(any(r['lio_implausible'] for r in rows))


class Node(unittest.TestCase):
    """The ROS node in-process (domain 88, localhost): only /qazbot_native/compare/* publishers, no TF; callbacks fed
    with trial06 messages; guards stop the odom output."""

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy
            from nav_msgs.msg import Odometry
        except ImportError as e:
            raise unittest.SkipTest(str(e))
        os.environ['ROS_DOMAIN_ID'] = os.environ.get('QAZBOT_TEST_DOMAIN', '88')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        cls.tmp = tempfile.TemporaryDirectory()
        cls.rclpy, cls.Odometry = rclpy, Odometry
        rclpy.init(args=['--ros-args', '-p', 'log_dir:=' + cls.tmp.name, '-p', 'use_sim_time:=true'])
        import native_odom_compare as nc
        cls.nc = nc

    @classmethod
    def tearDownClass(cls):
        cls.rclpy.try_shutdown()
        cls.tmp.cleanup()

    def setUp(self):
        self.node = self.nc.NativeOdomCompare()
        self.out = {k: [] for k in ('odom', 'plaus', 'status')}
        for k in self.out:
            getattr(self.node, 'pub_' + k).publish = self.out[k].append
        self.clock = 0.0
        self.node.now = lambda: self.clock

    def tearDown(self):
        self.node.close()
        self.node.destroy_node()

    def msg(self, stamp, frames, t7, cov=None):
        m = self.Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = int(stamp), int(round((stamp - int(stamp)) * 1e9))
        m.header.frame_id, m.child_frame_id = frames
        p, q = m.pose.pose.position, m.pose.pose.orientation
        p.x, p.y, p.z, q.x, q.y, q.z, q.w = (float(v) for v in t7)
        if cov:
            m.pose.covariance = cov
        return m

    def feed(self, until=None):
        ev = sorted([(r[0], 'n', r) for r in D['odom']] + [(r[0], 'l', r) for r in D['lio']], key=lambda e: e[0])
        for recv, k, r in ev:
            if until is not None and recv > until:
                break
            self.clock = recv
            if k == 'n':
                self.node.on_leg(self.msg(r[1], LEG, r[2:9], [7.1e-4 if i in (0, 7) else 0.0 for i in range(36)]))
            else:
                self.node.on_lio(self.msg(r[1], LIO, lio_t7(r)))

    def test_only_compare_topics_no_tf(self):
        topics = {n for n, _ in self.node.get_publisher_names_and_types_by_node(self.node.get_name(),
                                                                               self.node.get_namespace())}
        ours = {t for t in topics if t not in ('/rosout', '/parameter_events')}
        self.assertEqual(ours, {'/qazbot_native/compare/odom_imu', '/qazbot_native/compare/lio_plausibility',
                                '/qazbot_native/compare/status'})
        self.assertFalse(any('tf' in t.split('/')[-1] for t in ours))

    def test_trial06_end_to_end(self):
        self.feed()
        odom, plaus = self.out['odom'], self.out['plaus']
        st = [o.header.stamp.sec + o.header.stamp.nanosec * 1e-9 for o in odom]
        self.assertTrue(all(b > a for a, b in zip(st, st[1:])))
        # ~50 Hz; 6 % of the recorded leg odometry arrived > 0.2 s late (recorder bursts) and is withheld as stale
        self.assertGreater((len(st) - 1) / (st[-1] - st[0]), 45.0)
        self.assertLess((len(st) - 1) / (st[-1] - st[0]), 51.0)
        src = {round(r[1], 6): r[0] - r[1] for r in D['odom']}
        self.assertTrue(all(round(s, 6) in src for s in st))       # source stamps, never re-stamped
        self.assertTrue(all(src[round(s, 6)] <= no.NATIVE_MAX_AGE for s in st))   # never published stale
        o = odom[-1]
        self.assertEqual((o.header.frame_id, o.child_frame_id), ('native_odom', 'native_imu_footprint'))
        self.assertAlmostEqual(o.pose.covariance[0], 7.1e-4)
        self.assertEqual(len(plaus), len(D['lio']))
        first = next(json.loads(p.data)['t'] for p in plaus if json.loads(p.data)['lio_implausible'])
        self.assertLess(first - OFF, 2.0)
        self.assertTrue(all(json.loads(p.data)['diagnostic_only'] for p in plaus))
        self.node.on_status()
        s = json.loads(self.out['status'][-1].data)
        self.assertEqual(s['mode'], 'compare')
        self.assertFalse(s['publish_tf'])
        self.assertEqual(s['messages']['rows'], len(D['lio']))
        self.assertGreater(s['residuals']['10s']['max_d'], 1.5)

    def test_guards_stop_output(self):
        self.feed(until=ON - 3.0)
        n0 = len(self.out['odom'])
        self.assertGreater(n0, 100)
        last = D['odom'][[r[0] for r in D['odom']].index(max(r[0] for r in D['odom'] if r[0] <= ON - 3.0))]
        t, q = last[1], last[5:9]
        # foreign frame: dropped, nothing published
        self.clock = t + 0.03
        self.node.on_leg(self.msg(t + 0.03, ('odom', 'lidar_imu_chest_front'), (*last[2:5], *q)))
        self.assertEqual(len(self.out['odom']), n0)
        # stale (age 0.3 s > 0.2 s): accepted for history but not published
        self.clock = t + 0.36
        self.node.on_leg(self.msg(t + 0.06, LEG, (*last[2:5], *q)))
        self.assertEqual(len(self.out['odom']), n0)
        # origin reset: latched, nothing more even with good samples
        self.clock = t + 0.1
        self.node.on_leg(self.msg(t + 0.09, LEG, (last[2] + 0.5, last[3], last[4], *q)))
        for k in range(1, 20):
            self.clock = t + 0.1 + 0.006 * k
            self.node.on_leg(self.msg(t + 0.1 + 0.006 * k, LEG, (*last[2:5], *q)))
        self.assertEqual(len(self.out['odom']), n0)
        self.assertTrue(self.node.native.fault(self.clock).startswith('origin_reset'))

    def test_nan_latches_output(self):
        self.feed(until=ON - 3.0)
        n0 = len(self.out['odom'])
        r = D['odom'][-1]
        self.clock = r[1] + 100.0
        self.node.on_leg(self.msg(self.clock, LEG, (float('nan'), *r[3:9])))
        for k in range(1, 30):
            self.node.on_leg(self.msg(self.clock + 0.006 * k, LEG, r[2:9]))
        self.assertEqual(len(self.out['odom']), n0)
        self.assertEqual(self.node.native.fault(self.clock), 'non_finite')

    def test_stamp_back_clears(self):
        self.feed(until=ON - 3.0)
        self.assertTrue(self.node.cmp.pairs)
        r = D['odom'][0]
        self.clock = r[1]
        self.node.on_leg(self.msg(r[1] - 5.0, LEG, r[2:9]))
        self.assertEqual(len(self.node.cmp.pairs), 0)
        self.assertEqual(len(self.node.twist.track), 1)
        self.assertEqual(self.node.native.counts['stamp_back'], 1)


if __name__ == '__main__':
    unittest.main()
