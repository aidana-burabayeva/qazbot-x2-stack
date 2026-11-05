#!/usr/bin/env python3
"""Offline tests of the step_cmd.py native-base stop limiter (--limit-source native_base) on trial06 data.

No robot, no MC, no hardware DDS. Data: tests/s3/data/trial06_native_series.json.gz, extracted read-only from
logs/mc_execution23/trial06_repeat_left05_20s_joints/bag by tests/s3/extract_native_series.py (leg odometry, the
dynamic vendor TF of the base_link -> lidar_imu_chest_front chain, /tf_static, LIO odom_level, cmd_vel_authorized).
Independent reference: kinematic_series.json native_base of the same trial (prior offline audit, LIO sample times).

  NativeLimiterReplay  NativeBase alone, replayed at 20 Hz ticks in bag receive time (fast, no ROS graph).
  ToolReplay           the REAL step_cmd.Tool.run() loop in real time; rclpy node in an isolated domain (88,
                       localhost); only spin(), the adapter/trust/coordinator inputs, the graph queries and the
                       Trigger clients are replaced; recorded messages enter through the Tool's own callbacks.

Run: bash -c 'source ws/src/navigation/env.sh && python3 -m unittest -v tests/s3/test_native_limiter.py'
"""
import bisect
import contextlib
import csv
import io
import gzip
import json
import math
import os
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import step_cmd  # noqa: E402

DATA = HERE / 'data' / 'trial06_native_series.json.gz'
TRIAL = ROOT / 'logs/mc_execution23/trial06_repeat_left05_20s_joints'
TICK = 1.0 / step_cmd.RATE_HZ
D = json.load(gzip.open(DATA, 'rt'))
ON = next(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)                 # first nonzero command (receive time)
OFF = max(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)
END = D['odom'][-1][0]
EDGES = ('pelvis->waist_yaw_link', 'waist_yaw_link->waist_pitch_link', 'waist_pitch_link->torso_link')


def corrupt(t7, field, value):
    t7 = list(t7)
    if field == 'zero_q':
        t7[3:7] = [0.0, 0.0, 0.0, 0.0]
    else:
        t7[field] = value
    return t7


def events(drop_odom_after=None, drop_tf_after=None, tf_delay=0.0, tf_delay_after=None,
           bad_odom_at=None, bad_tf_at=None, bad_lio_at=None, field=6, value=float('nan')):
    """[(recv, kind, payload)] sorted by receive time; optional faults injected in receive time. bad_*_at: the first
    sample received after that time gets t7[field] = value (field 0 = x, 6 = qw; 'zero_q' = zero quaternion)."""
    ev, bad = [], dict(odom=bad_odom_at, tf=bad_tf_at, lio=bad_lio_at)

    def hit(kind, recv):
        if bad[kind] is not None and recv > bad[kind]:
            bad[kind] = None
            return True
        return False
    for r in D['odom']:
        if drop_odom_after is None or r[0] <= drop_odom_after:
            ev.append((r[0], 'odom', (r[1], corrupt(r[2:9], field, value) if hit('odom', r[0]) else r[2:9])))
    for rows in zip(*(D['tf'][e] for e in EDGES)):
        recv = rows[0][0]
        if drop_tf_after is not None and recv > drop_tf_after:
            continue
        if tf_delay_after is not None and recv > tf_delay_after:
            recv += tf_delay
        b = hit('tf', recv)
        ev.append((recv, 'tf', [(e.split('->')[0], e.split('->')[1], r[1], corrupt(r[2:9], field, value) if b and k == 1
                                 else r[2:9]) for k, (e, r) in enumerate(zip(EDGES, rows))]))
    for r in D['lio']:
        ev.append((r[0], 'lio', [r[1], float('nan'), r[3], r[4]] if hit('lio', r[0]) else r[1:]))
    ev.sort(key=lambda x: x[0])
    return ev


def new_native(max_age=step_cmd.NATIVE_MAX_AGE):
    nb = step_cmd.NativeBase(max_age)
    for parent, child, *t7 in D['tf_static'].values():
        nb.add_static(parent, child, t7)
    return nb


def replay(nb, ev, t_from, t_to, start_at=ON - 0.5):
    """20 Hz ticks in receive time; yields (t, fault, dist, dyaw) after `start_at` (start pose taken there)."""
    i, start, t = 0, None, t_from
    while t <= t_to:
        while i < len(ev) and ev[i][0] <= t:
            _, kind, p = ev[i]
            if kind == 'odom':
                nb.add_odom(p[0], p[1])
            elif kind == 'tf':
                for parent, child, stamp, t7 in p:
                    nb.add_tf(parent, child, stamp, t7)
            i += 1
        nb.update()
        why = nb.fault(t)
        if start is None and t >= start_at and not why:
            start = nb.pose
        if start is not None:
            d, dyaw = nb.displacement(start)
            yield t, why, d, dyaw
        t += TICK


def audit_native_base():
    """Prior offline audit (kinematic_series.json native_base): [(t, x, y, yaw_unwrapped)]."""
    with open(TRIAL / 'kinematic_series.json') as f:
        rows = json.load(f)['native_base']
    out, prev = [], None
    for r in rows:
        yaw = r[4] if prev is None else prev + math.atan2(math.sin(r[4] - prev), math.cos(r[4] - prev))
        out.append((r[0], r[1], r[2], yaw))
        prev = yaw
    return out


def audit_crossing(deg, start_t=ON - 0.5):
    """Bag time at which the audit's base yaw change from start_t first reaches `deg` (linear interpolation)."""
    a = audit_native_base()
    k = bisect.bisect_right([r[0] for r in a], start_t) - 1
    y0 = a[k][3]
    for p, q in zip(a[k:], a[k + 1:]):
        dp, dq = math.degrees(p[3] - y0), math.degrees(q[3] - y0)
        if dq >= deg > dp:
            return p[0] + (deg - dp) / (dq - dp) * (q[0] - p[0])
    return None


class NativeLimiterReplay(unittest.TestCase):
    def test_fixture_matches_trial(self):
        self.assertEqual(len(D['odom']), 3598)                      # kinematics.json message_counts
        self.assertTrue(all(len(D['tf'][e]) == 1009 for e in EDGES))
        self.assertAlmostEqual(OFF - ON, 5.80, delta=0.1)           # 5.866 s step, receive-time span of 117 commands

    def test_composition_pure_rotation_keeps_base_fixed(self):
        """Base turning in place with the IMU 0.3 m ahead: IMU travels on a circle, base_link stays put."""
        nb = step_cmd.NativeBase(0.2)
        nb.add_static('base_link', 'pelvis', (0, 0, 0, 0, 0, 0, 1))
        nb.add_static('torso_link', 'lidar_chest_front', (0.3, 0, 0.4, 0, 0, 0, 1))
        nb.add_static('lidar_chest_front', 'lidar_imu_chest_front', (0, 0, 0, -0.7071068, 0, 0, 0.7071068))
        q_bi = (-0.7071068, 0, 0, 0.7071068)
        for k in range(20):
            s = 100.0 + 0.05 * k
            for parent, child in (('pelvis', 'waist_yaw_link'), ('waist_yaw_link', 'waist_pitch_link'),
                                  ('waist_pitch_link', 'torso_link')):
                nb.add_tf(parent, child, s, (0, 0, 0, 0, 0, 0, 1))
            yaw = 0.05 * k
            q_ob = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
            q_oi = step_cmd.qmul(q_ob, q_bi)
            px, py, pz = step_cmd.qrot(q_ob, (0.3, 0, 0.4))
            nb.add_odom(s, (1.0 + px, 2.0 + py, pz, *q_oi))
            p = nb.update()
            self.assertAlmostEqual(p[1], 1.0, places=5)
            self.assertAlmostEqual(p[2], 2.0, places=5)
            self.assertAlmostEqual(p[3], yaw, places=5)
        self.assertEqual(nb.fault(100.0 + 0.05 * 19 + 0.1), '')

    def test_trial06_native_would_not_trip_30cm(self):
        ticks = list(replay(new_native(), events(), ON - 3.0, END))
        dmax = max(d for _, why, d, _ in ticks if d is not None)
        self.assertLess(dmax, 0.30)
        self.assertLess(dmax, 0.06)                                 # audit: max 0.0319 m (pelvis)
        lio_d = max(math.hypot(r[2] - D['lio'][0][2], r[3] - D['lio'][0][3]) for r in D['lio'])
        self.assertGreater(lio_d, 0.30)                             # the LIO estimate that tripped trial06

    def test_trial06_native_final_yaw_matches_audit(self):
        ticks = list(replay(new_native(), events(), ON - 3.0, END))
        final = ticks[-1][3]
        peak = max(dyaw for _, _, _, dyaw in ticks)
        a = audit_native_base()
        k = bisect.bisect_right([r[0] for r in a], ON - 0.5) - 1
        self.assertAlmostEqual(final, math.degrees(a[-1][3] - a[k][3]), delta=1.0)   # audit 67.4 deg
        self.assertLess(peak, 90.0)                                  # trial06 yaw limit 90 deg: never reached

    def test_trial06_native_tracks_audit_series(self):
        nb = new_native()
        for _ in replay(nb, events(), ON - 3.0, END):              # fills nb.rows (every evaluated sample)
            pass
        rows = {round(r[0], 3): r for r in nb.rows}
        stamps = sorted(rows)
        a = audit_native_base()
        k = bisect.bisect_right([r[0] for r in a], ON - 0.5) - 1
        s0 = rows[stamps[bisect.bisect_right(stamps, a[k][0]) - 1]]
        diffs = []
        for t, x, y, yaw in a[k:]:
            j = bisect.bisect_right(stamps, t) - 1
            if j < 0 or t - stamps[j] > 0.03:
                continue
            r = rows[stamps[j]]
            diffs.append(abs(math.degrees((r[3] - s0[3]) - (yaw - a[k][3]))))
        diffs.sort()
        self.assertGreater(len(diffs), 100)
        self.assertLess(diffs[int(0.95 * len(diffs))], 3.0)         # 30 ms stamp mismatch at <= 0.5 rad/s ~ 0.9 deg

    def test_trial06_yaw_threshold_trip_time(self):
        for deg in (15.0, 30.0, 60.0):
            expect = audit_crossing(deg)
            self.assertIsNotNone(expect)
            trip = next(t for t, why, d, dyaw in replay(new_native(), events(), ON - 3.0, END)
                        if not why and dyaw is not None and abs(dyaw) >= deg)
            with self.subTest(deg=deg, trip_after_on=trip - ON, expected_after_on=expect - ON):
                self.assertGreaterEqual(trip, expect - 0.10)
                self.assertLessEqual(trip, expect + 0.20)            # 20 Hz tick + odom/TF latency

    def test_no_false_stale_in_command_window(self):
        bad = [(round(t - ON, 2), why) for t, why, _, _ in replay(new_native(), events(), ON - 3.0, OFF + 3.0)
               if why and t >= ON - 1.0]
        self.assertEqual(bad, [])

    def test_odom_dropout_aborts(self):
        cut = ON + 1.0
        first = next((t, why) for t, why, _, _ in replay(new_native(), events(drop_odom_after=cut), ON - 3.0, END)
                     if why and t > ON)
        self.assertIn(first[1], ('native_odom_stale', 'native_base_stale'))
        self.assertLessEqual(first[0] - cut, 0.2 + 0.05 + 0.03)      # max_age + tick + last-sample age

    def test_tf_dropout_aborts(self):
        cut = ON + 1.0
        first = next((t, why) for t, why, _, _ in replay(new_native(), events(drop_tf_after=cut), ON - 3.0, END)
                     if why and t > ON)
        self.assertEqual(first[1], 'native_tf_stale')
        self.assertLessEqual(first[0] - cut, 0.2 + 0.05 + 0.03)

    def test_tf_delay_beyond_max_age_aborts(self):
        cut = ON + 1.0
        faults = [why for t, why, _, _ in replay(new_native(), events(tf_delay=0.35, tf_delay_after=cut), ON - 3.0, END)
                  if why and t > ON]
        self.assertTrue(faults and faults[0] == 'native_tf_stale', faults[:3])

    def first_fault_after(self, ev, t0):
        out = [(t, why) for t, why, _, _ in replay(new_native(), ev, ON - 3.0, END) if t > t0]
        first = next((x for x in out if x[1]), None)
        return first, out

    def test_nan_odom_qw_latches_non_finite(self):
        cut = ON + 1.0
        first, out = self.first_fault_after(events(bad_odom_at=cut, field=6), ON)
        self.assertEqual(first[1], 'native_non_finite:odom')
        self.assertLessEqual(first[0] - cut, TICK + 1e-6)
        self.assertTrue(all(why == 'native_non_finite:odom' for t, why in out if t >= first[0]))   # never clears

    def test_nan_odom_x_latches_non_finite(self):
        first, out = self.first_fault_after(events(bad_odom_at=ON + 1.0, field=0), ON)
        self.assertEqual(first[1], 'native_non_finite:odom')

    def test_zero_quaternion_odom_rejected(self):
        first, out = self.first_fault_after(events(bad_odom_at=ON + 1.0, field='zero_q'), ON)
        self.assertEqual(first[1], 'native_non_finite:odom')

    def test_nan_tf_rejected_and_edge_stamp_not_advanced(self):
        nb = new_native()
        e = EDGES[1]
        r0, r1 = D['tf'][e][0], D['tf'][e][1]
        for k, edge in enumerate(EDGES):
            nb.add_tf(*edge.split('->'), r0[1], D['tf'][edge][0][2:9])
        before = nb.tf_edge['waist_pitch_link']
        nb.add_tf(*e.split('->'), r1[1], corrupt(r1[2:9], 6, float('nan')))
        self.assertEqual(nb.tf_edge['waist_pitch_link'], before)
        self.assertEqual(nb.fault(r1[0]), 'native_non_finite:tf waist_yaw_link->waist_pitch_link')
        first, _ = self.first_fault_after(events(bad_tf_at=ON + 1.0, field=0), ON)
        self.assertEqual(first[1], 'native_non_finite:tf waist_yaw_link->waist_pitch_link')

    def test_non_finite_start_pose(self):
        nb = new_native()
        self.assertEqual(nb.fault(ON, (ON, float('nan'), 0.0, 0.0)), 'native_non_finite:start')

    def test_no_data_and_future_stamp(self):
        nb = new_native()
        self.assertEqual(nb.fault(ON), 'native_odom_stale')
        nb.add_odom(ON + 1.0, D['odom'][0][2:9])
        for e in EDGES:
            nb.add_tf(*e.split('->'), ON + 1.0, D['tf'][e][0][2:9])
        self.assertEqual(nb.fault(ON), 'native_stamp_future')

    def test_parse_defaults_unchanged(self):
        base = ['--rot', '0.5', '--expect-domain', '0', '--expect-mode', 'live', '--csv', '/nonexistent/x.csv']
        a = step_cmd.parse(base)
        self.assertEqual((a.limit_source, a.odom_topic), ('lio', '/qazbot_lio/odom_level'))
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            step_cmd.parse(base + ['--limit-source', 'native_base'])                    # needs a limit
        a = step_cmd.parse(base + ['--limit-source', 'native_base', '--stop-after-yaw-deg', '30'])
        self.assertEqual((a.native_odom_topic, a.native_max_age), ('/aima/mc/leg_odometry', 0.2))
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            step_cmd.parse(base + ['--limit-source', 'native_base', '--stop-after-dist', '0.3', '--native-max-age', '2'])


# ---------------------------------------------------------------------------------------------------- Tool loop
class Future:
    def __init__(self, r, ready_at=0.0):
        self.r, self.ready_at = r, ready_at

    def done(self):
        return time.time() >= self.ready_at

    def result(self):
        return self.r


class FakeClient:
    def __init__(self, name, calls, Trigger, delay=0.0, hook=None):
        self.name, self.calls, self.Trigger, self.delay, self.hook = name, calls, Trigger, delay, hook

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.calls.append((time.time(), self.name))
        if self.hook:
            self.hook()
        return Future(self.Trigger.Response(success=True, message='fake'), time.time() + self.delay)


class ToolReplay(unittest.TestCase):
    """REAL Tool.run(); replay starts at bag time ON - LEAD so the step (after ~1.1 s of preconditions/arm) begins
    about 0.5 s before the recorded command; the base is still standing then, as in trial06."""
    LEAD = 1.6

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_DOMAIN_ID'], os.environ['ROS_LOCALHOST_ONLY'] = '88', '1'
        os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
        import rclpy
        cls.rclpy = rclpy
        rclpy.init()
        cls.tmp = tempfile.mkdtemp(prefix='native_limiter_')

    @classmethod
    def tearDownClass(cls):
        cls.rclpy.try_shutdown()

    def run_tool(self, label, extra, ev, arm_delay=0.0, cut_odom_on_arm=False):
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        csvp = os.path.join(self.tmp, label + '.csv')
        a = step_cmd.parse(['--rot', '0.5', '--expect-domain', '88', '--expect-mode', 'dry_run', '--ns', '/qazbot_native_test',
                            '--csv', csvp] + extra)
        tool = step_cmd.Tool(a)
        calls = []
        st = dict(i=0, bag_at_limit=None, cut=False)

        def on_arm():
            st['cut'] = cut_odom_on_arm
        tool.srv = {k: FakeClient(k, calls, tool.Trigger, arm_delay if k == 'motion_adapter/arm_motion' else 0.0,
                                  on_arm if k == 'motion_adapter/arm_motion' else None) for k in tool.srv}
        tool.n.get_publishers_info_by_topic = lambda topic: []
        tool.n.get_subscriptions_info_by_topic = lambda topic: [types.SimpleNamespace(node_name='motion_adapter')]
        shift = time.time() - (ON - self.LEAD)                      # wall = bag + shift (stamps keep their age)

        def stamp_msg(h, s):
            s += shift
            h.stamp.sec, h.stamp.nanosec = int(s), int(round((s - int(s)) * 1e9)) % 1000000000

        if tool.native is not None:
            m = TFMessage()
            for parent, child, *t7 in D['tf_static'].values():
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id = parent, child
                (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x,
                 t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = t7
                m.transforms.append(t)
            tool._on_tf_static(m)

        def spin(timeout=0.0):
            time.sleep(min(timeout, 0.01))
            bag_now = time.time() - shift
            while st['i'] < len(ev) and ev[st['i']][0] <= bag_now:
                _, kind, p = ev[st['i']]
                st['i'] += 1
                if kind == 'odom' and st['cut']:
                    continue
                if kind == 'odom' and tool.native is not None:
                    o = Odometry()
                    stamp_msg(o.header, p[0])
                    o.header.frame_id, o.child_frame_id = 'leg_odom', step_cmd.NATIVE_IMU
                    pp, q = o.pose.pose.position, o.pose.pose.orientation
                    pp.x, pp.y, pp.z, q.x, q.y, q.z, q.w = p[1]
                    tool._on_native_odom(o)
                elif kind == 'tf' and tool.native is not None:
                    m = TFMessage()
                    for parent, child, s, t7 in p:
                        t = TransformStamped()
                        t.header.frame_id, t.child_frame_id = parent, child
                        stamp_msg(t.header, s)
                        (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                         t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = t7
                        m.transforms.append(t)
                    tool._on_tf(m)
                elif kind == 'lio':
                    o = Odometry()
                    stamp_msg(o.header, p[0])
                    o.pose.pose.position.x, o.pose.pose.position.y = p[1], p[2]
                    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(p[3] / 2), math.cos(p[3] / 2)
                    tool._on_odom(o)
            now = time.monotonic()
            tool.adapter = dict(state='idle', mode='dry_run', live=False, registration='dry_run', ready=True,
                                motion_armed=True, reason='', mc_source='rc')
            tool.adapter_at = now
            tool.coord, tool.coord_at = dict(busy=False, latched=True, fault='operator_stop'), now
            tool.trust, tool.trust_at, tool.trust_seq = True, now, tool.trust_seq + 1
            if tool.limit_hit and st['bag_at_limit'] is None:
                st['bag_at_limit'] = bag_now
        tool.spin = spin
        code = step_cmd.EXIT_ABORT
        try:
            code = tool.run()
        finally:
            tool.close(code)
            tool.n.destroy_node()
        with open(csvp) as f:
            rows = list(csv.DictReader(f))
        with open(csvp + '.summary.json') as f:
            summ = json.load(f)
        step_rows = [r for r in rows if r['phase'] == 'step']
        bag_step_on = float(step_rows[0]['t_unix']) - shift if step_rows else None
        print(json.dumps(dict(case=label, exit=code, abort=summ['abort_reason'], limit=summ['limit_hit'],
                              step_on_after_ON=None if bag_step_on is None else round(bag_step_on - ON, 3),
                              last_step_after_ON=round(max(float(r['t_unix']) for r in step_rows) - shift - ON, 3)
                              if step_rows else None, native=summ.get('native'))), file=sys.__stdout__, flush=True)
        return code, rows, summ, calls, bag_step_on, shift

    def assert_zero_after_steps(self, rows):
        last = max(i for i, r in enumerate(rows) if r['phase'] == 'step')
        tail = rows[last + 1:]
        self.assertTrue(tail and all(float(r['vx_cmd']) == 0 and float(r['wz_cmd']) == 0 for r in tail))
        self.assertGreaterEqual(float(tail[-1]['t_rel']) - float(tail[0]['t_rel']), 0.9)   # >= --pause of zeros

    def test_tool_native_30cm_not_tripped(self):
        dur = (OFF - ON) + 3.5                                       # recorded command plus 3 s of coast
        code, rows, summ, calls, on, _ = self.run_tool('native_dist', ['--rot-dur', f'{dur:.2f}', '--stop-after-dist', '0.3',
                                                                        '--limit-source', 'native_base'], events())
        self.assertEqual(code, step_cmd.EXIT_DONE, (summ['abort_reason'], summ['limit_hit']))
        self.assertLess(max(float(r['native_dist_m']) for r in rows if r['native_dist_m']), 0.06)
        self.assertGreater(max(float(r['dist_m']) for r in rows if r['dist_m']), 0.30)   # LIO logged, not used
        self.assertGreater(summ['native']['final_dyaw_deg'], 55)
        self.assertEqual([c[1] for c in calls][-2:], ['motion_adapter/stop', 'motion_adapter/resume'])
        self.assertTrue(os.path.getsize(os.path.join(self.tmp, 'native_dist.csv.native.csv')) > 1000)

    def test_tool_native_yaw_limit_time(self):
        deg = 30.0
        code, rows, summ, calls, on, shift = self.run_tool('native_yaw', ['--rot-dur', '20', '--stop-after-yaw-deg', f'{deg}',
                                                                         '--stop-after-dist', '0.3', '--limit-source',
                                                                         'native_base'], events())
        self.assertEqual(code, step_cmd.EXIT_LIMIT, summ['abort_reason'])
        self.assertTrue(summ['limit_hit'].startswith('yaw') and summ['limit_hit'].endswith('(native_base)'), summ['limit_hit'])
        last_step = max(float(r['t_unix']) for r in rows if r['phase'] == 'step') - shift
        expect = audit_crossing(deg, on)
        self.assertGreaterEqual(last_step, expect - 0.15, (last_step - ON, expect - ON))
        self.assertLessEqual(last_step, expect + 0.25, (last_step - ON, expect - ON))
        self.assert_zero_after_steps(rows)

    def test_tool_native_odom_dropout_aborts_with_zeros(self):
        cut = ON + 1.0
        code, rows, summ, calls, on, shift = self.run_tool('native_odom_cut', ['--rot-dur', '20', '--stop-after-dist', '0.3',
                                                                              '--limit-source', 'native_base'],
                                                           events(drop_odom_after=cut))
        self.assertEqual(code, step_cmd.EXIT_ABORT)
        self.assertIn(summ['abort_reason'], ('native_odom_stale', 'native_base_stale'))
        last_step = max(float(r['t_unix']) for r in rows if r['phase'] == 'step') - shift
        self.assertLessEqual(last_step - cut, 0.2 + 0.1)
        self.assert_zero_after_steps(rows)
        self.assertEqual([c[1] for c in calls][-1], 'motion_adapter/stop')     # abort: stop, no resume

    def test_tool_native_tf_dropout_aborts_with_zeros(self):
        cut = ON + 1.0
        code, rows, summ, calls, on, shift = self.run_tool('native_tf_cut', ['--rot-dur', '20', '--stop-after-yaw-deg', '90',
                                                                            '--limit-source', 'native_base'],
                                                           events(drop_tf_after=cut))
        self.assertEqual(code, step_cmd.EXIT_ABORT)
        self.assertEqual(summ['abort_reason'], 'native_tf_stale')
        last_step = max(float(r['t_unix']) for r in rows if r['phase'] == 'step') - shift
        self.assertLessEqual(last_step - cut, 0.2 + 0.1)
        self.assert_zero_after_steps(rows)

    def test_tool_native_arm_phase_abort(self):
        """Leg odometry stops while the arm service call is pending (1 s): abort before any nonzero command."""
        code, rows, summ, calls, on, shift = self.run_tool('native_arm_cut', ['--rot-dur', '20', '--stop-after-dist', '0.3',
                                                                             '--limit-source', 'native_base'], events(),
                                                           arm_delay=1.0, cut_odom_on_arm=True)
        self.assertEqual(code, step_cmd.EXIT_ABORT)
        self.assertIn(summ['abort_reason'], ('native_odom_stale', 'native_base_stale'))
        self.assertFalse(any(r['phase'] == 'step' for r in rows))
        self.assertTrue(all(float(r['vx_cmd']) == 0 and float(r['wz_cmd']) == 0 for r in rows))
        self.assertGreaterEqual(sum(r['phase'] == 'end_zeros' for r in rows), 19)
        self.assertEqual([c[1] for c in calls][-1], 'motion_adapter/stop')

    def test_tool_native_nan_qw_aborts_with_zeros(self):
        cut = ON + 1.0
        code, rows, summ, calls, on, shift = self.run_tool('native_nan', ['--rot-dur', '20', '--stop-after-yaw-deg', '90',
                                                                         '--limit-source', 'native_base'],
                                                           events(bad_odom_at=cut, field=6))
        self.assertEqual(code, step_cmd.EXIT_ABORT)
        self.assertEqual(summ['abort_reason'], 'native_non_finite:odom')
        self.assertEqual(summ['native']['non_finite'], 'odom')
        last_step = max(float(r['t_unix']) for r in rows if r['phase'] == 'step') - shift
        self.assertLessEqual(last_step - cut, 0.1)
        self.assert_zero_after_steps(rows)
        self.assertEqual([c[1] for c in calls][-1], 'motion_adapter/stop')

    def test_tool_default_lio_nan_sample_dropped(self):
        """Default LIO path: one NaN LIO sample is dropped (no unwrap poisoning); the 0.3 m limit still trips."""
        code, rows, summ, calls, on, shift = self.run_tool('lio_nan', ['--rot-dur', '20', '--stop-after-dist', '0.3'],
                                                           events(bad_lio_at=ON + 1.0))
        self.assertEqual(code, step_cmd.EXIT_LIMIT, summ['abort_reason'])
        self.assertEqual(summ.get('odom_non_finite_dropped'), 1)
        self.assertFalse(any(r['dist_m'] in ('nan', 'NaN') or r['dyaw_deg'] in ('nan', 'NaN') for r in rows))

    def test_tool_default_lio_reproduces_trial06_trip(self):
        """Default (no new flag): LIO limit, CSV header unchanged; trips 0.3 m as in trial06 (5.87 s after onset)."""
        code, rows, summ, calls, on, shift = self.run_tool('lio_default', ['--rot-dur', '20', '--stop-after-dist', '0.3'],
                                                           events())
        self.assertEqual(code, step_cmd.EXIT_LIMIT)
        self.assertTrue(summ['limit_hit'].startswith('dist') and 'native' not in summ['limit_hit'])
        self.assertNotIn('native_x', rows[0])
        self.assertNotIn('native', summ)
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 'lio_default.csv.native.csv')))
        last_step = max(float(r['t_unix']) for r in rows if r['phase'] == 'step') - shift
        self.assertAlmostEqual(last_step - ON, 5.87, delta=0.35)


if __name__ == '__main__':
    unittest.main()
