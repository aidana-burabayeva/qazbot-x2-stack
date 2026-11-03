#!/usr/bin/env python3
"""native_imu odometry mode (roadmap v12 M0a; design reports/20261027-native-odom-nav2-design.md 3.4-3.6):
  * ws/src/lio/native_odom_bridge.py on the trial06 fixture (in-place 67 deg turn whose LIO drifted 1.77 m):
    TF odom->base_footprint and /qazbot_native/odom, source stamps, decimation, guards (stale, future, non-finite,
    origin jump, foreign frame, stamp back), mode / rate refusal;
  * ws/src/lio/level_bridge.py publish_tf:=false: no TF publisher at all, /odom_level and /scan identical;
  * single owner of odom->base_footprint: localization.launch.py for odom_source lio / native_imu / refused values,
    and tools/stack/odom_owner_probe.py verdict logic.

No robot, no MC, no hardware DDS: nodes run in-process in an isolated domain (88, localhost); callbacks are fed directly
and publishers are captured (nothing is published on /aima/*).

Run: bash -c 'source ws/src/navigation/env.sh && env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=88 \
              ROS_LOCALHOST_ONLY=1 python3 -m unittest -v tests/lio/test_native_odom_bridge.py'
"""
import gzip
import importlib.util
import json
import math
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LIO_SRC = ROOT / 'ws' / 'src' / 'lio'
NAV = ROOT / 'ws' / 'src' / 'navigation'
STACK = ROOT / 'tools' / 'stack'
sys.path.insert(0, str(LIO_SRC))
sys.path.insert(0, str(STACK))
import native_odom as no  # noqa: E402
import odom_owner_probe as probe  # noqa: E402

D = json.load(gzip.open(ROOT / 'tests' / 's3' / 'data' / 'trial06_native_series.json.gz', 'rt'))
ON = next(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)          # first nonzero command (receive time)
OFF = max(r[0] for r in D['cmd'] if abs(r[2]) > 1e-9)          # last nonzero command
LEG = ('leg_odom', 'lidar_imu_chest_front')
SRC = {round(r[1], 6): r for r in D['odom']}                   # source stamp -> fixture row


def s_of(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def tf_yaw(t):
    q = t.transform.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ros():
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.parameter import Parameter
    except ImportError as e:
        raise unittest.SkipTest(str(e))
    os.environ.setdefault('ROS_DOMAIN_ID', '88')
    if os.environ['ROS_DOMAIN_ID'] == '0':
        raise RuntimeError('never run these tests on domain 0 (live stack)')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    if not rclpy.ok():
        rclpy.init()
        _OWN_INIT.append(rclpy)
    return rclpy, Odometry, Parameter


_OWN_INIT = []


def tearDownModule():
    for rclpy in _OWN_INIT:                                     # leave rclpy as found (other modules init their own)
        rclpy.try_shutdown()


class Bridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rclpy, cls.Odometry, cls.Parameter = ros()
        import native_odom_bridge as nb
        cls.nb = nb

    def make(self, **params):
        Bridge.k = getattr(Bridge, 'k', 0) + 1
        node = self.nb.NativeOdomBridge(parameter_overrides=[self.Parameter(k, value=v) for k, v in params.items()],
                                        cli_args=['--ros-args', '-r', f'__node:=test_native_bridge_{Bridge.k}'])
        self.addCleanup(node.destroy_node)
        out = dict(tf=[], odom=[], status=[])
        if node.tf is not None:
            node.tf.sendTransform = out['tf'].append
        node.pub_odom.publish = out['odom'].append
        node.pub_status.publish = out['status'].append
        self.clock = 0.0
        node.now = lambda: self.clock
        return node, out

    def msg(self, stamp, t7, frames=LEG, cov=None):
        m = self.Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = int(stamp), int(round((stamp - int(stamp)) * 1e9))
        m.header.frame_id, m.child_frame_id = frames
        p, q = m.pose.pose.position, m.pose.pose.orientation
        p.x, p.y, p.z, q.x, q.y, q.z, q.w = (float(v) for v in t7)
        m.pose.covariance = cov or [7.1e-4 if i in (0, 7) else (7.3e-5 if i == 35 else 0.0) for i in range(36)]
        return m

    def feed(self, node, until=None):
        for r in sorted(D['odom'], key=lambda r: r[0]):
            if until is not None and r[0] > until:
                break
            self.clock = r[0]
            node.on_leg(self.msg(r[1], r[2:9]))

    # ---- publishers / configuration
    def test_publishers(self):
        for publish_tf, tf_topics in ((True, {'/tf'}), (False, set())):
            node, _ = self.make(publish_tf=publish_tf)
            topics = {n for n, _ in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())}
            self.assertEqual(topics - {'/rosout', '/parameter_events'},
                             {'/qazbot_native/odom', '/qazbot_native/status'} | tf_topics)
            self.assertFalse(any(t.startswith('/aima') for t in topics))

    def test_refuses_unimplemented_mode_and_bad_rate(self):
        for params in (dict(mode='native_base'), dict(mode='lio'), dict(publish_rate_hz=5.0),
                       dict(publish_rate_hz=200.0)):
            with self.assertRaises(ValueError, msg=params):
                self.nb.NativeOdomBridge(parameter_overrides=[self.Parameter(k, value=v) for k, v in params.items()])
        node, _ = self.make(publish_rate_hz=30)               # an int from the command line is accepted
        self.assertEqual(node.rate, 30.0)

    # ---- trial06
    def test_trial06_in_place_turn(self):
        """TF odom->base_footprint = native IMU footprint: in-place turn of 67.5 deg moves the footprint ~0.115 m
        (LIO: 1.77 m in the same window); every TF sample is the native footprint of its own source sample."""
        node, out = self.make()
        self.feed(node)
        tf = out['tf']
        self.assertTrue(all((t.header.frame_id, t.child_frame_id) == ('odom', 'base_footprint') for t in tf))
        self.assertTrue(all(t.transform.translation.z == 0.0 and t.transform.rotation.x == 0.0
                            and t.transform.rotation.y == 0.0 for t in tf))
        st = [s_of(t.header) for t in tf]
        self.assertTrue(all(b > a for a, b in zip(st, st[1:])))
        rate = (len(st) - 1) / (st[-1] - st[0])
        self.assertGreater(rate, 27.0)                         # 30 Hz less the withheld stale (recorder-burst) samples
        self.assertLess(rate, 30.5)
        for t, s in zip(tf, st):
            r = SRC[round(s, 6)]                               # source stamp, never re-stamped
            self.assertLessEqual(r[0] - r[1], no.NATIVE_MAX_AGE)          # never published stale
            self.assertAlmostEqual(t.transform.translation.x, r[2], places=9)
            self.assertAlmostEqual(t.transform.translation.y, r[3], places=9)
            self.assertAlmostEqual(no.wrap(tf_yaw(t) - no.imu_footprint_yaw(r[5:9])[0]), 0.0, places=9)
        before = [t for t, s in zip(tf, st) if SRC[round(s, 6)][0] < ON - 0.2][-1]
        end = tf[-1]
        disp = math.hypot(end.transform.translation.x - before.transform.translation.x,
                          end.transform.translation.y - before.transform.translation.y)
        dyaw = math.degrees(no.wrap(tf_yaw(end) - tf_yaw(before)))
        self.assertAlmostEqual(dyaw, 67.5, delta=0.3)
        self.assertLess(disp, 0.15)                             # 0.115 m IMU lever arm swing, not 1.77 m
        self.assertAlmostEqual(disp, 0.115, delta=0.01)
        path = sum(math.hypot(b.transform.translation.x - a.transform.translation.x,
                              b.transform.translation.y - a.transform.translation.y) for a, b in zip(tf, tf[1:]))
        self.assertLess(path, 0.6)                              # LIO accumulated > 1.77 m in this window
        # LIO in the same fixture window, for the record
        lio = D['lio']
        a = [r for r in lio if r[0] < ON - 0.2][-1]
        self.assertGreater(math.hypot(lio[-1][2] - a[2], lio[-1][3] - a[3]), 1.7)

    def test_trial06_odometry_message(self):
        node, out = self.make()
        self.feed(node)
        odom, tf = out['odom'], {s_of(t.header): t for t in out['tf']}
        self.assertGreater(len(odom), 0.9 * len(tf))
        for o in odom:
            self.assertEqual((o.header.frame_id, o.child_frame_id), ('odom', 'base_footprint'))
            t = tf[s_of(o.header)]                              # same sample as a TF
            self.assertEqual(o.pose.pose.position.x, t.transform.translation.x)
            self.assertEqual(o.pose.pose.orientation.z, t.transform.rotation.z)
            self.assertAlmostEqual(o.pose.pose.position.z, SRC[round(s_of(o.header), 6)][4], places=9)
        self.assertAlmostEqual(odom[-1].pose.covariance[0], 7.1e-4)
        self.assertAlmostEqual(odom[-1].pose.covariance[35], 7.3e-5)
        self.assertEqual(odom[-1].twist.covariance[0], self.nb.TWIST_VAR_NOMINAL)
        # standing after the turn (FSM STAND at OFF + ~2 s): twist ~ 0; during the turn: positive yaw rate
        still = [o for o in odom if s_of(o.header) > OFF + 3.0]
        self.assertTrue(still)
        self.assertLess(max(abs(o.twist.twist.angular.z) for o in still), 0.05)
        self.assertLess(max(math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y) for o in still), 0.05)
        turning = [o.twist.twist.angular.z for o in odom if ON + 1.0 < s_of(o.header) < OFF - 1.0]
        self.assertGreater(sum(turning) / len(turning), 0.1)
        # no Odometry before the 0.3 s twist baseline, but TF from the first due sample
        self.assertGreaterEqual(s_of(odom[0].header) - min(tf), no.TWIST_BASELINE_S)

    def test_status(self):
        node, out = self.make()
        self.feed(node, until=ON - 3.0)
        node.on_status()
        s = json.loads(out['status'][-1].data)
        self.assertEqual((s['mode'], s['publish_tf'], s['frames']), ('native_imu', True, ['odom', 'base_footprint']))
        self.assertEqual(s['fault'], '')
        self.assertEqual(s['latched'], '')
        self.assertGreater(s['messages']['tf'], 100)

    # ---- guards: each stops TF and odometry (trust then sees T1_odom_stale = fail-closed)
    def prime(self):
        node, out = self.make()
        self.feed(node, until=ON - 3.0)
        r = max((r for r in D['odom'] if r[0] <= ON - 3.0), key=lambda r: r[0])
        self.n0 = (len(out['tf']), len(out['odom']))
        self.assertGreater(self.n0[0], 100)
        return node, out, r[1], list(r[2:9])

    def count(self, out):
        return len(out['tf']), len(out['odom'])

    def stream(self, node, t0, t7, n=40, dt=0.006, age=0.006, frames=LEG):
        for k in range(1, n + 1):
            self.clock = t0 + k * dt + age
            node.on_leg(self.msg(t0 + k * dt, t7, frames))

    def test_guard_foreign_frame(self):
        node, out, t, p = self.prime()
        for frames in (('odom', 'lidar_imu_chest_front'), ('leg_odom', 'base_link')):
            self.stream(node, t, p, frames=frames)
        self.assertEqual(self.count(out), self.n0)
        self.assertEqual(node.guard.latched, '')

    def test_guard_stale_withheld_then_resumes(self):
        node, out, t, p = self.prime()
        self.stream(node, t, p, age=0.3)                       # every sample 0.3 s old on arrival (> 0.2 s)
        self.assertEqual(self.count(out), self.n0)
        node.on_status()
        self.assertEqual(json.loads(out['status'][-1].data)['fault'], 'stale')
        self.stream(node, t + 0.3, p)                          # fresh again: publication resumes, no latch
        self.assertGreater(self.count(out)[0], self.n0[0])

    def test_guard_future_dropped(self):
        node, out, t, p = self.prime()
        self.stream(node, t + 0.5, p, age=-0.5)               # stamps 0.5 s ahead of the clock
        self.assertEqual(self.count(out), self.n0)
        self.assertEqual(node.guard.counts['future'], 40)

    def test_guard_non_finite_latches(self):
        node, out, t, p = self.prime()
        self.clock = t + 0.012
        node.on_leg(self.msg(t + 0.006, [float('nan')] + p[1:]))
        self.stream(node, t + 0.006, p)
        self.assertEqual(self.count(out), self.n0)
        self.assertEqual(node.guard.fault(self.clock), 'non_finite')
        node.on_status()
        self.assertEqual(json.loads(out['status'][-1].data)['latched'], 'non_finite')

    def test_guard_zero_quaternion_latches(self):
        node, out, t, p = self.prime()
        self.clock = t + 0.012
        node.on_leg(self.msg(t + 0.006, p[:3] + [0.0, 0.0, 0.0, 0.0]))
        self.stream(node, t + 0.006, p)
        self.assertEqual(self.count(out), self.n0)

    def test_guard_origin_jump_latches(self):
        """MC leg-odometry origin reset (e.g. MC restart): 0.5 m step with continuous stamps -> latched, no output
        even when the samples continue smoothly from the new origin."""
        node, out, t, p = self.prime()
        q = [p[0] + 0.5] + p[1:]
        self.stream(node, t, q)
        self.assertEqual(self.count(out), self.n0)
        self.assertTrue(node.guard.fault(self.clock).startswith('origin_reset'))

    def test_guard_heading_jump_latches(self):
        node, out, t, p = self.prime()
        yaw0 = no.imu_footprint_yaw(p[3:7])[0]
        s, c = math.sin(0.25), math.cos(0.25)                   # +0.5 rad about z
        x, y, z, w = p[3:7]
        q = (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)
        self.assertAlmostEqual(no.wrap(no.imu_footprint_yaw(q)[0] - yaw0), 0.5, places=6)
        self.stream(node, t, p[:3] + list(q))
        self.assertEqual(self.count(out), self.n0)
        self.assertTrue(node.guard.latched.startswith('origin_reset:dyaw'))

    # ---- long stream gap (M0a item 8): the jump bound's dt is capped at JUMP_MAX_DT (0.2 s)
    def test_gap_then_origin_reset_latches(self):
        """MC restart: 3 s without leg odometry, then the origin moved 1.5 m. Uncapped the bound would allow
        0.05 + 1.5 * 3 = 4.55 m; capped it is 0.35 m -> latched, nothing published afterwards."""
        node, out, t, p = self.prime()
        self.stream(node, t + 3.0, [p[0] + 1.5, p[1] - 0.4] + p[2:])
        self.assertEqual(self.count(out), self.n0)
        self.assertTrue(node.guard.latched.startswith('origin_reset:dp='), node.guard.latched)
        self.assertIn('gap=3.', node.guard.latched)
        node.on_status()
        self.assertTrue(json.loads(out['status'][-1].data)['latched'].startswith('origin_reset'))

    def test_gap_then_small_reset_latches(self):
        node, out, t, p = self.prime()
        self.stream(node, t + 3.0, [p[0] + 0.4] + p[1:])        # 0.4 m > 0.35 m capped bound
        self.assertEqual(self.count(out), self.n0)
        self.assertTrue(node.guard.latched.startswith('origin_reset'))

    def test_gap_without_reset_resumes(self):
        """Same 3 s gap, pose consistent (robot stood, or moved < 0.35 m): publication resumes on the new samples,
        no latch, twist baseline refills."""
        for dx in (0.0, 0.3):
            node, out, t, p = self.prime()
            self.stream(node, t + 3.0, [p[0] + dx] + p[1:], n=100)
            self.assertEqual(node.guard.latched, '')
            self.assertGreater(self.count(out)[0], self.n0[0])
            self.assertGreater(self.count(out)[1], self.n0[1])
            tf = out['tf'][-1]
            self.assertAlmostEqual(tf.transform.translation.x, p[0] + dx, places=9)

    def test_stamp_back_clears_twist_and_continues(self):
        node, out, t, p = self.prime()
        self.stream(node, t - 5.0, p, n=100)                   # clock stepped back 5 s (PTP resync)
        self.assertEqual(node.guard.counts['stamp_back'], 1)
        self.assertEqual(node.guard.latched, '')
        n_tf, n_odom = self.count(out)
        self.assertGreater(n_tf, self.n0[0])                   # TF continues on the new stamps
        new = [o for o in out['odom'][self.n0[1]:]]
        self.assertTrue(new)                                   # odometry again once the 0.3 s baseline refilled
        self.assertGreaterEqual(s_of(new[0].header) - (t - 5.0), no.TWIST_BASELINE_S)


class LevelBridgePublishTf(unittest.TestCase):
    """publish_tf:=false: level_bridge has no TF publisher; /odom_level and /scan are identical to publish_tf:=true."""

    @classmethod
    def setUpClass(cls):
        cls.rclpy, cls.Odometry, cls.Parameter = ros()
        import level_bridge as lb
        cls.lb = lb

    def run_bridge(self, publish_tf):
        import numpy as np
        from sensor_msgs.msg import Imu
        from sensor_msgs_py import point_cloud2 as pc2
        from std_msgs.msg import Header
        node = self.lb.LevelBridge(parameter_overrides=[self.Parameter('publish_tf', value=publish_tf)],
                                   cli_args=['--ros-args', '-r', f'__node:=test_level_bridge_{publish_tf}'.lower()])
        self.addCleanup(node.destroy_node)
        out = dict(tf=[], odom=[], scan=[])
        if node.tf is not None:
            node.tf.sendTransform = out['tf'].append
        node.pub_odom.publish = out['odom'].append
        node.pub_scan.publish = out['scan'].append
        topics = {n for n, _ in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())}

        def odom(t, x, yaw):
            m = self.Odometry()
            m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
            m.header.frame_id, m.child_frame_id = 'camera_init', 'body'
            m.pose.pose.position.x = x
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            return m
        node.on_odom(odom(100.0, 0.0, 0.0))                     # R_wb for levelling
        for k in range(120):                                    # 1.2 s of IMU at 100 Hz, gravity along +z
            m = Imu()
            m.header.stamp.sec, m.header.stamp.nanosec = 100 + k // 100, (k % 100) * 10**7
            m.linear_acceleration.z = 9.81
            node.on_imu(m)
        rng = np.random.default_rng(1)
        P = np.c_[rng.uniform(-6, 6, 3000), rng.uniform(-6, 6, 3000), rng.uniform(-0.5, 0.7, 3000)]
        for k in range(20):
            t = 101.0 + 0.1 * k
            o = odom(t, 0.05 * k, 0.02 * k)
            node.on_odom(o)
            h = Header()
            h.stamp, h.frame_id = o.header.stamp, 'camera_init'
            node.on_cloud(pc2.create_cloud_xyz32(h, P.tolist()))
        return node, out, topics

    def test_publish_tf_false_only_drops_tf(self):
        on, out_on, topics_on = self.run_bridge(True)
        off, out_off, topics_off = self.run_bridge(False)
        self.assertIn('/tf', topics_on)
        self.assertNotIn('/tf', topics_off)
        self.assertIsNone(off.tf)
        self.assertEqual(len(out_on['tf']), 20)
        self.assertEqual(out_off['tf'], [])
        self.assertGreater(len(out_on['odom']), 10)
        self.assertEqual(len(out_on['scan']), 20)
        self.assertEqual(out_on['odom'], out_off['odom'])       # message equality, field by field
        self.assertEqual(out_on['scan'], out_off['scan'])
        self.assertTrue(all((o.header.frame_id, o.child_frame_id) == ('odom', 'base_footprint')
                            for o in out_off['odom']))
        self.assertTrue(all(s.header.frame_id == 'base_footprint' for s in out_off['scan']))


def load_launch():
    spec = importlib.util.spec_from_file_location('loc_launch_m0a', NAV / 'localization.launch.py')
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(NAV))
    spec.loader.exec_module(mod)
    return mod


class SingleOwnerLaunch(unittest.TestCase):
    """localization.launch.py: exactly one odom->base_footprint owner for every accepted odom_source."""

    @classmethod
    def setUpClass(cls):
        try:
            from launch import LaunchContext
            from launch.actions import ExecuteProcess, TimerAction
            from launch.utilities import perform_substitutions
        except ImportError as e:
            raise unittest.SkipTest(str(e))
        cls.LaunchContext, cls.ExecuteProcess, cls.TimerAction = LaunchContext, ExecuteProcess, TimerAction
        cls.perform = staticmethod(perform_substitutions)
        cls.mod = load_launch()

    def processes(self, source):
        """[(start time, cmd list)] of the processes that would start for odom_source=source."""
        ld = self.mod.generate_launch_description()
        ctx = self.LaunchContext()
        ctx.launch_configurations.update(odom_source=source, native_rate_hz='30.0', lio_binary='fastlio',
                                         native_leg_topic='/aima/mc/leg_odometry',
                                         map_yaml='m.yaml', lio_cpus='', seed_pose_file='')
        out = []
        for e in ld.entities:
            acts, t = ((e.actions, float(e.period)) if isinstance(e, self.TimerAction) else ([e], 0.0))
            for a in acts:
                if not isinstance(a, self.ExecuteProcess):
                    continue
                if a.condition is not None and not a.condition.evaluate(ctx):
                    continue
                from launch_ros.actions import Node as RosNode
                if isinstance(a, RosNode):                     # map_server, amcl, lifecycle: never TF owners here
                    out.append((t, ['<ros node>', a.node_executable]))
                    continue
                cmd = [self.perform(ctx, c) for c in a.cmd]
                out.append((t, cmd))
        return out

    def owners(self, procs):
        own = []
        for _, cmd in procs:
            script = os.path.basename(cmd[1]) if len(cmd) > 1 else ''
            if script in ('level_bridge.py', 'native_odom_bridge.py'):
                pt = [cmd[i + 1] for i, c in enumerate(cmd[:-1]) if c == '-p' and cmd[i + 1].startswith('publish_tf:=')]
                self.assertLessEqual(len(pt), 1)
                self.assertIn('/tf:=/qazbot_d1_map/tf', cmd)
                if (pt or ['publish_tf:=true'])[0] == 'publish_tf:=true':
                    own.append(script)
        return own

    def test_lio_default_single_owner_level_bridge(self):
        ld = self.mod.generate_launch_description()
        default = next(e for e in ld.entities if getattr(e, 'name', None) == 'odom_source')
        self.assertEqual(self.perform(self.LaunchContext(), default.default_value), 'lio')
        procs = self.processes('lio')
        self.assertEqual(self.owners(procs), ['level_bridge.py'])
        self.assertFalse(any('native_odom_bridge.py' in ' '.join(c) for _, c in procs))

    def test_native_imu_single_owner_native_bridge(self):
        procs = self.processes('native_imu')
        self.assertEqual(self.owners(procs), ['native_odom_bridge.py'])
        lb = next(c for _, c in procs if c[1].endswith('level_bridge.py'))
        self.assertIn('publish_tf:=false', lb)
        self.assertIn('/scan:=/qazbot_lio/scan', lb)            # scan + odom_level still produced
        self.assertIn('/odom_level:=/qazbot_lio/odom_level', lb)
        nb = next(c for _, c in procs if c[1].endswith('native_odom_bridge.py'))
        self.assertIn('mode:=native_imu', nb)
        self.assertIn('publish_rate_hz:=30.0', nb)
        self.assertIn('leg_topic:=/aima/mc/leg_odometry', nb)           # default: the real MC topic

    def test_lio_start_times_unchanged_by_native_process(self):
        """The native bridge is the last staggered process: every lio-mode start time is unchanged."""
        lio, native = self.processes('lio'), self.processes('native_imu')
        self.assertEqual([t for t, _ in lio], [t for t, _ in native][:len(lio)])
        self.assertTrue(native[-1][1][1].endswith('native_odom_bridge.py'))
        self.assertEqual(len(native), len(lio) + 1)

    def test_other_sources_refused(self):
        ld = self.mod.generate_launch_description()
        arg = next(e for e in ld.entities if getattr(e, 'name', None) == 'odom_source')
        self.assertEqual(sorted(arg.choices), ['lio', 'native_imu'])
        for bad in ('native_base', 'LIO', ''):
            with self.assertRaises(ValueError):
                self.processes(bad)

    def test_native_bridge_has_shutdown_guard(self):
        from launch.actions import RegisterEventHandler
        ld = self.mod.generate_launch_description()
        guarded = [e.event_handler._OnActionEventBase__action_matcher for e in ld.entities
                   if isinstance(e, RegisterEventHandler)]
        native = [a for e in ld.entities if isinstance(e, self.TimerAction) for a in e.actions
                  if a.condition is not None and any('native_odom_bridge.py' in getattr(c, 'text', '')
                                                     for sub in a.cmd for c in sub)]
        self.assertEqual(len(native), 1)
        self.assertTrue(any(g is native[0] for g in guarded))


class JumpGuardGap(unittest.TestCase):
    """Pure JumpGuard: dt in the position bound is clamped to [0, JUMP_MAX_DT]."""

    def test_cap_justification(self):
        self.assertEqual(no.JUMP_MAX_DT, no.NATIVE_MAX_AGE)
        st = sorted(r[1] for r in D['odom'])
        self.assertLess(max(b - a for a, b in zip(st, st[1:])), no.JUMP_MAX_DT / 10)   # 166 Hz: gaps <= 14 ms
        self.assertAlmostEqual(no.JUMP_M + no.JUMP_M_PER_S * no.JUMP_MAX_DT, 0.35)

    def test_bound(self):
        for gap, d, reset in ((0.006, 0.05, False), (0.1, 0.19, False), (0.1, 0.21, True), (0.2, 0.349, False),
                              (0.2, 0.351, True), (3.0, 0.349, False), (3.0, 0.351, True), (30.0, 1.0, True),
                              (-5.0, 0.049, False), (-5.0, 0.051, True)):
            g = no.JumpGuard()
            self.assertEqual(g.check(100.0, 0.0, 0.0, 0.0), '')
            self.assertEqual(bool(g.check(100.0 + gap, d, 0.0, 0.0)), reset, (gap, d))

    def test_walking_stream_never_trips(self):
        """1 m/s (> 3x the Nav2 limit) sampled at 166 Hz: no reset."""
        g = no.JumpGuard()
        self.assertTrue(all(g.check(0.006 * k, 0.006 * k, 0.0, 0.0) == '' for k in range(1000)))


class OwnerProbeLogic(unittest.TestCase):
    def test_graph_owners(self):
        self.assertEqual(probe.graph_owners(['/qazbot_d1_map/amcl', '/qazbot_level_bridge']), ['lio'])
        self.assertEqual(probe.graph_owners(['/qazbot_native_odom_bridge', '/qazbot_d1_map/amcl']), ['native_imu'])
        self.assertEqual(probe.graph_owners(['/qazbot_native_odom_bridge', '/qazbot_level_bridge']),
                         ['lio', 'native_imu'])

    def test_match_and_verdict(self):
        tf = {k: (k * 1e-9, 0.0) for k in range(0, 10**9, 33 * 10**6)}
        native = {k: v for k, v in tf.items() if k >= 3 * 10**8}  # odometry starts after the twist baseline
        lio = {k: (k * 1e-9 + 1.0, 0.0) for k in range(0, 10**9, 10**8)}
        f = dict(lio=probe.match_fraction(tf, lio), native_imu=probe.match_fraction(tf, native))
        self.assertEqual(f, dict(lio=0.0, native_imu=1.0))
        self.assertEqual(probe.verdict(['native_imu'], f, len(tf), 'native_imu'), (True, []))
        self.assertFalse(probe.verdict(['native_imu'], f, len(tf), 'lio')[0])
        self.assertFalse(probe.verdict(['lio', 'native_imu'], f, len(tf))[0])
        self.assertFalse(probe.verdict(['native_imu'], f, 0)[0])
        # two publishers interleaved on the TF topic: neither stream explains all samples
        mixed = dict(tf)
        mixed.update({k + 1: v for k, v in lio.items()})
        both = dict(lio=probe.match_fraction(mixed, {k + 1: v for k, v in lio.items()}),
                    native_imu=probe.match_fraction(mixed, native))
        ok, problems = probe.verdict(['native_imu'], both, len(mixed))
        self.assertFalse(ok, both)


if __name__ == '__main__':
    unittest.main()
