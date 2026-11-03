#!/usr/bin/env python3
"""Runtime single-owner check of localization.launch.py odom_source (roadmap v12 M0a), isolated domain only.

Starts the REAL launch file (x2_lio_planar_bridge, map_server, AMCL, lifecycle manager, and for native_imu the native bridge)
with a stub in place of the FAST-LIO binary (a shell that sleeps; no LiDAR, no MC) and then:
  * graph: exactly one node publishes TF on /qazbot_d1_map/tf among x2_lio_planar_bridge / native_odom_bridge, and it is the
    one odom_source selects (x2_lio_planar_bridge has no TF publisher at all in native_imu);
  * native_imu data: recorded trial06 leg odometry (re-stamped to the current clock by this feeder, as a recorder
    replay would) is published on a TEST topic (native_leg_topic:=/qazbot_test/leg_odometry -- never /aima/*); every
    odom->base_footprint TF sample then equals a /qazbot_native/odom sample (tools/stack/odom_owner_probe.py verdict),
    at ~30 Hz.

Run (never on the robot graph):
  bash -c 'source ws/src/navigation/env.sh && env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=88 \
           ROS_LOCALHOST_ONLY=1 python3 -m unittest -v tests/lio/test_odom_source_launch.py'
"""
import gzip
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NAV = ROOT / 'ws' / 'src' / 'navigation'
sys.path.insert(0, str(ROOT / 'tools' / 'stack'))
import odom_owner_probe as probe  # noqa: E402

ISOLATED = os.environ.get('ROS_DOMAIN_ID') == '88' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
MAP = ROOT / 'ws' / 'src' / 'maps' / 'first_map_nav2_v3' / 'map.yaml'
TEST_LEG = '/qazbot_test/leg_odometry'
D = json.load(gzip.open(ROOT / 'tests' / 's3' / 'data' / 'trial06_native_series.json.gz', 'rt'))


@unittest.skipUnless(ISOLATED, 'needs ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 (never the robot graph)')
class OdomSourceLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        cls.rclpy = rclpy
        cls.own_init = not rclpy.ok()                           # another test module may have initialised rclpy
        if cls.own_init:
            rclpy.init()
        cls.node = rclpy.create_node('qazbot_test_odom_owner')
        cls.tmp = tempfile.TemporaryDirectory()
        cls.stub = os.path.join(cls.tmp.name, 'fastlio_stub')
        with open(cls.stub, 'w') as f:
            f.write('#!/bin/sh\nexec sleep 3600\n')
        os.chmod(cls.stub, 0o755)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if cls.own_init:
            cls.rclpy.try_shutdown()
        cls.tmp.cleanup()

    def launch(self, source):
        log = open(os.path.join(self.tmp.name, f'launch_{source}.log'), 'w')
        p = subprocess.Popen(['ros2', 'launch', str(NAV / 'localization.launch.py'), f'lio_binary:={self.stub}',
                              f'map_yaml:={MAP}', 'seed_pose_file:=/nonexistent/qazbot_seed_disabled.json',
                              f'odom_source:={source}',
                              f'native_leg_topic:={TEST_LEG}'],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                             preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))

        def stop():
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(30)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(10)
            log.close()
        self.addCleanup(stop)
        return p, log.name

    def spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_nodes(self, names, p, timeout=40.0):
        """Until every node in `names` is on the graph (condition, not a fixed sleep)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.assertIsNone(p.poll(), 'launch exited early')
            have = {(ns.rstrip('/') + '/' + n) for n, ns in self.node.get_node_names_and_namespaces()}
            if names <= have:
                return have
            self.spin(0.2)
        self.fail(f'nodes {names} not up within {timeout} s; graph {sorted(have)}')

    def tf_publishers(self):
        return [f"{i.node_namespace.rstrip('/')}/{i.node_name}"
                for i in self.node.get_publishers_info_by_topic(probe.TF_TOPIC)]

    def test_lio(self):
        p, log = self.launch('lio')
        # amcl is the last staggered node of the localization set: all TF owner candidates have started by then
        self.wait_nodes({'/qazbot_x2_lio_planar_bridge', '/qazbot_d1_map/amcl', '/qazbot_d1_map/lifecycle_manager_localization'}, p)
        self.spin(1.0)
        self.assertEqual(probe.graph_owners(self.tf_publishers()), ['lio'], self.tf_publishers())
        names = {n for n, _ in self.node.get_node_names_and_namespaces()}
        self.assertNotIn('qazbot_native_odom_bridge', names)

    def test_native_imu(self):
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        p, log = self.launch('native_imu')
        self.wait_nodes({'/qazbot_x2_lio_planar_bridge', '/qazbot_native_odom_bridge', '/qazbot_d1_map/amcl'}, p)
        self.spin(1.0)
        pubs = self.tf_publishers()
        self.assertEqual(probe.graph_owners(pubs), ['native_imu'], pubs)
        self.assertNotIn('/qazbot_x2_lio_planar_bridge', pubs)
        # x2_lio_planar_bridge still publishes the scan and odom_level topics
        lb = {t for t, _ in self.node.get_publisher_names_and_types_by_node('qazbot_x2_lio_planar_bridge', '/')}
        self.assertTrue({'/qazbot_lio/scan', '/qazbot_lio/odom_level'} <= lb, lb)

        tf, odom = {}, {}

        def key(h):
            return h.stamp.sec * 10**9 + h.stamp.nanosec

        def on_tf(m):
            for t in m.transforms:
                if (t.header.frame_id, t.child_frame_id) == ('odom', 'base_footprint'):
                    tf[key(t.header)] = (t.transform.translation.x, t.transform.translation.y)
        subs = [self.node.create_subscription(TFMessage, probe.TF_TOPIC, on_tf, QoSProfile(depth=200)),
                self.node.create_subscription(
                    Odometry, '/qazbot_native/odom',
                    lambda m: odom.__setitem__(key(m.header), (m.pose.pose.position.x, m.pose.pose.position.y)),
                    QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))]
        pub = self.node.create_publisher(Odometry, TEST_LEG, qos_profile_sensor_data)
        self.spin(1.0)                                          # discovery of the feeder
        rows = sorted(D['odom'], key=lambda r: r[1])[:600]      # ~3.6 s of standing leg odometry (166 Hz)
        t_wall0, s0 = time.time(), rows[0][1]
        for r in rows:
            due = t_wall0 + (r[1] - s0)
            while time.time() < due:
                self.rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.002, due - time.time())))
            st = r[1] - s0 + t_wall0                            # feeder re-stamps the replay to the current clock
            m = Odometry()
            m.header.stamp.sec, m.header.stamp.nanosec = int(st), int(round((st - int(st)) * 1e9)) % 10**9
            m.header.frame_id, m.child_frame_id = 'leg_odom', 'lidar_imu_chest_front'
            q = m.pose.pose
            q.position.x, q.position.y, q.position.z = r[2], r[3], r[4]
            q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w = r[5], r[6], r[7], r[8]
            pub.publish(m)
        self.spin(0.5)
        for s in subs:
            self.node.destroy_subscription(s)
        self.node.destroy_publisher(pub)
        self.assertGreater(len(tf), 60, f'TF samples {len(tf)}; log {log}')
        span = (max(tf) - min(tf)) * 1e-9
        rate = (len(tf) - 1) / span
        self.assertGreater(rate, 24.0)
        self.assertLess(rate, 31.0)
        fractions = {'lio': 0.0, 'native_imu': probe.match_fraction(tf, odom)}
        ok, problems = probe.verdict(probe.graph_owners(pubs), fractions, len(tf), 'native_imu')
        self.assertTrue(ok, problems)
        with open(log) as f:
            text = f.read()
        self.assertIn('native_odom_bridge started: mode=native_imu publish_tf=True', text)   # stack_up.sh waits on it
        self.assertNotIn('LATCHED', text)


if __name__ == '__main__':
    unittest.main()
