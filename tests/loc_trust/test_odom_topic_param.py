"""localization_trust_node.py odom_topic / odom_source parameters (roadmap v12 M0a, native_imu odometry mode).

Runs the real trust node in an isolated domain and checks the graph and its status:
  * default: subscribes /qazbot_lio/odom_level (unchanged), status odom_source=lio;
  * odom_topic:=/qazbot_native/odom odom_source:=native_imu: subscribes only that topic, logs and reports the source.
No thresholds or policy are involved: the parameters only select the input topic and label the status.

Run: bash -c 'source ws/src/navigation/env.sh && cd tests/loc_trust && env -u FASTRTPS_DEFAULT_PROFILES_FILE \
              ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m unittest -v test_odom_topic_param'
"""
import json
import os
import signal
import subprocess
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.join(HERE, '..', '..', 'ws', 'src', 'navigation')
MAP = os.path.join(NAV, '..', 'maps', 'first_map_nav2_v3', 'map.yaml')
ISOLATED = os.environ.get('ROS_DOMAIN_ID') == '88' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
INPUTS = ('localization_trust_inputs', '/qazbot_d1_map')


@unittest.skipUnless(ISOLATED, 'needs ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 (never the robot graph)')
class OdomTopicParam(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        from std_msgs.msg import String
        cls.rclpy, cls.String = rclpy, String
        cls.own_init = not rclpy.ok()
        if cls.own_init:
            rclpy.init()
        cls.node = rclpy.create_node('qazbot_test_trust_odom_topic')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if cls.own_init:
            cls.rclpy.try_shutdown()

    def run_trust(self, *params):
        log = tempfile.NamedTemporaryFile('w+', suffix='.log', delete=False)
        args = ['python3', os.path.join(NAV, 'localization_trust_node.py'), '--ros-args', '-p', f'map_yaml:={MAP}']
        for p in params:
            args += ['-p', p]
        p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def stop():
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(20)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(10)
            log.close()
            os.unlink(log.name)
        self.addCleanup(stop)
        status = []
        sub = self.node.create_subscription(self.String, '/qazbot_d1_map/localization_trust/status',
                                            lambda m: status.append(json.loads(m.data)), 10)
        self.addCleanup(self.node.destroy_subscription, sub)
        t0, subs = time.monotonic(), set()
        while time.monotonic() - t0 < 30 and not (status and subs):
            self.assertIsNone(p.poll(), 'trust node exited')
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                subs = {t for t, _ in self.node.get_subscriber_names_and_types_by_node(*INPUTS)}
            except Exception:                                   # node not discovered yet
                subs = set()
        self.assertTrue(status and subs, 'trust node not up')
        with open(log.name) as f:
            text = f.read()
        return subs, status[-1], text

    def test_default_is_lio_topic(self):
        subs, status, text = self.run_trust()
        self.assertIn('/qazbot_lio/odom_level', subs)
        self.assertNotIn('/qazbot_native/odom', subs)
        self.assertEqual((status['odom_source'], status['odom_topic']), ('lio', '/qazbot_lio/odom_level'))
        self.assertIn('odom_source=lio, odom_topic=/qazbot_lio/odom_level', text)

    def test_native_topic(self):
        subs, status, text = self.run_trust('odom_topic:=/qazbot_native/odom', 'odom_source:=native_imu')
        self.assertIn('/qazbot_native/odom', subs)
        self.assertNotIn('/qazbot_lio/odom_level', subs)
        self.assertEqual((status['odom_source'], status['odom_topic']), ('native_imu', '/qazbot_native/odom'))
        self.assertIn('odom_source=native_imu, odom_topic=/qazbot_native/odom', text)
        self.assertNotIn('different sources', text)

    def test_mismatch_warns(self):
        _, _, text = self.run_trust('odom_source:=native_imu')
        self.assertIn('different sources', text)


if __name__ == '__main__':
    unittest.main()
