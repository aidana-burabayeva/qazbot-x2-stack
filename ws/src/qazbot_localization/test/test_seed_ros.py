"""seed_pose.py --from-file on a ROS graph: publishes the saved pose ONCE with its covariance, only after AMCL's
initialpose subscriber is matched (a trust-node subscriber alone is not enough). Fake 'amcl' node, no real stack.
Isolated graph only (skipped otherwise):
  cd tests/loc_trust && env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 \
      python3 -m unittest -v test_seed_ros"""
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

from test_pose_persist import COV, MAP_YAML, NAV, P, SHA

ISOLATED = os.environ.get('ROS_DOMAIN_ID') == '89' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'


@unittest.skipUnless(ISOLATED, 'needs ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 (never the robot graph)')
class SeedOnGraph(unittest.TestCase):
    def test_publishes_once_with_saved_covariance_after_amcl_matches(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from rclpy.qos import qos_profile_system_default
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        trust = rclpy.create_node('localization_trust', namespace='qazbot_d1_map')
        got_trust, got_amcl = [], []
        trust.create_subscription(PoseWithCovarianceStamped, 'initialpose', got_trust.append, qos_profile_system_default)
        tf_pub = trust.create_publisher(TFMessage, '/qazbot_d1_map/tf', 10)
        def tf():
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'odom', 'base_footprint'
            t.header.stamp = trust.get_clock().now().to_msg()
            tf_pub.publish(TFMessage(transforms=[t]))
        trust.create_timer(0.1, tf)
        d = tempfile.TemporaryDirectory()
        path = os.path.join(d.name, 'pose.json')
        P.save(path, (1.25, -3.5, 0.7), COV, SHA, 'ready')
        proc = subprocess.Popen([sys.executable, os.path.join(NAV, 'seed_pose.py'), '--from-file', path,
                                 '--map-yaml', MAP_YAML], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        amcl = None
        try:
            end = time.monotonic() + 8.0                 # test bound: seed sees trust + TF but no AMCL yet
            while time.monotonic() < end:
                rclpy.spin_once(trust, timeout_sec=0.1)
            self.assertIsNone(proc.poll(), 'seed must keep waiting while AMCL is absent')
            self.assertEqual(got_trust, [])
            amcl = rclpy.create_node('amcl', namespace='qazbot_d1_map')
            amcl.create_subscription(PoseWithCovarianceStamped, 'initialpose', got_amcl.append, qos_profile_system_default)
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor()
            ex.add_node(trust)
            ex.add_node(amcl)
            end = time.monotonic() + 60.0
            while proc.poll() is None and time.monotonic() < end:
                ex.spin_once(timeout_sec=0.1)
            end = time.monotonic() + 2.0                 # drain: delivery was acked before the seed exited
            while time.monotonic() < end:
                ex.spin_once(timeout_sec=0.1)
            out, err = proc.communicate(timeout=10)
            self.assertEqual(proc.returncode, 0, err)
            self.assertEqual(json.loads(out.strip().splitlines()[-1])['status'],
                             'one hypothesis published; localization not accepted')
            self.assertEqual(len(got_amcl), 1)
            self.assertEqual(len(got_trust), 1)
            m = got_amcl[0]
            self.assertEqual(m.header.frame_id, 'map')
            self.assertEqual((m.pose.pose.position.x, m.pose.pose.position.y), (1.25, -3.5))
            import math
            self.assertAlmostEqual(2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w), 0.7)
            self.assertEqual(list(m.pose.covariance), [float(v) for v in COV])
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            if amcl is not None:
                amcl.destroy_node()
            trust.destroy_node()
            rclpy.shutdown()
            d.cleanup()


if __name__ == '__main__':
    unittest.main()
