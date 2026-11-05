"""Node integration: real ApproachGovernorNode on the ROS graph (synthetic inputs, no robot, no sleeps for sync)."""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    cfg = os.path.join(get_package_share_directory('qazbot_approach_governor'), 'config', 'approach_governor.yaml')
    node = launch_ros.actions.Node(package='qazbot_approach_governor', executable='approach_governor',
                                   name='approach_governor', parameters=[cfg], output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {}


def spin_until(node, pred, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


class TestGovernorNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.n = Node('gov_test')
        cls.out = []
        cls.status = []
        cls.cmd_pub = cls.n.create_publisher(Twist, 'cmd_vel_in', 10)
        cls.plan_pub = cls.n.create_publisher(Path, 'plan', 10)
        cls.odom_pub = cls.n.create_publisher(Odometry, 'odom', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        cls.tf = TransformBroadcaster(cls.n)
        cls.n.create_subscription(Twist, 'cmd_vel_out', lambda m: cls.out.append(m), 10)
        cls.n.create_subscription(DiagnosticStatus, 'status', lambda m: cls.status.append(m), 10)
        cls.stamp0 = cls.n.get_clock().now()

    @classmethod
    def tearDownClass(cls):
        cls.n.destroy_node()
        rclpy.shutdown()

    def pose(self, x):
        now = self.n.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = now
        o.pose.pose.position.x = x
        self.odom_pub.publish(o)

    def send_plan(self):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.n.get_clock().now().to_msg()
        from geometry_msgs.msg import PoseStamped
        for i in range(31):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = 0.1 * i
            p.poses.append(ps)
        self.plan_pub.publish(p)

    def drive(self, x0, x1, speed=0.5, hz=10):
        """Move the synthetic robot from x0 to x1 at `speed` (clock based, pose/odom published at `hz`)."""
        t0 = time.monotonic()
        next_pub = 0.0
        while True:
            el = time.monotonic() - t0
            x = x0 + speed * el
            if x >= x1:
                break
            if el >= next_pub:
                self.pose(x)
                if int(el) != int(next_pub - 1.0 / hz):
                    self.send_plan()  # Nav2 re-publishes the plan about once per second
                next_pub += 1.0 / hz
            rclpy.spin_once(self.n, timeout_sec=0.01)
        self.pose(x1)

    def command(self, vx=0.2, wz=0.3):
        self.out.clear()
        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        self.cmd_pub.publish(m)
        self.assertTrue(spin_until(self.n, lambda: len(self.out) > 0), 'no output command')
        return self.out[-1]

    def test_1_pass_far_then_brake_near_then_wz_untouched(self):
        self.assertTrue(spin_until(self.n, lambda: self.cmd_pub.get_subscription_count() > 0
                                   and self.plan_pub.get_subscription_count() > 0))
        self.send_plan()  # 3.0 m straight
        self.drive(0.0, 1.5)
        spin_until(self.n, lambda: False, 0.3)
        far = self.command(0.2, 0.3)
        self.assertAlmostEqual(far.linear.x, 0.2)
        self.assertAlmostEqual(far.angular.z, 0.3)
        self.drive(1.5, 2.85)  # remaining ~0.15 < 0.5 m/s * 0.4 s
        near = self.command(0.12, 0.3)
        self.assertEqual(near.linear.x, 0.0)
        self.assertAlmostEqual(near.angular.z, 0.3)  # yaw command forwarded untouched

    def test_2_status_reports_state(self):
        self.assertTrue(spin_until(self.n, lambda: any(s.message in ('braking', 'pass_through', 'no_data') for s in self.status), 5))

    def test_3_zero_command_passes_and_no_plan_passes(self):
        z = self.command(0.0, 0.0)
        self.assertEqual((z.linear.x, z.angular.z), (0.0, 0.0))

    def test_4_goal_coordinator_contract_single_stable_publisher_and_zero_forwarded(self):
        """GC trips on publisher count != 1 or a changed publisher GID on its cmd topic; zero must be forwarded."""
        gids = set()
        for _ in range(20):
            infos = self.n.get_publishers_info_by_topic('cmd_vel_out')
            self.assertEqual(len(infos), 1)
            gids.add(bytes(infos[0].endpoint_gid))
            z = self.command(0.0, 0.0)
            self.assertEqual((z.linear.x, z.linear.y, z.angular.z), (0.0, 0.0, 0.0))
        self.assertEqual(len(gids), 1)

    def test_5_implausible_odom_does_not_brake_and_is_reported(self):
        # robot at the end of the plan (remaining 0.1 m), odom jumps 20 m per sample -> speed estimate is a glitch
        # first a stand-still at the end: the robot is stopped, so the brake latch of test_1 is released
        for _ in range(8):
            self.pose(2.9)
            spin_until(self.n, lambda: False, 0.1)
        self.command(0.0, 0.0)  # commands arrive continuously in operation; this one lets the node see the stand-still
        for k in range(6):
            now = self.n.get_clock().now().to_msg()
            o = Odometry()
            o.header.stamp = now
            o.pose.pose.position.x = 2.9 + 20.0 * k
            self.odom_pub.publish(o)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = 2.9
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)
            spin_until(self.n, lambda: False, 0.1)
        self.send_plan()
        spin_until(self.n, lambda: False, 0.2)
        out = self.command(0.12, 0.0)
        self.assertEqual(out.linear.x, 0.12)
        self.status.clear()
        self.assertTrue(spin_until(self.n, lambda: any(
            any(kv.key == 'no_data_reason' and kv.value == 'speed_implausible' for kv in s.values) for s in self.status), 5))
