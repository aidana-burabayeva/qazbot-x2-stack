"""Regression test (T1 2026-11-04): a cloud delivered BEFORE its odometry must not be lost.

Root cause of the sporadic missing scans: FAST-LIO publishes /Odometry before /cloud_registered with the same stamp, but
the two topics are separate DDS readers, so the node can take the cloud first. The old node looked up the exact stamp,
missed, and returned silently. Forced reproduction on bag d04_static_bag_183015 (clouds k % 10 in {0, 1} published before
their odometry): old node 116/145 scans (29 lost), fixed node 145/145.

This test applies the same swap pattern to the small fixture already in the repository (no bag): after levelling, every
cloud with index k % 10 in {0, 1} is published and processed (barrier) BEFORE its odometry. Expected: exactly the scans of
the in-order replay (parity_dump of the same fixture), bit-identical; i.e. exact stamp matching, no cloud lost.
Isolated domain from ROS_DOMAIN_ID (from the caller, never 0, ROS_LOCALHOST_ONLY=1), no sleeps (waits are message / service events with a timeout).
"""
import os
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'tools'))
import fixture  # noqa: E402
from test_parity import read_dump  # noqa: E402

FIXTURE = os.environ.get('FIXTURE', os.path.join(HERE, 'data', 'static_204153_w50.bin'))
IMU = '/aima/hal/sensor/lidar_chest_front/imu'
NODE = 'lidar_projector'
DEADLINE_S = 20.0


@pytest.mark.launch_test
def generate_test_description():
    # Isolated graph: the domain comes from the environment (set by the caller), never 0 (robot graph); host-local only.
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    if os.environ.get('ROS_DOMAIN_ID', '').strip() in ('', '0'):
        raise RuntimeError('set an isolated ROS_DOMAIN_ID (not unset/0, the robot graph) for this test, e.g. ROS_DOMAIN_ID=88')
    share = get_package_share_directory('qazbot_lidar_projector')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar_projector.launch.xml'))),
        launch_testing.actions.ReadyToTest()])


def key_of(sec, nanosec):
    return sec * 10 ** 9 + nanosec


class TestCloudBeforeOdom(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('cloud_before_odom_test')
        cls.pub_imu = cls.node.create_publisher(
            Imu, IMU, QoSProfile(depth=400, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        cls.pub_odom = cls.node.create_publisher(Odometry, '/qazbot_lio/odometry', 100)
        cls.pub_cloud = cls.node.create_publisher(PointCloud2, '/qazbot_lio/cloud_registered', 20)
        cls.param_client = cls.node.create_client(GetParameters, f'/{NODE}/get_parameters')
        cls.scans = {}
        cls.node.create_subscription(
            LaserScan, '/qazbot_lio/scan',
            lambda m: cls.scans.__setitem__(key_of(m.header.stamp.sec, m.header.stamp.nanosec), m),
            QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, cond, timeout=DEADLINE_S):
        end = time.monotonic() + timeout
        while not cond():
            self.assertLess(time.monotonic(), end, f'timeout (scans so far {len(self.scans)})')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def barrier(self):
        # Single-threaded executor serves subscriptions before services: once the node answers, everything published
        # before the request has been taken and processed (same mechanism as test_integration.py).
        fut = self.param_client.call_async(GetParameters.Request(names=['range_max']))
        self.spin_until(fut.done)

    def publish_odom(self, e):
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
        m.header.frame_id, m.child_frame_id = 'camera_init', 'body'
        (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) = e.data[0]
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
         m.pose.pose.orientation.w) = e.data[1]
        m.pose.covariance = list(e.data[2])
        self.pub_odom.publish(m)
        self.barrier()

    def publish_cloud(self, e):
        xyz, dense = e.data
        m = PointCloud2()
        m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
        m.header.frame_id = 'camera_init'
        m.height, m.width = 1, len(xyz)
        m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                    for n, o in (('x', 0), ('y', 4), ('z', 8))]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(xyz), dense
        m.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
        self.pub_cloud.publish(m)
        self.barrier()

    def expected(self, replay):
        dump = os.environ.get('PARITY_DUMP') or os.path.join(
            get_package_prefix('qazbot_lidar_projector'), 'lib', 'qazbot_lidar_projector', 'parity_dump')
        with tempfile.TemporaryDirectory() as d:
            canon, out = os.path.join(d, 'canonical.bin'), os.path.join(d, 'cpp.bin')
            fixture.write(canon, replay)
            subprocess.run([dump, canon, out, '8.0', '2'], check=True, timeout=120)   # package YAML values
            return read_dump(out)[2]

    def test_cloud_before_its_odometry_is_not_lost(self):
        replay, n_imu = fixture.canonical_replay(fixture.read(FIXTURE))
        exp = {k: ranges for k, ranges, _, _ in self.expected(replay)}
        self.spin_until(lambda: self.pub_odom.get_subscription_count() == 1 and
                        self.pub_cloud.get_subscription_count() == 1 and
                        self.pub_imu.get_subscription_count() == 1 and self.param_client.service_is_ready())
        self.publish_odom(replay[0])
        for e in replay[1:1 + n_imu]:
            m = Imu()
            m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = e.data
            self.pub_imu.publish(m)
            rclpy.spin_once(self.node, timeout_sec=0)
        self.spin_until(lambda: self.node.count_subscribers(IMU) == 0)   # levelled

        rest = replay[1 + n_imu:]
        odom_by_key = {key_of(e.sec, e.nanosec): e for e in rest if e.kind == 'odom'}
        clouds = [e for e in rest if e.kind == 'cloud']
        swapped = {key_of(e.sec, e.nanosec) for k, e in enumerate(clouds)
                   if k % 10 in (0, 1) and key_of(e.sec, e.nanosec) in odom_by_key}
        swapped_processed = swapped & set(exp)
        self.assertGreaterEqual(len(swapped_processed), 2, 'fixture must exercise the swap on the processed parity')
        for e in rest:
            key = key_of(e.sec, e.nanosec)
            if e.kind == 'odom':
                if key not in swapped:
                    self.publish_odom(e)
                continue
            self.publish_cloud(e)          # swapped: the cloud is taken first, its odometry is not there yet
            if key in swapped:
                self.publish_odom(odom_by_key[key])
        # One writer delivers in order: once the last in-order (not swapped) scan is here, every earlier one is too.
        anchor = max(set(exp) - swapped)
        self.spin_until(lambda: anchor in self.scans)
        lost = sorted(k for k in exp if k <= anchor and k not in self.scans)
        self.assertEqual(lost, [], f'{len(lost)} of {len(exp)} scans lost ({len(swapped_processed)} swapped on the '
                         'processed parity)')
        self.spin_until(lambda: set(exp) <= set(self.scans))
        self.assertEqual(sorted(self.scans), sorted(exp), 'exact stamps only, no extra scans')
        for key, ranges in exp.items():
            np.testing.assert_array_equal(np.array(self.scans[key].ranges, dtype=np.float32), ranges)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
