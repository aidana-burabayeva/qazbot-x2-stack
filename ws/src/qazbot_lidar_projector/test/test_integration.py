"""Integration test of the lidar_projector node through its XML launch file, in an isolated DDS domain.

Launches `launch/lidar_projector.launch.xml` (package YAML as is: range_max 8.0, scan_every 2, publish_tf false,
production remaps /qazbot_lio/*), feeds the recorded fixture (lidar IMU + LIO odometry + 50 clouds with the real
48-byte FAST-LIO point layout) and checks:
  * topics, message types, frame_ids, stamps, publisher QoS (RELIABLE / VOLATILE) and that there are exactly the
    outputs of the Python node (scan, scan_obstacles, scan_clearing, odom_level) and no TF publisher;
  * the published data equals the C++ library dump (which test_parity.py proves equal to the Python candidate);
  * the lidar IMU subscription is dropped once levelled.
No sleeps: every wait is `spin_until(condition, timeout)` on a message / graph event.
"""
import os
import struct
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
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


def stamp_key(h):
    return h.stamp.sec * 10 ** 9 + h.stamp.nanosec


class TestLidarProjector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('lidar_projector_test')
        be = QoSProfile(depth=400, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        cls.pub_imu = cls.node.create_publisher(Imu, IMU, be)
        cls.pub_odom = cls.node.create_publisher(Odometry, '/qazbot_lio/odometry', 100)
        cls.pub_cloud = cls.node.create_publisher(PointCloud2, '/qazbot_lio/cloud_registered', 20)
        cls.param_client = cls.node.create_client(GetParameters, f'/{NODE}/get_parameters')
        cls.scans, cls.obs, cls.clr, cls.odom = {}, {}, {}, []
        qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        cls.node.create_subscription(LaserScan, '/qazbot_lio/scan', lambda m: cls.scans.__setitem__(stamp_key(m.header), m), qos)
        cls.node.create_subscription(PointCloud2, '/qazbot_lio/scan_obstacles',
                                     lambda m: cls.obs.__setitem__(stamp_key(m.header), m), qos)
        cls.node.create_subscription(PointCloud2, '/qazbot_lio/scan_clearing',
                                     lambda m: cls.clr.__setitem__(stamp_key(m.header), m), qos)
        cls.node.create_subscription(Odometry, '/qazbot_lio/odom_level', lambda m: cls.odom.append(m), qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, cond, timeout=DEADLINE_S):
        end = time.monotonic() + timeout
        while not cond():
            self.assertLess(time.monotonic(), end, f'timeout waiting for condition (odom_level so far {len(self.odom)})')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_odom(self, e):
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
        m.header.frame_id, m.child_frame_id = 'camera_init', 'body'
        (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) = e.data[0]
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
         m.pose.pose.orientation.w) = e.data[1]
        m.pose.covariance = list(e.data[2])
        self.pub_odom.publish(m)

    def barrier(self):
        fut = self.param_client.call_async(GetParameters.Request(names=['range_max']))
        self.spin_until(fut.done)

    def expected(self):
        dump = os.environ.get('PARITY_DUMP') or os.path.join(
            get_package_prefix('qazbot_lidar_projector'), 'lib', 'qazbot_lidar_projector', 'parity_dump')
        with tempfile.TemporaryDirectory() as d:
            out, canon = os.path.join(d, 'cpp.bin'), os.path.join(d, 'canonical.bin')
            fixture.write(canon, self.replay)
            # package YAML values: range_max 8.0, scan_every 2
            subprocess.run([dump, canon, out, '8.0', '2'], check=True, timeout=120)
            return read_dump(out)

    def test_end_to_end(self):
        events = fixture.read(FIXTURE)
        self.replay, n_imu = fixture.canonical_replay(events)
        _, exp_odoms, exp_clouds = self.expected()
        exp_keys = [k for k, *_ in exp_clouds]
        self.assertGreater(len(exp_keys), 15)

        # graph: the node is up with its publishers/subscribers (discovery events)
        # node-name discovery (participant data) can lag endpoint discovery: wait for it before by-node queries
        self.spin_until(lambda: NODE in {n for n, ns in self.node.get_node_names_and_namespaces() if ns == '/'} and
                        self.node.count_subscribers('/qazbot_lio/odometry') == 1 and
                        self.node.count_subscribers('/qazbot_lio/cloud_registered') == 1 and
                        self.node.count_subscribers(IMU) == 1 and
                        self.node.count_publishers('/qazbot_lio/scan') == 1 and
                        self.pub_odom.get_subscription_count() == 1 and self.pub_cloud.get_subscription_count() == 1 and
                        self.pub_imu.get_subscription_count() == 1 and self.param_client.service_is_ready())

        # publisher QoS and exact output set of the node
        for t in ('/qazbot_lio/scan', '/qazbot_lio/scan_obstacles', '/qazbot_lio/scan_clearing', '/qazbot_lio/odom_level'):
            infos = self.node.get_publishers_info_by_topic(t)
            self.assertEqual(len(infos), 1, t)
            self.assertEqual(infos[0].qos_profile.reliability, QoSReliabilityPolicy.RELIABLE, t)
            self.assertEqual(infos[0].qos_profile.durability, QoSDurabilityPolicy.VOLATILE, t)
        pubs = {n for n, _ in self.node.get_publisher_names_and_types_by_node(NODE, '/')}
        self.assertEqual(pubs - {'/rosout', '/parameter_events'},
                         {'/qazbot_lio/scan', '/qazbot_lio/scan_obstacles', '/qazbot_lio/scan_clearing',
                          '/qazbot_lio/odom_level'})
        self.assertEqual(self.node.count_publishers('/qazbot_d1_map/tf'), 0, 'publish_tf false: no TF publisher')
        self.assertEqual(self.node.count_publishers('/tf'), 0)
        subs = {n for n, _ in self.node.get_subscriber_names_and_types_by_node(NODE, '/')}
        self.assertEqual(subs - {'/parameter_events'},
                         {'/qazbot_lio/odometry', '/qazbot_lio/cloud_registered', IMU})

        # Replay (canonical order, see fixture.canonical_replay). `barrier()` is a parameter-service round trip: the
        # node's single-threaded executor serves subscriptions before services, so once it answers, everything
        # published before the request has been processed. Waits for outputs are message/graph events.
        keys = set(exp_keys)
        first = self.replay[0]
        self.publish_odom(first)
        self.barrier()
        for e in self.replay[1:1 + n_imu]:
            m = Imu()
            m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = e.data
            self.pub_imu.publish(m)
            rclpy.spin_once(self.node, timeout_sec=0)
        self.spin_until(lambda: self.node.count_subscribers(IMU) == 0)   # levelled: IMU subscription dropped
        for e in self.replay[1 + n_imu:]:
            if e.kind == 'odom':
                self.publish_odom(e)
                self.barrier()
                continue
            xyz, dense = e.data
            m = PointCloud2()
            m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
            m.header.frame_id = 'camera_init'
            m.height, m.width = 1, len(xyz)
            # FAST-LIO layout: 48-byte points, x y z at 0/4/8, padding and other fields in between
            names = [('x', 0), ('y', 4), ('z', 8), ('intensity', 32), ('normal_x', 16), ('normal_y', 20),
                     ('normal_z', 24), ('curvature', 36)]
            m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1) for n, o in names]
            m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 48, 48 * len(xyz), dense
            raw = np.zeros((len(xyz), 12), dtype='<f4')
            raw[:, :3] = xyz
            raw[:, 3:8] = 123.0   # garbage in the unused fields must not matter
            m.data = raw.tobytes()
            self.pub_cloud.publish(m)
            key = e.sec * 10 ** 9 + e.nanosec
            if key in keys:
                self.spin_until(lambda: key in self.scans and key in self.obs and key in self.clr)
        self.spin_until(lambda: len(self.odom) >= len(exp_odoms))

        # exactly the expected clouds (scan_every 2) were published, nothing else
        self.assertEqual(sorted(self.scans), sorted(exp_keys))
        self.assertEqual(sorted(self.obs), sorted(exp_keys))
        self.assertEqual(sorted(self.clr), sorted(exp_keys))
        for key, ranges, obs, clr in exp_clouds:
            s = self.scans[key]
            self.assertEqual(s.header.frame_id, 'base_footprint')
            self.assertEqual((s.angle_min, s.range_min, s.range_max), (np.float32(-np.pi), np.float32(0.45), 8.0))
            self.assertAlmostEqual(s.angle_increment, 2 * np.pi / 1440, places=7)
            self.assertEqual(len(s.ranges), 1440)
            np.testing.assert_array_equal(np.array(s.ranges, dtype=np.float32), ranges)
            for msg, ref in ((self.obs[key], obs), (self.clr[key], clr)):
                self.assertEqual(msg.header.frame_id, 'base_footprint')
                self.assertEqual((msg.height, msg.width, msg.point_step, msg.is_dense), (1, len(ref), 12, False))
                self.assertEqual([(f.name, f.offset, f.datatype) for f in msg.fields],
                                 [('x', 0, 7), ('y', 4, 7), ('z', 8, 7)])
                np.testing.assert_array_equal(np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 3), ref)
        by_key = {stamp_key(m.header): m for m in self.odom}
        self.assertEqual(sorted(by_key), sorted(k for k, _, _ in exp_odoms))
        for key, (x, y, z, yaw, vx, vy, wz), cov in exp_odoms:
            m = by_key[key]
            self.assertEqual((m.header.frame_id, m.child_frame_id), ('odom', 'base_footprint'))
            self.assertEqual((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z), (x, y, z))
            self.assertEqual((m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z), (vx, vy, wz))
            np.testing.assert_array_equal(np.array(m.pose.covariance).reshape(6, 6), cov)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
