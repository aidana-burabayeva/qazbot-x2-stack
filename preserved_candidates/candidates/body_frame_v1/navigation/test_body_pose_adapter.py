"""ROS message/buffer tests. Run only with isolated localhost DDS configuration."""
import os
import unittest
import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from body_pose_adapter import BodyPoseAdapter


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def tf(parent,child,ns=1_000_000_000_000,xyz=(0,0,0),q=(0,0,0,1)):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
    t.header.stamp.sec=ns//10**9;t.header.stamp.nanosec=ns%10**9
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
    return TFMessage(transforms=[t])


def odom(ns=1_000_000_000_000,xyz=(2,3.1,.9),yaw=math.pi/2):
    m=Odometry();m.header.frame_id='camera_init';m.child_frame_id='body'
    m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
    m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,xyz)
    m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
    return m


class BodyPoseAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Test requires ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.node=BodyPoseAdapter()
        self.node.pose_pub=Capture();self.node.tf_pub=Capture();self.node.status_pub=Capture()

    def tearDown(self):self.node.destroy_node()

    def level(self):self.node.on_level(tf('odom','camera_init'))
    def kin_static(self):self.node.on_robot_static(tf('base_link','lidar_imu_chest_front',xyz=(.1,0,.9)))

    def test_nominal_sensor_offset_stamp_and_frame(self):
        self.level();self.kin_static();m=odom();self.node.on_odom(m)
        p=self.node.pose_pub.messages[-1];t=self.node.tf_pub.messages[-1].transforms[0]
        self.assertAlmostEqual(p.pose.position.x,2)
        self.assertAlmostEqual(p.pose.position.y,3)
        self.assertEqual(p.header.stamp,m.header.stamp)
        self.assertEqual(t.header,p.header)
        self.assertEqual(t.child_frame_id,'x2_base_footprint')
        self.assertEqual(p.header.frame_id,'odom')
        self.assertEqual(t.transform.translation.z,p.pose.position.z)
        self.assertEqual(self.node.last_state,'body_pose_available')

    def test_missing_data_joins_only_when_available(self):
        self.node.on_odom(odom());self.assertEqual(self.node.last_state,'waiting_level_transform')
        self.level();self.assertEqual(self.node.last_state,'waiting_same_stamp_kinematics')
        self.assertEqual(len(self.node.pose_pub.messages),0)
        self.kin_static();self.assertEqual(len(self.node.pose_pub.messages),1)
        self.kin_static();self.assertEqual(len(self.node.pose_pub.messages),1)

    def test_dynamic_kinematics_interpolate_at_measurement_not_latest(self):
        self.level();t=1_000_000_000_000
        self.node.on_robot_tf(tf('base_link','lidar_imu_chest_front',ns=t,xyz=(.1,0,0)))
        self.node.on_odom(odom(ns=t+100_000_000,xyz=(.2,0,0),yaw=0))
        self.assertEqual(len(self.node.pose_pub.messages),0)
        self.node.on_robot_tf(tf('base_link','lidar_imu_chest_front',ns=t+200_000_000,xyz=(.3,0,0)))
        self.assertAlmostEqual(self.node.pose_pub.messages[-1].pose.position.x,0)

    def test_loss_has_bounded_pending_and_no_stale_fallback(self):
        self.level();self.node.on_odom(odom())
        newer=odom(ns=1_001_000_000_000,xyz=(4,5.1,.9))
        self.node.on_odom(newer)
        self.assertEqual(self.node.superseded_samples,1)
        self.assertEqual(len(self.node.pose_pub.messages),0)
        self.kin_static()
        self.assertEqual(len(self.node.pose_pub.messages),1)
        self.assertEqual(self.node.pose_pub.messages[-1].header.stamp,newer.header.stamp)
        self.assertAlmostEqual(self.node.pose_pub.messages[-1].pose.position.y,5)

    def test_invalid_or_regressed_stamp_does_not_publish(self):
        self.level();self.kin_static();self.node.on_odom(odom())
        self.node.on_odom(odom(ns=999_000_000_000))
        self.assertEqual(len(self.node.pose_pub.messages),1)
        self.assertEqual(self.node.last_state,'nonmonotonic_source_stamp')
        self.node.on_odom(odom(ns=0))
        self.assertEqual(len(self.node.pose_pub.messages),1)
        self.assertEqual(self.node.last_state,'invalid_source_stamp')

    def test_unknown_source_frame_fails_instead_of_relabeling(self):
        m=odom();m.header.frame_id='map'
        with self.assertRaises(ValueError):self.node.on_odom(m)
        self.assertEqual(len(self.node.pose_pub.messages),0)


if __name__=='__main__':unittest.main()
