#!/usr/bin/env python3
"""Timestamped IMU->body pose adapter. Publishes pose/TF only, never velocity.

Uses an event-driven latest-sample join of source odometry, vendor kinematics,
and the bridge's explicit level transform. No extrapolation, stale-TF fallback,
timer, MC service, or fabricated zero twist. Not a complete Nav2 odometry source.
"""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from control_geometry import transform, world_base, planar_pose


def tf_matrix(tf):
    p, q = tf.transform.translation, tf.transform.rotation
    return transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


class BodyPoseAdapter(Node):
    def __init__(self):
        super().__init__('qazbot_body_pose_adapter')
        self.robot = Buffer(); self.level = Buffer()
        self.pending = None; self.last_input_ns = None; self.last_output_ns = None
        self.superseded_samples = 0; self.last_state = None; self.last_report = None
        self.output_tf = self.declare_parameter('output_tf', '/qazbot_d1_map/tf').value
        self.output_pose = self.declare_parameter('output_pose', '/qazbot_body/pose').value
        self.output_status = self.declare_parameter('output_status', '/qazbot_body/status').value
        self.level_topic = self.declare_parameter('level_tf_static', '/qazbot_d1_map/tf_static').value
        self.source_topic = self.declare_parameter('source_odom', '/qazbot_lio/odometry').value
        self.level_frame = 'odom'; self.world_frame = 'camera_init'
        self.base_frame = 'base_link'; self.imu_frame = 'lidar_imu_chest_front'
        self.footprint_frame = 'x2_base_footprint'
        if self.output_tf in ['/tf','/tf_static'] or self.level_topic == '/tf_static':
            raise ValueError('Output and leveling TF must be private, separate from vendor TF')
        self.tf_pub = self.create_publisher(TFMessage, self.output_tf, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose, 10)
        self.status_pub = self.create_publisher(String, self.output_status, 10)
        retained = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', self.on_robot_tf, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.on_robot_static, retained)
        self.create_subscription(TFMessage, self.level_topic, self.on_level, retained)
        self.create_subscription(Odometry, self.source_topic, self.on_odom, qos_profile_sensor_data)

    def state(self, value):
        self.last_state = value
        key = (value,self.last_input_ns,self.last_output_ns,self.superseded_samples)
        if key == self.last_report:return
        self.last_report = key
        self.status_pub.publish(String(data=json.dumps({'state':value, 'source_stamp_ns':self.last_input_ns,
            'output_stamp_ns':self.last_output_ns, 'superseded_samples':self.superseded_samples,
            'qualification':'body pose only; velocity and motion execution not provided'})))

    def on_robot_tf(self, msg):
        for tf in msg.transforms:self.robot.set_transform(tf, 'vendor')
        self.emit_ready_sample()

    def on_robot_static(self, msg):
        for tf in msg.transforms:self.robot.set_transform_static(tf, 'vendor')
        self.emit_ready_sample()

    def on_level(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.level_frame and tf.child_frame_id == self.world_frame:
                if any(abs(v)>1e-12 for v in [tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]):
                    raise ValueError('Expected a pure world-level rotation, with unchanged origin')
                self.level.set_transform_static(tf, 'level_bridge')
        self.emit_ready_sample()

    def on_odom(self, msg):
        if msg.header.frame_id != self.world_frame or msg.child_frame_id != 'body':
            raise ValueError('Unexpected FAST-LIO frame contract')
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp <= 0:
            self.pending=None;self.state('invalid_source_stamp');return
        if self.last_input_ns is not None and stamp <= self.last_input_ns:
            self.pending=None
            self.state('nonmonotonic_source_stamp')
            return
        if self.pending is not None:self.superseded_samples += 1
        self.last_input_ns = stamp; self.pending = msg
        self.emit_ready_sample()

    def emit_ready_sample(self):
        if self.pending is None:return
        msg = self.pending; stamp = Time.from_msg(msg.header.stamp)
        if not self.level.can_transform(self.level_frame,self.world_frame,stamp):
            self.state('waiting_level_transform');return
        if not self.robot.can_transform(self.base_frame,self.imu_frame,stamp):
            self.state('waiting_same_stamp_kinematics');return
        level = tf_matrix(self.level.lookup_transform(self.level_frame,self.world_frame,stamp))[:3,:3]
        base_imu = tf_matrix(self.robot.lookup_transform(self.base_frame,self.imu_frame,stamp))
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        wi = transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        x,y,yaw = planar_pose(level,world_base(wi,base_imu))
        pose = PoseStamped();pose.header.stamp=msg.header.stamp;pose.header.frame_id=self.level_frame
        pose.pose.position.x=float(x);pose.pose.position.y=float(y)
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        tf=TransformStamped();tf.header=pose.header;tf.child_frame_id=self.footprint_frame
        tf.transform.translation.x=float(x);tf.transform.translation.y=float(y);tf.transform.rotation=pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[tf]));self.pose_pub.publish(pose)
        self.last_output_ns=self.last_input_ns;self.pending=None;self.state('body_pose_available')


def main():
    rclpy.init();node=BodyPoseAdapter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
