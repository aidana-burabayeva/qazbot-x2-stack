#!/usr/bin/env python3
"""Official pelvis truth -> separate delayed chest observation approximating recorded .66 inputs.
No MC commands. No simulated FAST-LIO, drift, 3D geometry or calibrated contact dynamics.
Requires a measured profile, vendor FK and isolated domain 87. Never falls back to pelvis as chest.
"""
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from collections import deque
import numpy as np
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):
    raise RuntimeError('isolated SIM only')
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tests/nav_e2e'),str(ROOT/'ws/src/navigation')]
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformException
from control_geometry import transform
from e2e_map import START,MAP_YAML
from loc_trust import OccGrid
from sensor_model import observe,TwistWindow,WedgeRays,wrap


def matrix_pose(p):
    v,q=p.position,p.orientation
    return transform([v.x,v.y,v.z],[q.x,q.y,q.z,q.w])


def matrix_tf(t):
    v,q=t.translation,t.rotation
    return transform([v.x,v.y,v.z],[q.x,q.y,q.z,q.w])


def planar(T):
    return np.array([T[0,3],T[1,3],math.atan2(T[1,0],T[0,0])])


def odometry(stamp,pose,twist=None):
    m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_footprint'
    m.pose.pose.position.x,m.pose.pose.position.y=map(float,pose[:2])
    m.pose.pose.orientation.z,m.pose.pose.orientation.w=math.sin(pose[2]/2),math.cos(pose[2]/2)
    if twist is not None:
        m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z=map(float,twist)
    return m


class Robot66Plant(Node):
    def __init__(self,ns,profile):
        super().__init__('robot66_plant',namespace=ns)
        self.profile=json.loads(Path(profile).read_text());self.config=self.profile['model']
        self.grid=OccGrid.from_pgm(str(MAP_YAML));self.rays=WedgeRays(self.profile)
        self.imu_from_lidar=np.array(self.profile['imu_from_lidar'])
        self.tf=Buffer(cache_time=Duration(seconds=3));self.pending=deque(maxlen=1500)
        self.origin=None;self.sample=None;self.latest_stamp=None;self.obs_t=-math.inf;self.scan_t=-math.inf
        self.last_source=None;self.fk_failures=0;self.observations=0;self.last_age=None;self.clock_resets=0
        self.twist=TwistWindow(self.config['twist_baseline_s']);self.truth_twist=TwistWindow(.2)
        self.pub_truth=self.create_publisher(Odometry,'model_truth',10)
        self.pub_observation_truth=self.create_publisher(Odometry,'model_observation_truth',10)
        self.pub_odom=self.create_publisher(Odometry,'odom',10)
        self.pub_tf=self.create_publisher(TFMessage,'tf',10)
        self.pub_scan=self.create_publisher(LaserScan,'scan',5)
        self.pub_status=self.create_publisher(String,'twin/status',10)
        self.create_subscription(TFMessage,'/tf',self.dynamic,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf_static',self.static,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/aima/hal/odom/state',self.hal,qos_profile_sensor_data)
        self.create_timer(.01,self.tick);self.create_timer(1.,self.status)

    def dynamic(self,m):
        for t in m.transforms:self.tf.set_transform(t,'official_mc')

    def static(self,m):
        for t in m.transforms:self.tf.set_transform_static(t,'official_mc')

    def hal(self,m):
        T=matrix_pose(m.pose.pose);p=planar(T)
        if self.origin is None:
            yaw=START[2]-p[2];c,s=math.cos(yaw),math.sin(yaw)
            self.origin=np.eye(4);self.origin[:2,:2]=[[c,-s],[s,c]]
            self.origin[:2,3]=np.array(START[:2])-self.origin[:2,:2]@p[:2]
        T=self.origin@T;pose=planar(T);stamp=m.header.stamp;t=stamp.sec+stamp.nanosec*1e-9
        if self.last_source is not None and t==self.last_source:return
        if self.last_source is not None and t<self.last_source:
            self.pending.clear();self.twist=TwistWindow(self.config['twist_baseline_s']);self.clock_resets+=1
        self.last_source=t
        self.pending.append((time.monotonic(),stamp,T))
        # Keep true pelvis pose separately; never substitute it for the chest observation.
        twist=self.truth_twist.add(t,pose)
        truth=odometry(stamp,pose,twist);truth.child_frame_id='pelvis'
        self.pub_truth.publish(truth)

    def tick(self):
        now=time.monotonic()
        if now-self.obs_t<1/self.config['odom_hz']:return
        ready=None
        while self.pending and self.pending[0][0]<=now-self.config['latency_s']:
            ready=self.pending.popleft()
        if ready is None:return  # no restamping frozen odometry
        received,stamp,T=ready
        try:
            tr=self.tf.lookup_transform('pelvis','lidar_chest_front',Time.from_msg(stamp))
        except TransformException:
            self.fk_failures+=1;return
        pose=observe(T,matrix_tf(tr.transform),self.imu_from_lidar)
        source_t=stamp.sec+stamp.nanosec*1e-9
        self.sample=(stamp,pose);self.obs_t=now;self.observations+=1
        self.pub_observation_truth.publish(odometry(stamp,pose))
        self.last_age=(self.get_clock().now().nanoseconds*1e-9)-source_t
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='odom';tf.child_frame_id='base_footprint'
        tf.transform.translation.x,tf.transform.translation.y=map(float,pose[:2]);tf.transform.rotation.z,tf.transform.rotation.w=math.sin(pose[2]/2),math.cos(pose[2]/2)
        self.pub_tf.publish(TFMessage(transforms=[tf]))
        twist=self.twist.add(source_t,pose)
        if twist is not None:self.pub_odom.publish(odometry(stamp,pose,twist))
        if now-self.scan_t>=1/self.config['scan_hz']:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint';g=self.profile['scan']
            scan.angle_min=g['angle_min'];scan.angle_increment=g['angle_increment'];scan.angle_max=g['angle_min']+(g['bins']-1)*g['angle_increment']
            scan.range_min=g['range_min'];scan.range_max=g['range_max']
            scan.ranges=self.rays.cast(pose,self.grid.dist_at).astype(np.float32).tolist()
            self.pub_scan.publish(scan);self.scan_t=now

    def status(self):
        current_age=None if self.sample is None else self.get_clock().now().nanoseconds*1e-9-(self.sample[0].sec+self.sample[0].nanosec*1e-9)
        m=String();m.data=json.dumps(dict(profile=self.profile['id'],qualification=self.profile['qualification'],
            observations=self.observations,fk_failures=self.fk_failures,source_age_s=current_age,source_age_at_publication_s=self.last_age,clock_resets=self.clock_resets,
            odom_point='lidar IMU on moving chest',truth_point='pelvis',scan='2D map wedge; no raw FAST-LIO',drift='none'))
        self.pub_status.publish(m)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);stop=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    n=Robot66Plant(sys.argv[1],sys.argv[2])
    try:
        while not stop[0] and rclpy.ok():rclpy.spin_once(n,timeout_sec=.01)
    finally:n.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
