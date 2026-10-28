#!/usr/bin/env python3
"""Plant bridge: OFFICIAL MuJoCo body (driven by the OFFICIAL mc) -> the QazBot navigation stack's inputs.

Replaces what FAST-LIO + level_bridge give the stack on the robot, in the twin's private namespace:
  /aima/hal/odom/state (world pose of the simulated pelvis, truth)  ->  <ns>/odom, TF odom->base_footprint, <ns>/model_truth
  D1-map ray-cast of the 2-D scan from that pose                     ->  <ns>/scan   (360 beams, same geometry as the other twin plants)
Nothing here moves the robot: motion comes only from mc (via /aima/mc/locomotion/velocity). map->odom is NOT published
(the REAL AMCL owns it). Limits (documented, not hidden): the pose is the pelvis, not the chest-LiDAR IMU point LIO tracks
(~0.1 m planar offset on the robot); odometry is MuJoCo truth (no drift, no LIO artefacts); the scan is an ideal ray-cast
(the robot's is a ~100 deg wedge); MuJoCo has a flat floor without the map's walls, so only the scan "sees" them.
The sim world frame is mapped so that the first received pose equals e2e_map.START in map/odom.
Usage: mc_plant_bridge.py NS     (SIM container only)
"""
import math
import os
import signal
import sys
from pathlib import Path

if os.environ.get('QAZBOT_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('plant bridge runs only in the isolated SIM (source tools/twin/env_sim.sh or env_container.sh)')
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests/nav_e2e'), str(ROOT / 'tests/controller_dry'), str(ROOT / 'ws/src/navigation')]

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_msgs.msg import TFMessage

from e2e_map import START                      # noqa: E402
from loc_model_fixture import LocSim           # noqa: E402  (reuses the D1 ray-cast scan)

wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def planar_pose(msg):
    """x, y, heading of the body x-axis projected on the floor (valid while the body is upright)."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    r00 = 1 - 2 * (q.y * q.y + q.z * q.z)
    r10 = 2 * (q.x * q.y + q.z * q.w)
    return p.x, p.y, math.atan2(r10, r00), p.z


class HalPlant(LocSim):
    def __init__(self, ns, grid=None, start=None, fixed_world=False):
        self.start = START if start is None else start
        super().__init__(ns, grid=grid)
        self.publish_map_odom = False            # the real AMCL is the only map->odom publisher
        self.hal = None                          # (t, x, y, yaw) in the sim world
        self.last_emitted_stamp = None
        self.p0 = (0., 0., 0.) if fixed_world else None
        self.history = []                        # (t, x, y, yaw) in odom for finite-difference velocity
        self.truth = self.create_publisher(Odometry, 'model_truth', 10)
        self.create_subscription(Odometry, '/aima/hal/odom/state', self.on_hal, qos_profile_sensor_data)

    def on_hal(self, m):
        source=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if not math.isfinite(source) or source<=0 or (self.hal is not None and source<=self.hal[0]):return
        x, y, yaw, z = planar_pose(m)
        if not all(math.isfinite(v) for v in (x,y,yaw,z)):return
        self.hal=(source,m.header.stamp)
        if self.p0 is None:
            self.p0 = (x, y, yaw)
        c, s = math.cos(-self.p0[2]), math.sin(-self.p0[2])
        rx, ry = c * (x - self.p0[0]) - s * (y - self.p0[1]), s * (x - self.p0[0]) + c * (y - self.p0[1])
        sc, ss = math.cos(self.start[2]), math.sin(self.start[2])
        self.x = self.start[0] + sc * rx - ss * ry
        self.y = self.start[1] + ss * rx + sc * ry
        self.yaw = wrap(self.start[2] + wrap(yaw - self.p0[2]))
        self.z = z

    def sample_stamp(self):
        if self.hal is None:return None
        age=self.get_clock().now().nanoseconds*1e-9-self.hal[0]
        return self.hal[1] if -.05<=age<=.2 else None

    def step(self):                              # no dynamics: publish what the official sim says
        stamp=self.sample_stamp()
        if self.p0 is None or stamp is None or self.hal[0]==self.last_emitted_stamp:
            return
        t = self.hal[0]
        self.last_emitted_stamp=t
        self.history.append((t, self.x, self.y, self.yaw))
        while len(self.history) > 2 and t - self.history[0][0] > 0.2:
            self.history.pop(0)
        t0, x0, y0, w0 = self.history[0]
        dt = t - t0
        if dt > 0.05:
            vx, vy = (self.x - x0) / dt, (self.y - y0) / dt
            self.v = vx * math.cos(self.yaw) + vy * math.sin(self.yaw)
            self.w = wrap(self.yaw - w0) / dt
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)
        self.truth.publish(o)

    def pub_scan(self):
        if self.p0 is not None and self.sample_stamp() is not None:
            super().pub_scan(stamp=self.sample_stamp())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = [False]
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__(0, True))
    plant = HalPlant(sys.argv[1])
    ex = SingleThreadedExecutor()
    ex.add_node(plant)
    try:
        while not stop[0] and rclpy.ok():
            ex.spin_once(timeout_sec=.02)
    finally:
        plant.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
