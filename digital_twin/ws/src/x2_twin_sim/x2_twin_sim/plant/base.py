"""Common plant node: publishes what FAST-LIO + ``level_bridge`` publish on the robot, from a simulated body.

Topic names, rates, QoS and frame conventions mirror ``ws/src/lio/level_bridge.py`` so the navigation stack runs unmodified:
  * ``/qazbot_lio/odom_level``  nav_msgs/Odometry, frame ``odom`` -> ``base_footprint``, twist in the child frame (REP-105)
  * ``/qazbot_d1_map/tf``       odom -> base_footprint (the private TF graph the stack uses; map -> odom belongs to AMCL)
  * ``/qazbot_lio/scan``        sensor_msgs/LaserScan in ``base_footprint``
  * ``scan_obstacles`` / ``scan_clearing`` PointCloud2, finite planar observations only (not calibrated E1R clouds)
Subclasses implement :meth:`current_state`.
"""
import json
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from x2_twin_sim.geometry import Pose2D
from x2_twin_sim.guards import require_sim_environment
from x2_twin_sim.level_scan import build_ranges, coverage_bins
from x2_twin_sim.occupancy import OccupancyMap, RayCaster
from x2_twin_sim.sensor_observations import bounded_observations

State = Tuple[Pose2D, float, float]   # pose in odom, forward speed, yaw rate


class PlantNode(Node):
    def __init__(self, name: str):
        require_sim_environment()
        super().__init__(name)
        p = self.declare_parameter
        self.tf_topic = p('tf_topic', '/qazbot_d1_map/tf').value
        self.odom_topic = p('odom_topic', '/qazbot_lio/odom_level').value
        self.scan_topic = p('scan_topic', '/qazbot_lio/scan').value
        self.truth_topic = p('truth_topic', '/x2_twin/model_truth').value
        self.map_yaml = p('map_yaml', '').value
        self.start_pose = tuple(float(v) for v in p('start_pose', [-1.275, -4.825, 0.0]).value)
        odom_hz, scan_hz = float(p('odom_hz', 10.0).value), float(p('scan_hz', 5.0).value)
        if not self.map_yaml:
            raise ValueError('parameter map_yaml is required')
        self.caster = RayCaster(OccupancyMap.from_yaml(self.map_yaml), n_beams=int(p('scan_n_beams', 1440).value),
                                range_min=float(p('scan_range_min', 0.45).value), range_max=float(p('scan_range_max', 8.0).value),
                                fov_deg=float(p('scan_fov_deg', 103.0).value))
        self.tf_pub = self.create_publisher(TFMessage, self.tf_topic, QoSProfile(depth=100))
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 50)
        self.truth_pub = self.create_publisher(Odometry, self.truth_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.obstacle_pub = self.create_publisher(PointCloud2, self.scan_pub.topic_name + '_obstacles', 20)
        self.clearing_pub = self.create_publisher(PointCloud2, self.scan_pub.topic_name + '_clearing', 20)
        # Scan semantics: legacy = what level_bridge publishes today (inf = no return, everywhere); i1 = the I1 candidate
        # (finite / +inf observed-free inside the FOV / NaN unknown). Scenario commands: transient discs in the map and scan dropouts.
        self.scan_format = str(p('scan_format', 'legacy').value)
        if self.scan_format not in ('legacy', 'i1'):
            raise ValueError("scan_format must be 'legacy' or 'i1'")
        self.ghosts, self.scan_mode = [], 'normal'            # scan_mode: normal | none (no message) | empty (no returns at all)
        self.create_subscription(String, 'x2_twin/scenario_cmd', self._on_cmd, 10)
        # Acknowledgement = the plant's applied state (with the command's seq): the scenario proves delivery instead of re-sending.
        self.state_pub = self.create_publisher(String, 'x2_twin/scenario_state', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_seq = 0
        self._publish_state()
        self.create_timer(1.0 / odom_hz, self._publish_odom)
        self.create_timer(1.0 / scan_hz, self._publish_scan)

    def current_state(self) -> Optional[State]:
        raise NotImplementedError

    def _publish_odom(self) -> None:
        state = self.current_state()
        if state is None:
            return
        (x, y, yaw), v, w = state
        stamp = self.get_clock().now().to_msg()
        q_z, q_w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = q_z, q_w
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q_z, q_w
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odom_pub.publish(odom)
        self.truth_pub.publish(odom)

    def _on_cmd(self, m: String) -> None:
        cmd = json.loads(m.data)
        if 'ghosts' in cmd:
            self.ghosts = [(float(g['x']), float(g['y']), float(g['r'])) for g in cmd['ghosts']]
        if 'scan' in cmd:
            mode = str(cmd['scan'])
            if mode not in ('normal', 'none', 'empty') and not mode.startswith('blind:'):
                raise ValueError(mode)                       # blind:LO:HI = bearings LO..HI (deg) deliver no point (blocked sector)
            self.scan_mode = mode
        self.cmd_seq = int(cmd.get('seq', self.cmd_seq))
        self._publish_state()

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=json.dumps(dict(seq=self.cmd_seq, ghosts=self.ghosts, scan=self.scan_mode,
                                                           scan_format=self.scan_format))))

    def _publish_scan(self) -> None:
        state = self.current_state()
        if state is None or self.scan_mode == 'none':
            return
        ranges = self.caster.scan(state[0], self.ghosts)  # map == odom at the start (AMCL owns the correction)
        bearing_all = self.caster.angle_min + self.caster.angle_increment * np.arange(self.caster.n)
        sensor = np.zeros(self.caster.n, bool)               # bearings for which the sensor delivers points at all (its sector)
        sensor[self.caster.observed] = True
        if self.scan_mode == 'empty':
            ranges = np.full_like(ranges, np.inf)
            sensor[:] = False
        elif self.scan_mode.startswith('blind:'):
            lo, hi = (float(v) for v in self.scan_mode.split(':')[1:3])
            blocked = (np.degrees(bearing_all) >= lo - 1e-9) & (np.degrees(bearing_all) <= hi + 1e-9)
            ranges[blocked] = np.inf
            sensor[blocked] = False
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        scan.angle_min, scan.angle_increment = self.caster.angle_min, self.caster.angle_increment
        scan.angle_max = scan.angle_min + scan.angle_increment * (self.caster.n - 1)
        scan.range_min, scan.range_max = float(self.caster.rmin), float(self.caster.rmax)
        ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        if self.scan_format == 'i1':
            fin = np.isfinite(ranges)
            ranges = build_ranges(ranges[fin], bearing_all[fin], coverage_bins(bearing_all[sensor], self.caster.n), self.caster.n)
        scan.ranges = ranges.astype(float).tolist()
        mark, clear = bounded_observations(ranges, bearing_all, sensor, self.caster.rmin, self.caster.rmax)
        self.scan_pub.publish(scan)
        self.obstacle_pub.publish(pc2.create_cloud_xyz32(scan.header, mark.tolist()))
        self.clearing_pub.publish(pc2.create_cloud_xyz32(scan.header, clear.tolist()))


def spin(node_factory) -> None:
    """Run a node until SIGINT/SIGTERM and shut down cleanly (shared ``main`` for plant nodes)."""
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
