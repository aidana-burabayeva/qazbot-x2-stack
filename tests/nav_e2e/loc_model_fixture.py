#!/usr/bin/env python3
"""Kinematic MC model + emulated AMCL outputs + fake MC state, for the REAL adapter / REAL trust e2e.

Isolated domain 87 only (refuses otherwise). Publishes NOTHING on /cmd_vel or /aima/mc/locomotion/velocity,
offers no SetMcInputSource service. The only vendor-typed topic is a FAKE /aima/mc/common/state (10 Hz, the
real MC rate) inside domain 87, so the adapter's state machine can run and a PS5 takeover can be emulated.

Model input: <ns>/cmd_vel, which the launcher remaps to <ns>/motion_adapter/dry_run_cmd (adapter output).
Localization emulation (what AMCL would publish): map->odom = loc_offset, amcl_pose and particle_cloud at the
believed pose (= loc_offset (+) true pose). Scan: D1 map ray-cast at the TRUE pose (+ optional box obstacle).
Controls (all in <ns>): fixture_scan_enabled, fixture_odom_enabled, fixture_amcl_enabled (Bool);
fixture_loc_offset (Float64MultiArray dx, dy, dyaw); fixture_mc (String JSON: source/body/fsm/enabled).
"""
import json
import math
import os
import signal
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import Particle, ParticleCloud
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray, String

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/controller_dry'))
sys.path.insert(0, str(ROOT / 'ws/src/navigation'))
from sim_robot import Sim  # noqa: E402
import loc_trust as L  # noqa: E402
sys.path.insert(0, str(ROOT / 'tests/nav_e2e'))
from e2e_map import MAP_YAML, START  # noqa: E402

GRID = L.OccGrid.from_pgm(str(MAP_YAML))
RAY = np.arange(0.05, 8.0, 0.02)                    # 2 cm ray step < map resolution (5 cm)
N_BEAMS = 360


class LocSim(Sim):
    def __init__(self, ns):
        self.enabled = dict(scan=True, odom=True)
        super().__init__(ns)
        a = -math.pi + 2 * math.pi / N_BEAMS * np.arange(N_BEAMS)
        self.beam_angles = a
        self.ray_xy = np.stack([np.outer(np.cos(a), RAY), np.outer(np.sin(a), RAY)], -1).reshape(-1, 2)

    def pub_scan(self):
        if not self.enabled['scan']:
            return
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), 'base_footprint'
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / N_BEAMS
        s.angle_max = s.angle_min + s.angle_increment * (N_BEAMS - 1)
        s.range_min, s.range_max = 0.05, 8.0
        pose = (self.x, self.y, self.yaw)
        hit = (GRID.dist_at(L.to_map(self.ray_xy, pose)) <= 0.025).reshape(N_BEAMS, len(RAY))
        r = np.where(hit.any(1), RAY[hit.argmax(1)], np.inf)
        if self.wall_x >= 0:
            world = self.yaw + self.beam_angles
            c = np.cos(world)
            with np.errstate(divide='ignore', invalid='ignore'):
                t = np.where(c > 1e-6, (self.wall_x - self.x) / c, np.inf)
            yh = self.y + t * np.sin(world)
            ok = (t > 0) & (np.abs(yh - self.wall_y) <= self.wall_half)
            r = np.where(ok, np.minimum(r, t), r)
        s.ranges = [float(v) if v <= s.range_max else float('inf') for v in r]
        self.scan_pub.publish(s)


class Toggle:
    """Odometry output that can be cut; the model's ground truth stays observable on <ns>/model_truth."""
    def __init__(self, pub, flag, truth):
        self.pub, self.flag, self.truth = pub, flag, truth

    def publish(self, m):
        self.truth.publish(m)
        if self.flag():
            self.pub.publish(m)


class Emulator(Node):
    def __init__(self, ns, sim):
        super().__init__('loc_mc_emulator', namespace=ns)
        from aimdk_msgs.msg import McCommonState
        self.sim, self.McCommonState = sim, McCommonState
        self.amcl_on = True
        self.mc = dict(source='sdk_qazbot_nav2', body=1, fsm=4, enabled=True)
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = self.create_publisher(OccupancyGrid, 'map', tl)
        self.map.publish(self._grid())
        self.amcl = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', tl)
        self.cloud = self.create_publisher(ParticleCloud, 'particle_cloud', 10)
        self.mc_pub = self.create_publisher(McCommonState, '/aima/mc/common/state',
                                            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.rng = np.random.default_rng(3)
        sub = self.create_subscription
        sub(Bool, 'fixture_scan_enabled', lambda m: sim.enabled.__setitem__('scan', m.data), 1)
        sub(Bool, 'fixture_odom_enabled', lambda m: sim.enabled.__setitem__('odom', m.data), 1)
        sub(Bool, 'fixture_amcl_enabled', self._amcl_enabled, 1)
        sub(Float64MultiArray, 'fixture_loc_offset', lambda m: setattr(sim, 'map_odom', tuple(m.data[:3])), 1)
        sub(String, 'fixture_mc', lambda m: self.mc.update(json.loads(m.data)), 1)
        self.create_timer(0.1, self._amcl_tick)       # AMCL update rate is scan-driven (10 Hz scan here)
        self.create_timer(0.1, self._mc_tick)         # McCommonState is 10 Hz on the robot

    def _grid(self):
        import cv2
        import yaml
        y = yaml.safe_load(MAP_YAML.read_text())
        a = np.flipud(cv2.imread(str(MAP_YAML.parent / y['image']), 0))
        prob = (255 - a.astype(float)) / 255
        data = np.full(a.shape, -1, dtype=np.int8)
        data[prob < y['free_thresh']] = 0
        data[prob > y['occupied_thresh']] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = float(y['resolution'])
        m.info.width, m.info.height = a.shape[1], a.shape[0]
        m.info.origin.position.x, m.info.origin.position.y = float(y['origin'][0]), float(y['origin'][1])
        m.info.origin.orientation.w = 1.0
        m.data = data.ravel().tolist()
        return m

    def _amcl_enabled(self, m):
        self.amcl_on = m.data
        self.sim.publish_map_odom = m.data            # AMCL is the only map->odom publisher

    def _amcl_tick(self):
        if not self.amcl_on:
            return
        s = self.sim
        x, y, yaw = L.compose(s.map_odom, (s.x, s.y, s.yaw))
        st = self.get_clock().now().to_msg()
        a = PoseWithCovarianceStamped()
        a.header.stamp, a.header.frame_id = st, 'map'
        a.pose.pose.position.x, a.pose.pose.position.y = x, y
        a.pose.pose.orientation.z, a.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        a.pose.covariance[0] = a.pose.covariance[7] = 0.01
        a.pose.covariance[35] = 0.004
        self.amcl.publish(a)
        c = ParticleCloud()
        c.header.stamp, c.header.frame_id = st, 'map'
        for dx, dy in self.rng.normal(0, 0.05, (100, 2)):
            q = Particle()
            q.pose.position.x, q.pose.position.y, q.weight = x + dx, y + dy, 0.01
            c.particles.append(q)
        self.cloud.publish(c)

    def _mc_tick(self):
        if not self.mc.get('enabled', True):
            return
        m = self.McCommonState()
        m.body_status.value, m.fsm_state.current_state = int(self.mc['body']), int(self.mc['fsm'])
        m.input_source.name = self.mc['source']
        sp = m.speed_status
        sp.speed_mode.value = 1
        sp.forward_bounds.min_value, sp.forward_bounds.max_value = -1.0, 0.8
        sp.lateral_bounds.min_value, sp.lateral_bounds.max_value = -1.0, 1.0
        sp.angular_bounds.min_value, sp.angular_bounds.max_value = -1.0, 1.0
        self.mc_pub.publish(m)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('loc model fixture requires isolated domain 87, localhost only')
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped = [False]
    signal.signal(signal.SIGINT, lambda *a: stopped.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *a: stopped.__setitem__(0, True))
    ns = sys.argv[1]
    sim = LocSim(ns)
    sim.x, sim.y, sim.yaw, sim.wall_y = START[0], START[1], START[2], START[1]
    from nav_msgs.msg import Odometry
    sim.odom_pub = Toggle(sim.odom_pub, lambda: sim.enabled['odom'], sim.create_publisher(Odometry, 'model_truth', 10))
    em = Emulator(ns, sim)
    ex = SingleThreadedExecutor()
    ex.add_node(sim)
    ex.add_node(em)
    try:
        while not stopped[0] and rclpy.ok():
            ex.spin_once(timeout_sec=.05)
    finally:
        sim.destroy_node()
        em.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
