#!/usr/bin/env python3
"""Operator panel: read-only diagnostics + 10 Hz trace for an interactive SIM session (runs in its own container).

It NEVER publishes a pose, a goal, a velocity or a service request to the navigation stack and changes no Trust threshold.
It only
  * explains a seed: Trust's own soft_score of the operator's initialpose against the REAL scan and the D1 map (thresholds are the
    existing TrustConfig values, read-only) and the best nearby pose, so a seed that does not fit the scan is visible;
  * shows Trust / coordinator / adapter state as RViz text (Marker);
  * writes /results/live_status.json (1 Hz), /results/trace.jsonl (10 Hz) and /results/events.jsonl (seeds, goals, state changes).
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker

sys.path.insert(0, os.environ.get('TWIN_STACK_DIR', '/opt/x2twin/stack') + '/navigation')
import loc_trust as L  # noqa: E402

try:
    from aimdk_msgs.msg import McLocomotionVelocity
except ImportError:                      # panel still works without the MC command trace
    McLocomotionVelocity = None

NS = '/qazbot_d1_map'
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Panel(Node):
    def __init__(self, args):
        super().__init__('operator_panel')
        self.out = Path(args.out)
        self.cfg = L.TrustConfig()                      # read-only: thresholds are only shown, never changed
        self.grid = L.OccGrid.from_pgm(args.map_yaml)
        self.events = (self.out / 'events.jsonl').open('a', buffering=1)
        self.trace = (self.out / 'trace.jsonl').open('a', buffering=1)
        self.trust, self.coord, self.adapter = {}, {}, {}
        self.truth = self.amcl = self.odom = None
        self.cmd_nav = self.cmd_auth = self.mc_cmd = None
        self.scans = []                                 # (t, endpoints) recent
        self.seed = None                                # dict(t, pose, diag)
        self.last_goal = None
        self.nav_status = {}                            # goal uuid -> last status code (action_msgs/GoalStatus)
        self.nav_code = None
        self.truth_hist = []
        self.prev_key = {}
        self.t0 = time.time()
        qos = QoSProfile(depth=10)
        self.create_subscription(String, NS + '/localization_trust/status', lambda m: self.on_status('trust', m), qos)
        self.create_subscription(String, NS + '/goal_coordinator/status', lambda m: self.on_status('coord', m), qos)
        self.create_subscription(String, NS + '/motion_adapter/status', lambda m: self.on_status('adapter', m), qos)
        self.create_subscription(PoseWithCovarianceStamped, NS + '/initialpose', self.on_seed, qos)
        self.create_subscription(PoseWithCovarianceStamped, NS + '/amcl_pose', self.on_amcl, qos)
        self.create_subscription(PoseStamped, NS + '/goal_pose', self.on_goal, qos)
        self.create_subscription(GoalStatusArray, NS + '/nav2/navigate_to_pose/_action/status', self.on_nav_status,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/x2_twin/model_truth', self.on_truth, qos)
        self.create_subscription(Odometry, '/qazbot_lio/odom_level', lambda m: setattr(self, 'odom', self.pose_of(m.pose.pose)), qos)
        self.create_subscription(LaserScan, '/qazbot_lio/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, NS + '/nav2/cmd_vel_nav', lambda m: setattr(self, 'cmd_nav', (m.linear.x, m.linear.y, m.angular.z)), qos)
        self.create_subscription(Twist, NS + '/cmd_vel_authorized', lambda m: setattr(self, 'cmd_auth', (m.linear.x, m.linear.y, m.angular.z)), qos)
        if McLocomotionVelocity is not None:
            self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity',
                                     lambda m: setattr(self, 'mc_cmd', (m.forward_velocity, m.lateral_velocity, m.angular_velocity)),
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.marker = self.create_publisher(Marker, NS + '/operator/panel', 1)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.slow)
        self.event('PANEL_START', cfg=dict(hit_min_abs=self.cfg.hit_min_abs, score_sigma=self.cfg.score_sigma))

    # ----------------------------------------------------------------------------------------- helpers
    @staticmethod
    def pose_of(p):
        return [p.position.x, p.position.y, yaw_of(p.orientation)]

    def event(self, kind, **kw):
        self.events.write(json.dumps(dict(t=time.time(), kind=kind, **kw), default=float) + '\n')

    def on_status(self, key, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        setattr(self, key, d)
        if key == 'trust':
            sig = (d.get('ready'), d.get('motion_ok'), tuple(d.get('reasons', [])))
        else:
            sig = json.dumps({k: d.get(k) for k in ('busy', 'paused', 'latched', 'fault', 'ready', 'gate', 'state', 'reason', 'driver_mode') if k in d}, sort_keys=True, default=str)
        if self.prev_key.get(key) != sig:
            self.prev_key[key] = sig
            self.event(key.upper() + '_CHANGE', status=d if key != 'trust' else dict(ready=d.get('ready'), motion_ok=d.get('motion_ok'),
                                                                                    reasons=d.get('reasons'), metrics=d.get('metrics')))

    def on_amcl(self, m):
        self.amcl = self.pose_of(m.pose.pose) + [m.pose.covariance[0], m.pose.covariance[7], m.pose.covariance[35]]

    def on_truth(self, m):
        self.truth = self.pose_of(m.pose.pose)
        self.truth_hist.append((time.time(), self.truth[0], self.truth[1]))
        self.truth_hist = [h for h in self.truth_hist if h[0] > time.time() - 2.0]

    def on_goal(self, m):
        self.last_goal = [m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)]
        self.event('GOAL', goal=self.last_goal, truth=self.truth, amcl=self.amcl, trust_ready=self.trust.get('ready'))

    NAV_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

    def on_nav_status(self, m):
        for st in m.status_list:
            uid = bytes(st.goal_info.goal_id.uuid).hex()
            if self.nav_status.get(uid) != st.status:
                self.nav_status[uid] = st.status
                self.nav_code = st.status
                self.event('NAV_STATUS', goal_uuid=uid, status=self.NAV_NAMES.get(st.status, st.status), truth=self.truth, amcl=self.amcl,
                           goal=self.last_goal)

    def on_scan(self, m):
        pts = L.scan_endpoints(m.ranges, m.angle_min, m.angle_increment, self.cfg.rmin, self.cfg.rmax)
        self.scans = (self.scans + [(time.time(), pts)])[-8:]

    def speed(self):
        h = self.truth_hist
        if len(h) < 2 or h[-1][0] - h[0][0] < 0.3:
            return None
        return math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2]) / (h[-1][0] - h[0][0])

    # ----------------------------------------------------------------------------------------- seed explanation
    def on_seed(self, m):
        pose = self.pose_of(m.pose.pose)
        self.seed = dict(t=time.time(), pose=pose, diag=None, frame=m.header.frame_id)
        self.event('SEED', pose=pose, frame=m.header.frame_id, truth=self.truth, covariance_diag=[m.pose.covariance[0], m.pose.covariance[7],
                                                                                                      m.pose.covariance[35]])

    def explain_seed(self):
        s = self.seed
        if not s or s['diag'] is not None or time.time() - s['t'] < 0.8:
            return
        scans = [p for t, p in self.scans if t > s['t'] + 0.1 and len(p)]
        if len(scans) < 3 and time.time() - s['t'] < 4.0:
            return
        if not scans:
            s['diag'] = dict(verdict='NO_SCAN', text='no scan received after the seed: cannot judge the seed')
            self.event('SEED_DIAG', **s['diag'])
            return
        sc = float(np.mean([L.soft_score(p, self.grid, tuple(s['pose']), self.cfg.score_sigma) for p in scans]))
        amb = L.ambiguity(scans[-1], self.grid, tuple(s['pose']), self.cfg)
        fits = sc >= self.cfg.hit_min_abs
        dx, dy, dth = amb['best_offset']
        s['diag'] = dict(verdict='FITS' if fits else 'DOES_NOT_FIT', seed_score=round(sc, 3), trust_hit_min_abs=self.cfg.hit_min_abs,
                         best_nearby_score=round(float(amb['best']), 3), best_offset=[round(dx, 2), round(dy, 2), round(math.degrees(dth), 1)],
                         scans_used=len(scans),
                         text=(f'seed fits the scan (score {sc:.2f} >= {self.cfg.hit_min_abs})' if fits else
                               f'SEED DOES NOT FIT THE SCAN: score {sc:.2f} < {self.cfg.hit_min_abs}; best nearby pose is '
                               f'{dx:+.2f} m, {dy:+.2f} m, {math.degrees(dth):+.0f} deg away (score {amb["best"]:.2f}). '
                               'Trust will not go READY until AMCL converges; consider re-placing the pose.'))
        self.event('SEED_DIAG', **s['diag'])

    # ----------------------------------------------------------------------------------------- periodic
    def text(self):
        t = self.trust
        lines = []
        if not t:
            lines.append('TRUST: no status yet (stack starting)')
        else:
            state = 'READY' if t.get('ready') else 'NOT READY'
            mot = 'MOTION_OK' if t.get('motion_ok') else 'motion blocked'
            lines.append(f'TRUST: {state} / {mot}')
            if t.get('reasons'):
                lines.append('reasons: ' + ', '.join(t['reasons'][:4]))
            m = t.get('metrics') or {}
            bits = [f'{k}={m[k]:.2f}' for k in ('sigma_xy', 'second_mode_weight', 'scan_score') if isinstance(m.get(k), (int, float))]
            if bits:
                lines.append('  '.join(bits))
        if self.seed is None:
            lines.append('no initialpose yet: RViz > 2D Pose Estimate')
        elif self.seed['diag']:
            lines.append('seed: ' + self.seed['diag']['text'][:140])
        else:
            lines.append('seed received, judging fit ...')
        c = self.coord or {}
        if c:
            flag = [k for k in ('busy', 'paused', 'latched') if c.get(k)]
            lines.append('coordinator: ' + (','.join(flag) or 'idle') + (' FAULT:' + str(c['fault'])[:50] if c.get('fault') else '') +
                         '   nav2: ' + self.NAV_NAMES.get(self.nav_code, '-'))
        return '\n'.join(lines)

    def slow(self):
        self.explain_seed()
        ref = self.amcl or (self.seed or {}).get('pose') or self.truth or [0.0, 0.0, 0.0]
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.ns, mk.id, mk.type, mk.action = 'operator_panel', 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = ref[0], ref[1] + 1.4, 1.8
        mk.pose.orientation.w = 1.0
        mk.scale.z = 0.35
        ready = bool(self.trust.get('ready'))
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (0.2, 1.0, 0.2, 1.0) if ready else (1.0, 0.75, 0.1, 1.0)
        if self.seed and self.seed['diag'] and self.seed['diag']['verdict'] == 'DOES_NOT_FIT':
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.25, 0.25
        mk.text = self.text()
        self.marker.publish(mk)
        live = dict(t=time.time(), uptime_s=time.time() - self.t0, trust=dict(ready=self.trust.get('ready'), motion_ok=self.trust.get('motion_ok'),
                                                                                  reasons=self.trust.get('reasons'), metrics=self.trust.get('metrics')),
                    seed_diag=(self.seed or {}).get('diag'), seed=(self.seed or {}).get('pose'), goal=self.coord, adapter=self.adapter,
                    truth=self.truth, amcl=self.amcl, speed_mps=self.speed(), panel_text=mk.text)
        tmp = self.out / 'live_status.json.tmp'
        tmp.write_text(json.dumps(live, default=float))
        os.replace(tmp, self.out / 'live_status.json')

    def tick(self):
        row = dict(t=time.time(), truth=self.truth, amcl=self.amcl, odom=self.odom, v=self.speed(), cmd_nav=self.cmd_nav, cmd_auth=self.cmd_auth,
                   mc_cmd=self.mc_cmd, ready=self.trust.get('ready'), motion_ok=self.trust.get('motion_ok'),
                   busy=(self.coord or {}).get('busy'), paused=(self.coord or {}).get('paused'), latched=(self.coord or {}).get('latched'),
                   nav=self.nav_code, goal=self.last_goal)
        self.trace.write(json.dumps(row, default=float) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-yaml', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Panel(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.event('PANEL_STOP')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
