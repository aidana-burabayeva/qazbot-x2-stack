#!/usr/bin/env python3
"""Passive walk check: LIO odometry vs twist vs AMCL while an operator walks X2.

Waits for motion, records until the robot is still again, then compares
  * LIO displacement vs the integral of the bridge twist (rotated to odom),
  * AMCL displacement in map vs LIO displacement in odom,
  * map->odom jumps, stamp gaps/rollbacks, finite scan rays.
Subscribes only. Usage: walk_check.py OUT.json [max_seconds]
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

NS = '/qazbot_d1_map'
START_M, STILL_V, STILL_S = 0.05, 0.03, 4.0


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


class Rec(Node):
    def __init__(self):
        super().__init__('qazbot_walk_check')
        self.odom, self.amcl, self.mo, self.scan = [], [], [], []
        self.create_subscription(Odometry, '/qazbot_lio/odom_level', self.on_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped, NS + '/amcl_pose', self.on_amcl,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, NS + '/tf', self.on_tf, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, '/qazbot_lio/scan', self.on_scan,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        self.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y,
                          yaw_of(p.orientation), t.linear.x, t.linear.y, t.angular.z, time.time()))

    def on_amcl(self, m):
        p = m.pose.pose
        self.amcl.append((time.time(), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_tf(self, m):
        for t in m.transforms:
            if (t.header.frame_id, t.child_frame_id) == ('map', 'odom'):
                tr = t.transform.translation
                self.mo.append((time.time(), tr.x, tr.y, yaw_of(t.transform.rotation)))

    def on_scan(self, m):
        self.scan.append((time.time(), int(np.isfinite(np.array(m.ranges)).sum())))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    out, limit = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
    rclpy.init()
    n = Rec()
    t0 = time.time()
    while not n.odom and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    base = np.array(n.odom[-1][1:3])
    start = still_since = None
    print('armed: waiting for motion...', flush=True)
    while time.time() - t0 < limit:
        rclpy.spin_once(n, timeout_sec=0.05)
        o = n.odom[-1]
        if start is None:
            if math.hypot(o[1] - base[0], o[2] - base[1]) > START_M:
                start = time.time()
                print('motion detected', flush=True)
        else:
            moving = math.hypot(o[4], o[5]) > STILL_V or abs(o[6]) > 0.1
            still_since = None if moving else (still_since or time.time())
            if still_since and time.time() - still_since > STILL_S:
                break
    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    res = {'motion_detected': start is not None}
    if start:
        O = np.array([r for r in n.odom if r[7] >= start - 1.0])
        t = O[:, 0]
        dt = np.diff(t)
        d_lio = O[-1, 1:3] - O[0, 1:3]
        # integrate the base-frame twist into odom with the LIO heading
        v = np.array([(O[i, 4] * math.cos(O[i, 3]) - O[i, 5] * math.sin(O[i, 3]),
                       O[i, 4] * math.sin(O[i, 3]) + O[i, 5] * math.cos(O[i, 3])) for i in range(len(O))])
        d_tw = (v[:-1] * dt[:, None]).sum(axis=0)
        dyaw_lio = wrap(O[-1, 3] - O[0, 3])
        dyaw_tw = float((O[:-1, 6] * dt).sum())
        res['lio'] = {'samples': len(O), 'rate_hz': round(len(dt) / (t[-1] - t[0]), 2), 'dt_max_s': float(dt.max()),
                      'stamp_rollbacks': int((dt <= 0).sum()), 'max_step_m': float(np.hypot(*np.diff(O[:, 1:3], axis=0).T).max()),
                      'displacement_m': float(np.hypot(*d_lio)), 'twist_integral_m': float(np.hypot(*d_tw)),
                      'twist_vs_lio_ratio': float(np.hypot(*d_tw) / max(np.hypot(*d_lio), 1e-6)),
                      'twist_vs_lio_vector_err_m': float(np.hypot(*(d_tw - d_lio))),
                      'yaw_lio_deg': math.degrees(dyaw_lio), 'yaw_twist_deg': math.degrees(dyaw_tw),
                      'peak_speed_mps': float(np.hypot(O[:, 4], O[:, 5]).max())}
        A = np.array([r for r in n.amcl if r[0] >= start - 1.0])
        if len(A) >= 2:
            da = A[-1, 1:3] - A[0, 1:3]
            res['amcl'] = {'updates': len(A), 'displacement_m': float(np.hypot(*da)),
                           'yaw_change_deg': math.degrees(wrap(A[-1, 3] - A[0, 3])),
                           'vs_lio_displacement_diff_m': float(abs(np.hypot(*da) - np.hypot(*d_lio)))}
        else:
            res['amcl'] = {'updates': len(A), 'note': 'AMCL did not update during the walk'}
        M = np.array([r for r in n.mo if r[0] >= start - 1.0])
        if len(M) >= 2:
            j = np.hypot(*np.diff(M[:, 1:3], axis=0).T)
            res['map_odom'] = {'samples': len(M), 'max_jump_m': float(j.max()), 'total_change_m': float(np.hypot(*(M[-1, 1:3] - M[0, 1:3]))),
                               'yaw_change_deg': math.degrees(wrap(M[-1, 3] - M[0, 3])), 'jumps_over_0.1m': int((j > 0.1).sum())}
        S = [s[1] for s in n.scan if s[0] >= start - 1.0]
        res['scan'] = {'count': len(S), 'finite_rays_min': min(S) if S else None, 'finite_rays_median': float(np.median(S)) if S else None}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))
    n.destroy_node()
    rclpy.try_shutdown()


main()
