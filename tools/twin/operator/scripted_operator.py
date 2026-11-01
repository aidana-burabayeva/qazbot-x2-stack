#!/usr/bin/env python3
"""Scripted stand-in for the human operator in regressions: publishes EXACTLY what the RViz tools publish
(2D Pose Estimate -> /qazbot_d1_map/initialpose with the RViz default covariance; 2D Goal Pose -> /qazbot_d1_map/goal_pose)
and observes. It is NOT part of the product path and never replaces the human in the documented flow.
Runs inside a container of the session (stack image), e.g.:  docker compose exec -T panel python3 - < scripted_operator.py <args>
"""
import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

NS = '/qazbot_d1_map'
NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class Op(Node):
    def __init__(self):
        super().__init__('scripted_operator')
        q = QoSProfile(depth=10)
        self.seed_pub = self.create_publisher(PoseWithCovarianceStamped, NS + '/initialpose', q)
        self.goal_pub = self.create_publisher(PoseStamped, NS + '/goal_pose', q)
        self.trust, self.coord, self.adapter, self.truth, self.nav = {}, {}, {}, None, {}
        self.create_subscription(String, NS + '/localization_trust/status', lambda m: setattr(self, 'trust', json.loads(m.data)), q)
        self.create_subscription(String, NS + '/goal_coordinator/status', lambda m: setattr(self, 'coord', json.loads(m.data)), q)
        self.create_subscription(String, NS + '/motion_adapter/status', lambda m: setattr(self, 'adapter', json.loads(m.data)), q)
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/x2_twin/model_truth', self._truth, q)
        self.create_subscription(GoalStatusArray, NS + '/nav2/navigate_to_pose/_action/status', self._nav,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _truth(self, m):
        p = m.pose.pose
        self.truth = [p.position.x, p.position.y, math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)]

    def _nav(self, m):
        for s in m.status_list:
            self.nav[bytes(s.goal_info.goal_id.uuid).hex()] = (NAMES.get(s.status, s.status), time.time())

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout, step=0.2):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=step)
            if cond():
                return True
        return False

    def seed(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25               # RViz 2D Pose Estimate defaults
        m.pose.covariance[35] = 0.06853891945200942
        self.wait(lambda: self.seed_pub.get_subscription_count() >= 3, 10)   # AMCL, Trust inputs, panel/recorder (discovery race)
        self.spin(1.0)
        self.seed_pub.publish(m)

    def goal(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.wait(lambda: self.goal_pub.get_subscription_count() >= 3, 10)   # coordinator, panel, recorder (discovery race)
        self.spin(1.0)
        self.goal_pub.publish(m)

    def trigger(self, name, timeout=15):
        c = self.create_client(Trigger, name)
        if not c.wait_for_service(timeout_sec=timeout):
            return dict(success=False, message='service unavailable: ' + name)
        f = c.call_async(Trigger.Request())
        self.wait(f.done, timeout)
        r = f.result()
        return dict(success=bool(r and r.success), message=(r.message if r else 'no response'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('steps', nargs='+', help='register | seed X Y YAW | ready T | goal X Y YAW | finish T | cancel | resume | sleep S | state')
    a = ap.parse_args()
    rclpy.init()
    op = Op()
    op.spin(1.5)                                        # statuses are transient-local: remember what existed before this call
    initial = {k: v[0] for k, v in op.nav.items()}
    log = []
    try:
        i = 0
        s = a.steps
        while i < len(s):
            t0 = time.time()
            cmd = s[i]
            if cmd == 'register':
                r = op.trigger(NS + '/motion_adapter/register')
                ok = op.wait(lambda: op.adapter.get('registration') in ('registered', 'failed'), 30)
                log.append(dict(step='register', service=r, registration=op.adapter.get('registration'), error=op.adapter.get('registration_error'), waited=ok))
                i += 1
            elif cmd == 'seed':
                op.spin(1.0)
                x, y, yaw = (float(v) for v in s[i + 1:i + 4])
                op.seed(x, y, yaw)
                log.append(dict(step='seed', pose=[x, y, yaw], truth=op.truth))
                i += 4
            elif cmd == 'ready':
                T = float(s[i + 1])
                ok = op.wait(lambda: bool(op.trust.get('ready')) and bool(op.trust.get('motion_ok')), T)
                trust_after = round(time.time() - t0, 1)
                coord_ok = op.wait(lambda: bool(op.coord.get('ready')), 60) if ok else False     # the Goal Coordinator needs a moment after READY
                log.append(dict(step='ready', ok=ok, after_s=trust_after, reasons=op.trust.get('reasons'), metrics=op.trust.get('metrics'),
                                coordinator_ready=coord_ok, coordinator_ready_after_s=round(time.time() - t0, 1)))
                i += 2
            elif cmd == 'goal':
                x, y, yaw = (float(v) for v in s[i + 1:i + 4])
                before = set(op.nav)
                op.goal(x, y, yaw)
                op.wait(lambda: set(op.nav) - before, 15)
                log.append(dict(step='goal', goal=[x, y, yaw], truth=op.truth, accepted=bool(set(op.nav) - before), coord=op.coord))
                i += 4
            elif cmd == 'finish':
                T = float(s[i + 1])
                done = ('SUCCEEDED', 'CANCELED', 'ABORTED')
                # only goals that are new in this call, or were still running when it started, count
                relevant = lambda: [v for k, v in op.nav.items() if k not in initial or initial[k] not in done]
                term = lambda: any(v[0] in done for v in relevant())
                ok = op.wait(term, T, 0.5)
                last = max(relevant(), key=lambda v: v[1])[0] if relevant() else None
                log.append(dict(step='finish', terminal=ok, status=last, truth=op.truth, after_s=round(time.time() - t0, 1)))
                i += 2
            elif cmd == 'cancel':
                log.append(dict(step='cancel', result=op.trigger(NS + '/goal_coordinator/stop')))
                i += 1
            elif cmd == 'resume':
                log.append(dict(step='resume', result=op.trigger(NS + '/goal_coordinator/resume')))
                i += 1
            elif cmd == 'sleep':
                op.spin(float(s[i + 1]))
                i += 2
            elif cmd == 'state':
                op.spin(1.0)
                log.append(dict(step='state', trust=dict(ready=op.trust.get('ready'), motion_ok=op.trust.get('motion_ok'), reasons=op.trust.get('reasons'),
                                                        metrics=op.trust.get('metrics')), coord=op.coord, adapter=op.adapter, truth=op.truth))
                i += 1
            else:
                raise SystemExit('unknown step ' + cmd)
    finally:
        print(json.dumps(log, indent=1, default=str))
        op.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
