#!/usr/bin/env python3
"""Virtual MC: the AimDK X2 motion-controller INTERFACE for the digital twin (SIM only).

Sits where the real MC sits, below x2_motion_adapter, so the adapter runs unchanged with live:=true:
  * serves  /aimdk_5Fmsgs/srv/SetMcInputSource  (ADD/ENABLE of an SDK source: name sdk_*, priority 20..39),
  * listens /aima/mc/locomotion/velocity        (McLocomotionVelocity, BEST_EFFORT) and accepts a command only
    from the registered source; everything else is counted and dropped,
  * publishes /aima/mc/common/state at 10 Hz    (McCommonState, the real MC rate) with the registered source,
  * forwards accepted forward/angular velocity as Twist to the plant (--plant-cmd-topic, the kinematic model),
    which owns dynamics: 0.2 s command timeout, start thresholds, lag. Calibration belongs there, not here.
It never publishes a velocity topic itself. Refuses to start outside the isolated SIM environment.
Usage: virtual_mc.py PLANT_CMD_TOPIC
"""
import json
import os
import signal
import sys

import rclpy
from aimdk_msgs.msg import McCommonState, McInputAction, McLocomotionVelocity
from aimdk_msgs.srv import SetMcInputSource
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

SDK_PRIORITY = (20, 39)       # AimDK: SDK input sources
FWD, LAT, ANG = (-1.0, 0.8), (-1.0, 1.0), (-1.0, 1.0)   # same live bounds the existing fixture reports
BODY_STANDING, FSM_LOCOMOTION = 1, 4                    # values used by tests/nav_e2e fixtures


class VirtualMC(Node):
    def __init__(self, plant_topic):
        super().__init__('virtual_mc')
        self.source = None          # registered SDK source name
        self.priority = None
        self.counters = dict(accepted=0, rejected_unregistered=0, rejected_lateral=0, registrations=0,
                             registrations_refused=0)
        self.last = (0.0, 0.0)
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_service(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource', self._register)
        self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self._on_velocity, be)
        self.state_pub = self.create_publisher(McCommonState, '/aima/mc/common/state', be)
        self.plant = self.create_publisher(Twist, plant_topic, 10)
        self.status = self.create_publisher(String, '~/status', 10)
        self.create_timer(0.1, self._state_tick)
        self.create_timer(0.5, self._status_tick)

    def _register(self, req, resp):
        name, prio = req.input_source.name, req.input_source.priority
        ok = (req.action.value in (McInputAction.INPUTACTION_ADD, McInputAction.INPUTACTION_ENABLE)
              and name.startswith('sdk_') and SDK_PRIORITY[0] <= prio <= SDK_PRIORITY[1])
        if ok and self.source is None:
            self.source, self.priority = name, prio
            self.counters['registrations'] += 1
            resp.response.header.code = 0
        else:
            self.counters['registrations_refused'] += 1
            resp.response.header.code = 1
        return resp

    def _on_velocity(self, m):
        if self.source is None or m.source != self.source:
            self.counters['rejected_unregistered'] += 1
            return
        if m.lateral_velocity != 0.0:
            self.counters['rejected_lateral'] += 1           # plant is non-holonomic; the adapter never sends lateral
            return
        self.counters['accepted'] += 1
        self.last = (m.forward_velocity, m.angular_velocity)
        t = Twist()
        t.linear.x, t.angular.z = self.last
        self.plant.publish(t)

    def _state_tick(self):
        m = McCommonState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.input_source.name = self.source or ''
        m.input_source.priority = self.priority or 0
        m.body_status.value, m.fsm_state.current_state = BODY_STANDING, FSM_LOCOMOTION
        sp = m.speed_status
        sp.speed_mode.value = 1
        sp.forward_bounds.min_value, sp.forward_bounds.max_value = FWD
        sp.lateral_bounds.min_value, sp.lateral_bounds.max_value = LAT
        sp.angular_bounds.min_value, sp.angular_bounds.max_value = ANG
        self.state_pub.publish(m)

    def _status_tick(self):
        self.status.publish(String(data=json.dumps(dict(source=self.source, priority=self.priority,
                                                        last_cmd=self.last, counters=self.counters))))


def main():
    if (os.environ.get('QAZBOT_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID') != '87'
            or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('virtual MC runs only in the isolated SIM (source twin/env_sim.sh)')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = [False]
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__(0, True))
    node = VirtualMC(sys.argv[1])
    try:
        while not stop[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
