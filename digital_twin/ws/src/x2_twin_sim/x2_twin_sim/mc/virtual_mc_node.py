"""Virtual MC: the AimDK motion-controller INTERFACE for fast regressions (no MuJoCo). Not the official MC; see input_sources.

Serves /aimdk_5Fmsgs/srv/SetMcInputSource, listens on /aima/mc/locomotion/velocity, publishes /aima/mc/common/state (10 Hz, the
real MC rate) and forwards accepted forward/angular velocity to the plant as Twist.
"""
import json

import rclpy
from aimdk_msgs.msg import McCommonState, McLocomotionVelocity
from aimdk_msgs.srv import SetMcInputSource
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from x2_twin_sim.guards import require_sim_environment
from x2_twin_sim.mc.input_sources import InputSourceRegistry

BODY_STANDING, FSM_STABLE = 1, 4          # wire values observed on the robot (STAND_DEFAULT)


class VirtualMcNode(Node):
    def __init__(self):
        require_sim_environment()
        super().__init__('virtual_mc')
        p = self.declare_parameter
        plant_cmd = p('plant_cmd_topic', '/x2_twin/plant_cmd').value
        self.bounds = {k: tuple(float(x) for x in p(k + '_bounds', d).value)
                       for k, d in (('forward', [-1.0, 0.8]), ('lateral', [-1.0, 1.0]), ('angular', [-1.0, 1.0]))}
        self.registry = InputSourceRegistry()
        self.current = ''
        self.last = (0.0, 0.0)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_service(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource', self._on_register)
        self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self._on_velocity, best_effort)
        self.state_pub = self.create_publisher(McCommonState, '/aima/mc/common/state', best_effort)
        self.plant_pub = self.create_publisher(Twist, plant_cmd, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(0.5, self._publish_status)

    def _on_register(self, req, resp):
        resp.response.header.code = self.registry.apply(
            req.action.value, req.input_source.name, req.input_source.priority, req.input_source.timeout)
        return resp

    def _on_velocity(self, m: McLocomotionVelocity) -> None:
        if not self.registry.accepts(m.source):
            return
        if m.lateral_velocity != 0.0:
            return                                          # the plants are non-holonomic; the adapter never sends lateral
        self.current, self.last = m.source, (m.forward_velocity, m.angular_velocity)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.last
        self.plant_pub.publish(cmd)

    def _publish_state(self) -> None:
        m = McCommonState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.input_source.name = self.current
        m.body_status.value, m.fsm_state.current_state = BODY_STANDING, FSM_STABLE
        sp = m.speed_status
        sp.speed_mode.value = 1
        sp.forward_bounds.min_value, sp.forward_bounds.max_value = self.bounds['forward']
        sp.lateral_bounds.min_value, sp.lateral_bounds.max_value = self.bounds['lateral']
        sp.angular_bounds.min_value, sp.angular_bounds.max_value = self.bounds['angular']
        self.state_pub.publish(m)

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(
            {'source': self.current, 'last_cmd': self.last, 'counters': self.registry.counters})))


def main() -> None:
    rclpy.init()
    node = VirtualMcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
