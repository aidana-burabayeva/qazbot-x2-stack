"""Light plant (no MuJoCo): first-order unicycle with MC-like command handling. IDEALISED, NOT CALIBRATED.

Constants come from the AimDK locomotion documentation (command zeroed after 0.2 s; start thresholds ~0.1 m/s and
~0.03-0.05 rad/s) and a first-order lag; the documentation itself says thresholds are learned by the MC and must be measured
on the robot. Use for fast regression of the stack's logic, never as evidence about the real MC (see docs/model_fidelity.md).
"""
import math
from typing import Optional

from geometry_msgs.msg import Twist

from x2_twin_sim.geometry import wrap_angle
from x2_twin_sim.plant.base import PlantNode, State, spin

CMD_TIMEOUT_S = 0.2
START_VX, START_WZ = 0.1, 0.05
TAU_S = 0.25


class KinematicPlantNode(PlantNode):
    def __init__(self):
        super().__init__('kinematic_plant')
        self.x, self.y, self.yaw = self.start_pose
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.moving = False
        self.last = self.get_clock().now().nanoseconds * 1e-9
        self.create_subscription(Twist, self.declare_parameter('cmd_topic', '/x2_twin/plant_cmd').value, self._on_cmd, 10)
        self.create_timer(0.02, self._step)

    def _on_cmd(self, m: Twist) -> None:
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), self.get_clock().now().nanoseconds * 1e-9

    def _step(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt, self.last = now - self.last, now
        cv, cw = self.cmd if now - self.cmd_t <= CMD_TIMEOUT_S else (0.0, 0.0)
        if not self.moving and abs(cv) < START_VX and abs(cw) < START_WZ:
            cv = cw = 0.0
        self.moving = abs(cv) > 1e-6 or abs(cw) > 1e-6 or abs(self.v) > 0.01 or abs(self.w) > 0.01
        a = 1.0 - math.exp(-dt / TAU_S)
        self.v += (cv - self.v) * a
        self.w += (cw - self.w) * a
        self.yaw = wrap_angle(self.yaw + self.w * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def current_state(self) -> Optional[State]:
        return (self.x, self.y, self.yaw), self.v, self.w


def main() -> None:
    spin(KinematicPlantNode)
