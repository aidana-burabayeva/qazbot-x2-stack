"""Plant fed by the OFFICIAL MuJoCo body: pose from ``/aima/hal/odom/state`` (simulated pelvis, ground truth).

Limits (stated, not hidden): the pose is the pelvis, not the chest-LiDAR IMU point FAST-LIO tracks (~0.1 m planar offset on the
robot); there is no odometry drift; the floor in MuJoCo is flat without the map's walls (only the scan 'sees' them).
"""
import math
from typing import Optional

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from x2_twin_sim.geometry import FrameAligner, planar_pose, wrap_angle
from x2_twin_sim.plant.base import PlantNode, State, spin


class HalPlantNode(PlantNode):
    """By default map = start (+) world pose; the spawn pose is the world origin.

    The ``anchor`` parameter can select legacy ``'first_sample'`` anchoring.
    """

    def __init__(self):
        super().__init__('hal_plant')
        self.hal_topic = self.declare_parameter('hal_odom_topic', '/aima/hal/odom/state').value
        anchor = self.declare_parameter('anchor', 'nominal').value
        self.aligner = FrameAligner(self.start_pose, anchor=anchor)
        self.pose = None
        self.track = []                                   # (t, x, y, yaw) for the finite-difference twist
        self.v = self.w = 0.0
        self.create_subscription(Odometry, self.hal_topic, self._on_hal, qos_profile_sensor_data)

    def _on_hal(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        pose = self.aligner.align(planar_pose(p.x, p.y, q.x, q.y, q.z, q.w))
        t = self.get_clock().now().nanoseconds * 1e-9
        self.track.append((t, *pose))
        while len(self.track) > 2 and t - self.track[0][0] > 0.2:
            self.track.pop(0)
        t0, x0, y0, w0 = self.track[0]
        if t - t0 > 0.05:
            vx, vy = (pose[0] - x0) / (t - t0), (pose[1] - y0) / (t - t0)
            self.v = vx * math.cos(pose[2]) + vy * math.sin(pose[2])
            self.w = wrap_angle(pose[2] - w0) / (t - t0)
        self.pose = pose

    def current_state(self) -> Optional[State]:
        return None if self.pose is None else (self.pose, self.v, self.w)


def main() -> None:
    spin(HalPlantNode)
