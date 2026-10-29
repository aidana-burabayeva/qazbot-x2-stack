"""Truth-only localisation for the twin (roadmap P2 / A2): AMCL and Localization Trust are replaced by ground truth.

The plant already publishes odom -> base_footprint in the MAP frame (FrameAligner puts the MuJoCo pelvis pose into the map frame),
so the exact map -> odom is the identity. This node publishes that single TF edge (the one AMCL normally owns), the trust verdict the
Goal Coordinator consumes (``localization_trust/trusted``) and a status with the shape of the real trust node. It exists to isolate the
controller and the MC from localisation errors; it is never used for acceptance of the real stack.
"""
import json

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from x2_twin_sim.guards import require_sim_environment
from x2_twin_sim.plant.base import spin

NS = 'qazbot_d1_map'


def identity_map_odom(stamp) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'map', 'odom'
    tf.transform.rotation.w = 1.0
    return tf


def trust_status() -> dict:
    return dict(trusted=True, ready=True, reasons=[], metrics={}, source='truth_localizer')


class TruthLocalizer(Node):
    def __init__(self):
        require_sim_environment()
        super().__init__('truth_localizer', namespace=NS)
        self.tf_pub = self.create_publisher(TFMessage, f'/{NS}/tf', QoSProfile(depth=100))
        self.trusted = self.create_publisher(Bool, 'localization_trust/trusted', 10)
        self.status = self.create_publisher(String, 'localization_trust/status', 10)
        # The coordinator calls arm_check for every new goal and is `ready` only while the service exists (real node: ambiguity scan).
        for name in ('arm_check', 'confirm_anchor'):
            self.create_service(Trigger, f'localization_trust/{name}', self._ok)
        self.create_timer(0.05, lambda: self.tf_pub.publish(TFMessage(transforms=[identity_map_odom(self.get_clock().now().to_msg())])))
        self.create_timer(0.1, lambda: self.trusted.publish(Bool(data=True)))
        self.create_timer(0.5, lambda: self.status.publish(String(data=json.dumps(trust_status()))))

    @staticmethod
    def _ok(_request, response):
        response.success, response.message = True, 'truth localisation: always armed'
        return response


def main() -> None:
    spin(TruthLocalizer)
