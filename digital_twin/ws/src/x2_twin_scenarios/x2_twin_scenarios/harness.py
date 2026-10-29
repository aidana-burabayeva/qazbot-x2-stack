"""Client-side harness: observe and drive a RUNNING stack the way an operator/RViz would. No process management here."""
import json
import math
import time
from typing import Callable, Dict, List

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

NS = 'qazbot_d1_map'


class Harness(Node):
    """Namespace /qazbot_d1_map: the same relative names the stack and RViz use."""

    def __init__(self):
        super().__init__('twin_harness', namespace=NS)
        self.state: Dict[str, dict] = {'coord': {}, 'adapter': {}, 'trust': {}, 'mc': {}}
        self.pose = None                      # (x, y, yaw) ground truth from the plant
        self.vel = None
        self.events: List[tuple] = []         # journal of coordinator fault / trust reason changes
        self.results: List[dict] = []
        self.trace: List[tuple] = []          # (t, x, y, yaw, v, w) of the truth, ~10 Hz, written to trace.csv by the scenario
        self._t0 = time.monotonic()
        self.create_subscription(Odometry, '/x2_twin/model_truth', self._on_truth, 50)
        self.create_subscription(String, 'goal_coordinator/status', lambda m: self._json('coord', m), 10)
        self.create_subscription(String, 'motion_adapter/status', lambda m: self._json('adapter', m), 10)
        self.create_subscription(String, 'localization_trust/status', lambda m: self._json('trust', m), 10)
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._srv_clients: Dict[str, object] = {}

    # ---- subscriptions
    def _on_truth(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, 2.0 * math.atan2(q.z, q.w))
        self.vel = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        t = time.monotonic() - self._t0
        if not self.trace or t - self.trace[-1][0] >= 0.1:
            self.trace.append((round(t, 3), *self.pose, *self.vel))

    def _json(self, key: str, m: String) -> None:
        d = json.loads(m.data)
        self.state[key] = d
        if key == 'coord':
            self._note('coord', dict(busy=d.get('busy'), gate=d.get('gate'), fault=d.get('fault'), latched=d.get('latched')))
        elif key == 'trust':
            mt = d.get('metrics', {})
            self._note('trust', dict(trusted=d.get('trusted'), ready=d.get('ready'), reasons=d.get('reasons'),
                                     jump_latched_xy=mt.get('jump_latched_xy'), truth_pose=self.pose, truth_vel=self.vel))

    def now(self) -> float:
        return time.monotonic() - self._t0

    def _note(self, kind: str, value: dict) -> None:
        last = [e for e in self.events if e[1] == kind]
        if not last or last[-1][2] != value:
            self.events.append((round(time.monotonic() - self._t0, 2), kind, value))

    # ---- waiting / calling
    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, predicate: Callable[[], bool], seconds: float, what: str) -> None:
        """Observation deadline of the TEST (not a runtime timeout of the stack)."""
        end = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError(f'observation deadline ({seconds:.0f} s): {what}; coord={self.state["coord"]} '
                                   f'adapter={self.state["adapter"].get("reason")} trust={self.state["trust"].get("reasons")}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def client(self, srv_type, name):
        if name not in self._srv_clients:
            self._srv_clients[name] = self.create_client(srv_type, name)
        return self._srv_clients[name]

    def call(self, srv_type, name: str, request, seconds: float = 60.0):
        c = self.client(srv_type, name)
        self.wait(c.service_is_ready, seconds, f'service {name}')
        fut = c.call_async(request)
        self.wait(fut.done, seconds, f'response of {name}')
        return fut.result()

    def trigger(self, name: str) -> Trigger.Response:
        return self.call(Trigger, name, Trigger.Request())

    def empty(self, name: str) -> None:
        self.call(Empty, name, Empty.Request(), 10.0)

    # ---- navigation
    def send_goal(self, x: float, y: float, yaw: float):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.wait(self.nav.server_is_ready, 60, 'navigate_to_pose server')
        fut = self.nav.send_goal_async(goal)
        self.wait(fut.done, 60, 'goal acceptance')
        return fut.result()

    def finish(self, handle, seconds: float = 180.0) -> int:
        fut = handle.get_result_async()
        self.wait(fut.done, seconds, 'navigation result')
        return fut.result().status

    def pose_error(self, target) -> tuple:
        p = self.pose
        yaw = abs(math.atan2(math.sin(p[2] - target[2]), math.cos(p[2] - target[2])))
        return math.hypot(p[0] - target[0], p[1] - target[1]), yaw

    # ---- results
    def check(self, name: str, ok, detail=None, hard: bool = True) -> None:
        """hard=False records the outcome and continues (used for goal outcomes, which are measurements)."""
        self.results.append(dict(name=name, ok=bool(ok), detail=detail, hard=hard))
        print(json.dumps(self.results[-1], default=str), flush=True)
        if not ok and hard:
            raise AssertionError(name)


SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
