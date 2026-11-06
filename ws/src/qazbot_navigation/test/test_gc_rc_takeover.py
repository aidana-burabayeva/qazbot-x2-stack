"""Goal Coordinator side of the manual-takeover fix (T6 2026-11-06). The adapter latches `operator_takeover` on
manual-intervention evidence also while IDLE inside a busy goal (T1 pause, Nav2 command gap); the coordinator must
cancel the Nav2 goal and latch, the old goal must not resume after RC release, and the driver is disarmed when the
goal is terminal (or rejected after arming) so operator RC use between goals does not latch the adapter.
No ROS graph (object.__new__ + fakes, see test_gc_t1_pause); coroutines are driven by hand."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gc_t1_pause import LIVE_OK, LIVE_PAUSED, make, tick  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action.server import GoalResponse  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

LIVE_IDLE_GAP = dict(LIVE_OK, state='idle', reason='no_command')
TAKEOVER = dict(live=True, registration='registered', ready=False, state='latched', reason='operator_takeover',
                latch='operator_takeover', motion_armed=False)


def done(value):
    f = Future()
    f.set_result(value)
    return f


class Client:
    """Fake rclpy service client: ready, answers immediately, counts calls."""

    def __init__(self, response=None, ready=True):
        self.response, self.ready, self.calls = response, ready, 0
    def service_is_ready(self): return self.ready
    def server_is_ready(self): return self.ready
    def call_async(self, req):
        self.calls += 1
        return done(self.response)


class Resp:
    pass


class Handle:
    def __init__(self):
        self.is_cancel_requested, self.is_active, self.ended = False, True, None
        self.request = NavigateToPose.Goal()
    def canceled(self): self.ended = 'canceled'
    def abort(self): self.ended = 'aborted'
    def succeed(self): self.ended = 'succeeded'


class Child:
    def __init__(self):
        self.accepted, self.cancels, self.result = True, 0, Future()
        self.goal_id = type('U', (), {'uuid': [0] * 16})()
    def cancel_goal_async(self): self.cancels += 1
    def get_result_async(self): return self.result


class NavClient:
    def __init__(self, child): self.child = child
    def send_goal_async(self, req, feedback_callback=None): return done(self.child)
    def server_is_ready(self): return True


def finish(coro):
    try:
        coro.send(None)
    except StopIteration as stop:
        return stop.value
    raise AssertionError('coroutine did not finish')


def executing(n):
    """Starts execute() for a fresh Nav2 child; returns (coroutine suspended at the Nav2 result, handle, child)."""
    child, h = Child(), Handle()
    n.client, n.child = NavClient(child), None
    coro = n.execute(h)
    coro.send(None)                                              # suspended in get_result_async
    return coro, h, child


def forward(n, vx=0.3):
    n.get_publishers_info_by_topic = lambda t: [type('E', (), {'endpoint_gid': b'1'})()]
    n.resolve_topic_name = lambda t: t
    before = len(n.pub.msgs)
    m = Twist()
    m.linear.x = vx
    n.command(m)
    return n.pub.msgs[before:]


class GcFixture(unittest.TestCase):
    def setUp(self):
        self.n, self.c = make()
        self.n.driver_disarm = Client(Trigger.Response(success=True))


class NormalNavigation(GcFixture):
    def test_goal_commands_flow_and_disarm_when_terminal(self):
        n = self.n
        coro, h, child = executing(n)
        self.assertTrue(n.gate)
        self.assertEqual(len(forward(n)), 1)                     # authorized command forwarded
        self.assertEqual(n.driver_disarm.calls, 0)               # goal still owns the driver
        child.result.set_result(type('R', (), {'status': GoalStatus.STATUS_SUCCEEDED,
                                               'result': NavigateToPose.Result()})())
        finish(coro)
        self.assertEqual(h.ended, 'succeeded')
        self.assertFalse(n.busy)
        self.assertEqual(n.driver_disarm.calls, 1)

    def test_disarm_unavailable_is_logged_and_adapter_stays_armed(self):
        n = self.n
        n.driver_disarm = Client(ready=False)
        n.disarm_driver()
        self.assertEqual(n.driver_disarm.calls, 0)


class TakeoverDuringGoal(GcFixture):
    def assert_tripped_and_old_goal_dead(self, coro, h, child):
        n = self.n
        self.assertTrue(n.latched)
        self.assertFalse(n.gate)
        self.assertEqual(n.fault, 'operator_takeover')
        self.assertEqual(child.cancels, 1)
        self.assertEqual(n.pub.msgs[-1], Twist())
        # RC released; adapter after the operator's ~/resume reports idle/driver_not_armed; trust is fine
        tick(n, self.c, 1.0, True, False, adapter=dict(LIVE_OK, state='idle', reason='driver_not_armed',
                                                       motion_armed=False))
        self.assertFalse(n.gate)
        self.assertEqual(forward(n), [])                         # Nav2 old-goal command not forwarded
        self.assertFalse(n.resume(None, Resp()).success)         # old action not terminal yet
        child.result.set_result(type('R', (), {'status': GoalStatus.STATUS_CANCELED,
                                               'result': NavigateToPose.Result()})())
        finish(coro)
        self.assertEqual(h.ended, 'aborted')                     # latched -> aborted, no replay
        self.assertEqual(n.driver_disarm.calls, 1)
        self.assertEqual(forward(n, 0.0), [])                    # Nav2 final zero after the cancel (not forwarded)
        r = n.resume(None, Resp())
        self.assertTrue(r.success)
        self.assertIn('NEW goal', r.message)
        self.assertFalse(n.gate)
        self.assertEqual(forward(n), [])                         # nothing moves without a new goal

    def test_takeover_while_driving(self):
        coro, h, child = executing(self.n)
        self.n.adapter(String(data=json.dumps(TAKEOVER)))
        self.assert_tripped_and_old_goal_dead(coro, h, child)

    def test_takeover_in_nav2_command_gap(self):
        coro, h, child = executing(self.n)
        self.n.adapter(String(data=json.dumps(LIVE_IDLE_GAP)))   # adapter IDLE inside the goal: no trip
        self.assertFalse(self.n.latched)
        self.n.adapter(String(data=json.dumps(TAKEOVER)))
        self.assert_tripped_and_old_goal_dead(coro, h, child)

    def test_takeover_during_t1_pause(self):
        n = self.n
        coro, h, child = executing(n)
        tick(n, self.c, 1.0, False, True, adapter=LIVE_PAUSED)   # T1 pause: gate closed, goal kept
        self.assertFalse(n.latched)
        self.assertFalse(n.gate)
        n.adapter(String(data=json.dumps(dict(TAKEOVER, t1_pause=True))))
        self.assertTrue(n.latched)
        tick(n, self.c, 0.5, True, False, adapter=TAKEOVER)      # trust back: T1 reopen must NOT happen
        self.assertFalse(n.gate)
        self.assert_tripped_and_old_goal_dead(coro, h, child)


class GoalCancel(GcFixture):
    def test_cancel_trips_and_disarms_when_terminal(self):
        n = self.n
        coro, h, child = executing(n)
        n.cancel(h)
        self.assertTrue(n.latched)
        self.assertEqual(child.cancels, 1)
        h.is_cancel_requested = True
        child.result.set_result(type('R', (), {'status': GoalStatus.STATUS_CANCELED,
                                               'result': NavigateToPose.Result()})())
        finish(coro)
        self.assertEqual(h.ended, 'canceled')
        self.assertEqual(n.driver_disarm.calls, 1)


class SdkUnavailable(GcFixture):
    def test_unregistered_adapter_trips_busy_goal(self):
        coro, h, child = executing(self.n)
        self.n.adapter(String(data=json.dumps(dict(live=True, registration='not_requested', ready=False,
                                                   state='idle', reason='registration_not_requested'))))
        self.assertTrue(self.n.latched)
        self.assertEqual(child.cancels, 1)


class Admission(GcFixture):
    def setUp(self):
        super().setUp()
        n = self.n
        n.busy, n.nav_active, n.client = False, True, NavClient(Child())
        state = GetState.Response()
        state.current_state.id = 3
        n.life = Client(state)
        n.arm = Client(Trigger.Response(success=True))
        n.driver_arm = Client(Trigger.Response(success=True))
        self.req = NavigateToPose.Goal()
        self.req.pose.header.frame_id = 'map'
        self.req.pose.pose.orientation.w = 1.0

    def admit(self, clean):
        coro = self.n.goal(self.req)
        coro.send(None)                                          # suspended waiting for a NEW trust observation
        self.n.trust_waiter.set_result(clean)
        return finish(coro)

    def test_rejection_after_arming_disarms(self):
        self.assertEqual(self.admit(False), GoalResponse.REJECT)
        self.assertEqual(self.n.driver_arm.calls, 1)
        self.assertEqual(self.n.driver_disarm.calls, 1)

    def test_accepted_goal_stays_armed(self):
        self.assertEqual(self.admit(True), GoalResponse.ACCEPT)
        self.assertEqual(self.n.driver_disarm.calls, 0)
        self.assertTrue(self.n.busy)


if __name__ == '__main__':
    unittest.main()
