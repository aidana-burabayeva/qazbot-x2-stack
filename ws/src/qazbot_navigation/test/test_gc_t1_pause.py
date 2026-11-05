"""Goal Coordinator: T1-only trust loss while busy pauses (gate closed, zero) and continues the SAME goal; loss
> t1_pause_max_s or any non-T1 untrust trips (cancel + latch). Operator decision 2026-10-28.
No ROS graph: the node is built with object.__new__ and fake publishers/clock (rclpy/nav2_msgs must import)."""
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import goal_coordinator as G  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402


class Clock:
    t = 1000.0
    def monotonic(self): return self.t


class Pub:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


class Child:
    def __init__(self): self.cancels = 0
    def cancel_goal_async(self): self.cancels += 1


class Log:
    def warn(self, *a): pass
    info = error = warn


def make():
    clock = Clock(); G.time = clock
    n = object.__new__(G.GoalCoordinator)
    n.busy = True; n.reserving = False; n.latched = False; n.fault = ''; n.public = None
    n.gate = True; n.cancel_sent = False; n.zero_seen = True; n.ever_command = True
    n.trusted = True; n.trust_at = clock.t; n.command_gid = None
    n.driver_ready = True; n.driver_mode = 'live'; n.driver_status = {}; n.driver_status_at = clock.t
    n.allow_dry_run_driver = False; n.trust_max_age = 0.6; n.t1_pause_max = G.T1_PAUSE_MAX_S
    n.pause_since = None; n.trust_t1 = False; n.trust_status_at = clock.t
    n.alignment_watch = G.GoalAlignmentWatch(timeout=30.0); n.execution_failure = ''; n.trust_waiter = None
    n.pub = Pub(); n.child = Child(); n.get_logger = lambda: Log()
    return n, clock


def tick(n, clock, seconds, trusted, t1, status=True, adapter=None):
    """Trust node at 5 Hz (Bool then status, as published), coordinator check at 20 Hz."""
    for i in range(int(round(seconds / 0.05))):
        clock.t += 0.05
        if i % 4 == 0:
            n.trust(Bool(data=trusted))
            if status:
                n.trust_status(String(data=json.dumps(dict(trusted=trusted, ready=True, t1_transient=t1))))
            if adapter is not None:
                n.adapter(String(data=json.dumps(adapter)))
        n.check()


LIVE_OK = dict(live=True, registration='registered', ready=True, state='active', reason='driving', motion_armed=True)
LIVE_PAUSED = dict(live=True, registration='registered', ready=False, state='idle', reason='t1_pause:localization_untrusted',
                   motion_armed=True, t1_pause=True)


class T1Pause(unittest.TestCase):
    def test_default_constant(self):
        self.assertEqual(G.T1_PAUSE_MAX_S, 3.0)

    def test_blip_pauses_then_continues_same_goal(self):
        n, c = make(); child = n.child
        tick(n, c, 2.0, False, True, adapter=LIVE_PAUSED)
        self.assertFalse(n.gate); self.assertFalse(n.latched); self.assertEqual(child.cancels, 0)
        self.assertTrue(n.pub.msgs and all(m == Twist() for m in n.pub.msgs))      # zero velocity, once
        tick(n, c, 0.5, True, False, adapter=LIVE_OK)
        self.assertTrue(n.gate); self.assertIsNone(n.pause_since); self.assertIs(n.child, child)
        self.assertEqual(child.cancels, 0); self.assertFalse(n.latched); self.assertTrue(n.busy)

    def test_no_forwarding_while_paused(self):
        n, c = make()
        tick(n, c, 0.3, False, True, adapter=LIVE_PAUSED)
        before = len(n.pub.msgs)
        n.get_publishers_info_by_topic = lambda t: [type('E', (), dict(endpoint_gid=b'1'))()]
        n.resolve_topic_name = lambda t: t
        m = Twist(); m.linear.x = 0.3; n.command(m)
        self.assertEqual(len(n.pub.msgs), before)

    def test_prolonged_t1_loss_cancels_and_latches(self):
        n, c = make(); child = n.child
        tick(n, c, 2.9, False, True, adapter=LIVE_PAUSED)
        self.assertFalse(n.latched)
        tick(n, c, 0.3, False, True, adapter=LIVE_PAUSED)
        self.assertTrue(n.latched); self.assertEqual(child.cancels, 1)
        self.assertEqual(n.fault, 'localization_untrusted_t1_prolonged')
        tick(n, c, 0.5, True, False, adapter=LIVE_OK)
        self.assertFalse(n.gate); self.assertTrue(n.latched)                     # needs resume + NEW goal

    def test_repeated_blips_each_below_limit_do_not_cancel(self):
        n, c = make(); child = n.child
        for _ in range(3):
            tick(n, c, 2.0, False, True, adapter=LIVE_PAUSED)
            tick(n, c, 0.6, True, False, adapter=LIVE_OK)
        self.assertEqual(child.cancels, 0); self.assertTrue(n.gate)

    def test_non_t1_untrust_trips_immediately(self):
        for name in ('T5', 'T8', 'not_ready'):
            n, c = make(); child = n.child
            tick(n, c, 0.05, False, False)
            self.assertTrue(n.latched, name); self.assertEqual(child.cancels, 1)
            self.assertEqual(n.fault, 'localization_untrusted')

    def test_t1_then_t5_trips(self):
        n, c = make(); child = n.child
        tick(n, c, 1.0, False, True, adapter=LIVE_PAUSED)
        tick(n, c, 0.05, False, False)
        self.assertTrue(n.latched); self.assertEqual(child.cancels, 1)

    def test_missing_trust_status_trips_after_classification_window(self):
        n, c = make(); child = n.child
        tick(n, c, 0.5, False, True, status=False)
        self.assertFalse(n.latched); self.assertFalse(n.gate)                  # gate closed, zero sent at once
        self.assertEqual(n.pub.msgs[-1], Twist())
        tick(n, c, 0.3, False, True, status=False)
        self.assertTrue(n.latched); self.assertEqual(child.cancels, 1)

    def test_trust_node_silent_trips(self):
        n, c = make(); child = n.child
        for _ in range(20): c.t += 0.05; n.check()
        self.assertTrue(n.latched); self.assertEqual(child.cancels, 1)

    def test_adapter_latch_and_operator_stop_still_trip(self):
        n, c = make()
        n.adapter(String(data=json.dumps(dict(LIVE_PAUSED, state='latched', latch='foreign_velocity_publisher'))))
        self.assertTrue(n.latched); self.assertEqual(n.fault, 'foreign_velocity_publisher')
        n, c = make()
        n.stop(None, type('R', (), {})()); self.assertTrue(n.latched); self.assertEqual(n.child.cancels, 1)

    def test_adapter_not_ready_without_t1_pause_trips(self):
        n, c = make()
        n.adapter(String(data=json.dumps(dict(LIVE_PAUSED, t1_pause=False))))
        self.assertTrue(n.latched); self.assertEqual(n.fault, 'driver_not_ready')


class ObservedOrderingP10(unittest.TestCase):
    """P10 (live 2026-10-28, bag goal_bag_mcap_215612): the trust node publishes status BEFORE the Bool
    (status 1793196780.4561 trusted=false t1_transient=false reasons=[T5_map_odom_jump, not_ready:...],
    Bool False 1793196780.4566, Nav2 cancel 1793196780.4755). Tests replay that ordering."""

    def status(self, n, trusted, t1, reasons=()):
        n.trust_status(String(data=json.dumps(dict(trusted=trusted, ready=trusted, t1_transient=t1, reasons=list(reasons)))))

    def test_observed_t5_status_then_bool_trips_once(self):
        n, c = make(); child = n.child
        c.t += 0.0005; self.status(n, False, False, ['T5_map_odom_jump', 'not_ready:awaiting_initialpose_and_amcl'])
        self.assertTrue(n.latched); self.assertEqual(n.fault, 'localization_untrusted'); self.assertEqual(child.cancels, 1)
        c.t += 0.0005; n.trust(Bool(data=False))
        self.assertIsNone(n.pause_since)          # no misleading 'paused pending T1' after the trip
        for _ in range(80): c.t += 0.05; n.check()
        self.assertEqual(child.cancels, 1); self.assertEqual(n.fault, 'localization_untrusted')

    def test_status_first_t1_pauses_and_continues(self):
        n, c = make(); child = n.child
        for _ in range(10):                        # 2 s of T1-only loss, status then Bool at 5 Hz
            self.status(n, False, True, ['T1_scan_stale']); n.trust(Bool(data=False))
            for _ in range(4): c.t += 0.05; n.check()
        self.assertFalse(n.latched); self.assertEqual(child.cancels, 0); self.assertFalse(n.gate)
        self.status(n, True, False); n.trust(Bool(data=True)); c.t += 0.05; n.check()
        self.assertTrue(n.gate); self.assertIsNone(n.pause_since); self.assertEqual(child.cancels, 0)

    def test_bool_first_with_previous_trusted_status_waits_for_classifying_status(self):
        n, c = make(); child = n.child
        self.status(n, True, False); c.t += 0.15
        n.trust(Bool(data=False))                  # Bool before the status that explains it
        for _ in range(2): c.t += 0.05; n.check()
        self.assertFalse(n.latched); self.assertEqual(child.cancels, 0)
        self.status(n, False, True, ['T1_scan_stale'])
        for _ in range(10): c.t += 0.05; n.check()
        self.assertFalse(n.latched); self.assertEqual(child.cancels, 0)

    def test_status_first_t1_longer_than_limit_cancels(self):
        n, c = make(); child = n.child
        for _ in range(17):                        # 3.4 s > t1_pause_max_s
            self.status(n, False, True, ['T1_scan_stale']); n.trust(Bool(data=False))
            for _ in range(4): c.t += 0.05; n.check()
        self.assertTrue(n.latched); self.assertEqual(n.fault, 'localization_untrusted_t1_prolonged'); self.assertEqual(child.cancels, 1)


if __name__ == '__main__':
    unittest.main()
