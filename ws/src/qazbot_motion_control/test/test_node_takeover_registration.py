"""Node-level tests of x2_motion_adapter.py (T6 2026-11-06): the real node runs as a subprocess (live:=true) against a
fake MC / SDK / remote controller in this process, on an isolated graph (ROS_DOMAIN_ID != 0, forced to 88 if unset or
0; ROS_LOCALHOST_ONLY=1). Exercises the real rclpy logger path that the ROS-free tests could not catch
(G2b: "Logger severity cannot be changed between calls" after ADD code=1 -> ENABLE) and the manual-takeover wiring
(TakeoverMonitor -> latch while IDLE inside a goal, no resume after release, new motion only after ~/resume + a NEW
~/arm_motion). Fakes follow AimDK X2 1.1.0 5.1.3 (SetMcInputSource, input-source arbitration) and 5.1.2
(/aima/mc/locomotion/velocity, the remote controller streams source='rc'). Needs aimdk_msgs (skipped otherwise)."""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
    os.environ['ROS_DOMAIN_ID'] = '88'                 # never the robot's graph
os.environ['ROS_LOCALHOST_ONLY'] = '1'
TMP = tempfile.mkdtemp(prefix='t6_adapter_node_')
os.environ['ROS_LOG_DIR'] = os.path.join(TMP, 'ros_log')     # no ~/.ros/log entries

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from aimdk_msgs.msg import McCommonState, McLocomotionVelocity
    from aimdk_msgs.srv import SetMcInputSource
    IMPORT_ERROR = None
except ImportError as exc:                              # pragma: no cover - environment without aimdk_msgs
    IMPORT_ERROR = exc

SCRIPT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts',
                                       'x2_motion_adapter.py'))
OURS = 'sdk_qazbot_nav2'
NS = '/t6'
ADD, ENABLE = 1001, 2001
WAIT_S = 15.0                                           # upper bound of a condition wait, not a sleep


def setUpModule():
    if IMPORT_ERROR is not None:
        raise unittest.SkipTest(f'rclpy/aimdk_msgs not importable: {IMPORT_ERROR}')
    rclpy.init()


def tearDownModule():
    if IMPORT_ERROR is None:
        rclpy.try_shutdown()
    shutil.rmtree(TMP, ignore_errors=True)


class Harness:
    """Fake MC state (10 Hz), remote controller stream (45 Hz), localization inputs, Nav2 authorized commands and the
    SetMcInputSource service. Named like the vendor rc node so the adapter's publisher scan accepts it."""

    def __init__(self, codes, service=True):
        self.node = rclpy.create_node('soc0_rc1')
        n = self.node
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.codes, self.calls = list(codes), []
        self.mc_source, self.rc, self.cmd = 'rc', (0.0, 0.0, 0.0), None
        self.ours, self.status, self.status_count = [], {}, 0
        self.state_pub = n.create_publisher(McCommonState, '/aima/mc/common/state', be)
        self.mc_pub = n.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', be)
        self.tf_pub = n.create_publisher(TFMessage, NS + '/tf', QoSProfile(depth=50))
        self.trust_pub = n.create_publisher(Bool, NS + '/trusted', 10)
        self.trust_status_pub = n.create_publisher(String, NS + '/trust_status', 10)
        self.cmd_pub = n.create_publisher(Twist, NS + '/cmd', 10)
        n.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self._on_mc, be)
        n.create_subscription(String, NS + '/motion_adapter/status', self._on_status, 10)
        if service:
            self.srv = n.create_service(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource', self._on_set)
        self.clients = {k: n.create_client(Trigger, f'{NS}/motion_adapter/{k}')
                        for k in ('arm_motion', 'disarm_motion', 'resume')}
        n.create_timer(0.1, self._state)
        n.create_timer(1.0 / 45, self._rc)
        n.create_timer(0.05, self._cmd)
        n.create_timer(0.2, self._loc)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)

    # -- fakes
    def _on_set(self, req, resp):
        self.calls.append(req.action.value)
        resp.response.header.code = self.codes.pop(0)
        resp.response.task_id = len(self.calls)
        return resp

    def _state(self):
        m = McCommonState()
        m.input_source.name = self.mc_source
        m.body_status.value, m.fsm_state.current_state = 1, 4            # STAND, STABLE (5.1.4)
        self.state_pub.publish(m)

    def _rc(self):
        m = McLocomotionVelocity()
        m.source = 'rc'
        m.forward_velocity, m.lateral_velocity, m.angular_velocity = self.rc
        self.mc_pub.publish(m)

    def _cmd(self):
        if self.cmd is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = self.cmd
            self.cmd_pub.publish(t)

    def _loc(self):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[t]))
        self.trust_pub.publish(Bool(data=True))
        self.trust_status_pub.publish(String(data=json.dumps({'trusted': True, 'ready': True})))

    def _on_mc(self, m):
        if m.source == OURS:
            self.ours.append((m.forward_velocity, m.lateral_velocity, m.angular_velocity))

    def _on_status(self, m):
        self.status, self.status_count = json.loads(m.data), self.status_count + 1

    # -- helpers
    def spin_until(self, pred, what):
        deadline = time.monotonic() + WAIT_S
        while time.monotonic() < deadline:
            self.ex.spin_once(timeout_sec=0.05)
            if pred():
                return
        raise AssertionError(f'timeout waiting for {what}; status={self.status}')

    def observe(self, seconds):
        """Bounded observation window for a NEGATIVE check (something must not happen)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)

    def call(self, name):
        c = self.clients[name]
        self.spin_until(c.service_is_ready, name + ' service')
        f = c.call_async(Trigger.Request())
        self.spin_until(f.done, name + ' response')
        return f.result()

    def non_zero_since(self, i):
        return [v for v in self.ours[i:] if any(x != 0.0 for x in v)]

    def mc_publisher_nodes(self):
        return {i.node_name for i in self.node.get_publishers_info_by_topic('/aima/mc/locomotion/velocity')}

    def destroy(self):
        self.ex.remove_node(self.node)
        self.node.destroy_node()


class AdapterProcess:
    def __init__(self, test):
        self.log_path = os.path.join(TMP, f'{test.id().rsplit(".", 1)[-1]}.log')
        self.log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(
            [sys.executable, SCRIPT, '--ros-args', '-r', '__ns:=' + NS, '-p', 'live:=true',
             '-p', 'lock_path:=' + os.path.join(TMP, 'owner.lock'), '-p', f'cmd_vel_topic:={NS}/cmd',
             '-p', f'tf_topic:={NS}/tf', '-p', f'trust_topic:={NS}/trusted', '-p', f'trust_status_topic:={NS}/trust_status',
             '-p', f'odom_topic:={NS}/odom'],
            stdout=self.log, stderr=subprocess.STDOUT, env=dict(os.environ))

    def alive(self):
        return self.proc.poll() is None

    def text(self):
        self.log.flush()
        with open(self.log_path) as f:
            return f.read()

    def stop(self):
        if self.alive():
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.log.close()


class NodeCase(unittest.TestCase):
    codes, service = (0,), True

    def setUp(self):
        self.h = Harness(self.codes, self.service)
        self.a = AdapterProcess(self)

    def tearDown(self):
        self.a.stop()
        self.h.destroy()

    def assert_alive_without_traceback(self):
        out = self.a.text()
        self.assertTrue(self.a.alive(), out)
        self.assertNotIn('Traceback', out)
        self.assertNotIn('Logger severity cannot be changed', out)

    def wait_registered_ready(self):
        self.h.spin_until(lambda: self.h.status.get('registration') == 'registered' and self.h.status.get('ready'),
                          'registered and ready')

    def drive_goal(self):
        """New goal: arm, Nav2 commands flow to MC with our source; MC selects our source."""
        self.assertTrue(self.h.call('arm_motion').success)
        i = len(self.h.ours)
        self.h.cmd = (0.3, 0.0, 0.0)
        self.h.spin_until(lambda: self.h.non_zero_since(i), 'our non-zero MC command')
        self.h.mc_source = OURS
        self.h.spin_until(lambda: self.h.status.get('state') == 'active' and self.h.status.get('mc_source') == OURS,
                          'active with our source')

    def nav2_gap(self):
        self.h.cmd = None                                               # T1 pause / Nav2 command gap: adapter IDLE
        self.h.spin_until(lambda: self.h.status.get('state') == 'idle', 'idle inside the goal')


class AddFailsEnableSucceeds(NodeCase):
    codes = (1, 0)

    def test_add_1_enable_0_registers_and_node_survives(self):
        self.wait_registered_ready()
        self.assertEqual(self.h.calls, [ADD, ENABLE])
        self.h.observe(1.0)                                             # status keeps coming: node is alive
        self.assert_alive_without_traceback()
        out = self.a.text()
        self.assertIn('ADD failed (code=1', out)
        self.assertIn('SDK source registered by ENABLE', out)
        self.assertIn('motion_adapter', ''.join(self.h.mc_publisher_nodes()))


class AddFailsEnableFails(NodeCase):
    codes = (1, 1)

    def test_add_1_enable_1_fails_closed_and_node_survives(self):
        self.h.spin_until(lambda: self.h.status.get('registration') == 'failed', 'registration failed')
        self.assertEqual(self.h.calls, [ADD, ENABLE])
        self.h.observe(1.0)
        self.assert_alive_without_traceback()
        self.assertIn('SDK registration failed', self.a.text())
        self.assertFalse(self.h.status.get('ready'))
        self.assertNotIn('motion_adapter', ''.join(self.h.mc_publisher_nodes()))   # no MC publisher
        self.assertFalse(self.h.call('arm_motion').success)


class SdkUnavailable(NodeCase):
    service = False

    def test_no_sdk_service_no_registration_no_publisher_no_arm(self):
        self.h.spin_until(lambda: self.h.status_count >= 3, 'status')
        s = self.h.status
        self.assertEqual((s['registration'], s['sdk_service_ready'], s['ready']), ('not_requested', False, False))
        self.assertNotIn('motion_adapter', ''.join(self.h.mc_publisher_nodes()))
        self.assertFalse(self.h.call('arm_motion').success)
        self.assert_alive_without_traceback()


class TakeoverDuringNav2Gap(NodeCase):
    codes = (1, 0)                                       # also the re-registration path (ADD 1 -> ENABLE 0)

    def test_rc_command_in_idle_gap_latches_release_does_not_resume_new_goal_needed(self):
        self.wait_registered_ready()
        self.drive_goal()                                               # normal navigation: commands flow
        self.nav2_gap()
        self.h.rc = (0.4, 0.0, 0.0)                                     # operator moves the stick (MC: rc wins)
        self.h.spin_until(lambda: self.h.status.get('state') == 'latched', 'takeover latch in IDLE')
        s = self.h.status
        self.assertEqual(s['latch'], 'operator_takeover')
        self.assertIn("'rc'", s['takeover_evidence'])
        self.assertFalse(s['motion_armed'])
        self.h.mc_source = 'rc'
        self.h.observe(0.5)                                             # operator drives
        self.h.rc, self.h.mc_source = (0.0, 0.0, 0.0), OURS             # release; rc lease expired
        i = len(self.h.ours)
        self.h.cmd = (0.3, 0.0, 0.0)                                    # Nav2 still on the OLD goal
        self.h.observe(1.5)
        self.assertEqual(self.h.non_zero_since(i), [])                  # the old goal does not resume
        self.assertEqual(self.h.status['state'], 'latched')
        self.assertTrue(self.h.call('resume').success)                  # explicit operator acknowledgement
        self.h.spin_until(lambda: self.h.status.get('state') == 'idle', 'idle after resume')
        i = len(self.h.ours)
        self.h.observe(1.0)
        self.assertEqual(self.h.non_zero_since(i), [])                  # resume alone does not move the robot
        self.assertEqual(self.h.status['reason'], 'driver_not_armed')
        self.drive_goal()                                               # only a NEW goal (arm) moves again
        self.assert_alive_without_traceback()


class SourceEdgeDuringNav2Gap(NodeCase):
    def test_mc_source_leaving_ours_in_idle_gap_latches(self):
        self.wait_registered_ready()
        self.drive_goal()
        self.nav2_gap()
        self.h.observe(0.5)
        self.assertEqual(self.h.status['state'], 'idle')                # same source while idle: not a takeover
        self.h.mc_source = 'rc'                                         # edge ours -> rc, no rc command seen
        self.h.spin_until(lambda: self.h.status.get('state') == 'latched', 'takeover latch from source edge')
        self.assertIn(f'{OURS} -> rc', self.h.status['takeover_evidence'])
        self.assert_alive_without_traceback()


class RcBetweenGoals(NodeCase):
    def test_rc_after_disarm_does_not_latch(self):
        self.wait_registered_ready()
        self.drive_goal()
        self.h.cmd = None
        self.assertTrue(self.h.call('disarm_motion').success)           # coordinator: goal terminal
        self.h.spin_until(lambda: self.h.status.get('motion_armed') is False, 'disarmed')
        self.h.rc, self.h.mc_source = (0.4, 0.0, 0.0), 'rc'             # operator repositions the robot
        self.h.observe(1.0)
        self.assertNotEqual(self.h.status['state'], 'latched')
        self.h.rc = (0.0, 0.0, 0.0)
        self.drive_goal()                                               # next goal is accepted without resume
        self.assert_alive_without_traceback()


if __name__ == '__main__':
    unittest.main()
