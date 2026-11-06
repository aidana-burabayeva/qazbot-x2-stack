"""SDK input-source registration sequence (T5 2026-11-06, operator option C): ADD, on failure exactly one ENABLE
(AimDK X2 1.1.0 5.1.3; official example mc_locomotion_velocity.py:89-97). No ROS, no robot, no MC."""
import os
import sys
import unittest

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')))
from x2_motion_adapter import (IDLE, REG_ADD, REG_ENABLE, AdapterCore, Config,  # noqa: E402
                               RegistrationSequence, driver_input)
from test_adapter_core import OURS, Sim, published  # noqa: E402


class FakeSdk:
    """Scripted SetMcInputSource: one reply (code or exception) per call, records the actions it received."""

    def __init__(self, *replies):
        self.replies, self.calls = list(replies), []

    def drive(self, seq):
        """Runs the sequence the way the node does (start, then one call per returned action)."""
        action = seq.start()
        logs = []
        while action is not None:
            self.calls.append(action)
            reply = self.replies.pop(0)
            if isinstance(reply, Exception):
                action, level, text = seq.on_exception(reply)
            else:
                action, level, text = seq.on_response(reply, task_id=7, state_value=2)
            logs.append((level, text))
        return logs


class Sequence(unittest.TestCase):
    def test_cold_start_after_reboot_add_only(self):
        seq, sdk = RegistrationSequence(), FakeSdk(0)
        logs = sdk.drive(seq)
        self.assertEqual(sdk.calls, [REG_ADD])                      # ENABLE never sent
        self.assertEqual(seq.state, 'registered')
        self.assertTrue(seq.registered)
        self.assertEqual([lv for lv, _ in logs], ['info'])

    def test_restart_with_existing_source_add_then_enable(self):
        seq, sdk = RegistrationSequence(), FakeSdk(1, 0)
        logs = sdk.drive(seq)
        self.assertEqual(sdk.calls, [REG_ADD, REG_ENABLE])
        self.assertTrue(seq.registered)
        self.assertEqual(logs[0][0], 'warn')
        self.assertIn('likely', logs[0][1])                          # cites the example's assumption
        self.assertIn('code=1 task_id=7 state=2', logs[0][1])

    def test_both_fail(self):
        seq, sdk = RegistrationSequence(), FakeSdk(1, 1)
        logs = sdk.drive(seq)
        self.assertEqual(sdk.calls, [REG_ADD, REG_ENABLE])          # exactly two calls, nothing after ENABLE
        self.assertEqual(seq.state, 'failed')
        self.assertFalse(seq.registered)                             # node never creates the MC publisher
        self.assertEqual(logs[-1][0], 'error')
        for part in ('ADD code=1 task_id=7 state=2', 'ENABLE code=1 task_id=7 state=2'):
            self.assertIn(part, seq.error)
        self.assertIsNone(seq.start())                               # one sequence per process

    def test_add_exception_fails_without_enable(self):
        seq, sdk = RegistrationSequence(), FakeSdk(RuntimeError('service unavailable'))
        logs = sdk.drive(seq)
        self.assertEqual(sdk.calls, [REG_ADD])
        self.assertEqual(seq.state, 'failed')
        self.assertIn('ADD raised', seq.error)
        self.assertEqual(logs[-1][0], 'error')

    def test_enable_exception_fails(self):
        seq, sdk = RegistrationSequence(), FakeSdk(1, RuntimeError('gone'))
        sdk.drive(seq)
        self.assertEqual(sdk.calls, [REG_ADD, REG_ENABLE])
        self.assertEqual(seq.state, 'failed')
        self.assertIn('ENABLE raised after ADD code=1', seq.error)

    def test_start_only_once_and_no_unsolicited_response(self):
        seq = RegistrationSequence()
        self.assertRaises(RuntimeError, seq.on_response, 0)
        self.assertEqual(seq.start(), REG_ADD)
        self.assertIsNone(seq.start())                               # e.g. ~/register after the automatic start
        seq.on_response(0)
        self.assertRaises(RuntimeError, seq.on_response, 0)


class GateAfterRegistration(unittest.TestCase):
    """Node tick = driver_input() then AdapterCore.step(); reproduced here without ROS."""

    def _tick(self, s, reg_state, armed, resend=None):
        out = []
        for _ in range(int(0.5 * s.cfg.publish_hz)):
            s.t += 1.0 / s.cfg.publish_hz
            s.mc.stamp = s.t
            if resend is not None:
                s.send(*resend)
            cmd, stamp, blocked = driver_input(True, reg_state, armed, s.cmd, s.cmd_t)
            s.cmd, s.cmd_t = cmd, stamp
            if not blocked:
                out.append(s.core.step(s.t, s.cmd, s.cmd_t, s.mc))
        return out

    def test_failed_or_pending_registration_never_drives(self):
        for state in ('not_requested', 'pending', 'failed'):
            s = Sim()
            self.assertEqual(driver_input(True, state, True, (.3, 0, 0), s.t)[2], 'registration_' + state)
            self.assertEqual(published(self._tick(s, state, True, resend=(.3, 0, 0))), [], state)

    def test_idle_after_registration_until_armed_and_fresh_command(self):
        s = Sim()
        s.core.resume_after = s.t                                    # set on registration
        self.assertEqual(published(self._tick(s, 'registered', False)), [])          # idle: silent
        self.assertEqual(published(self._tick(s, 'registered', False, resend=(.3, 0, 0))), [])   # not armed
        self.assertEqual(s.core.state, IDLE)
        s.send(.3, 0, 0)                                             # queued before arming
        s.core.resume_after = s.t + 1e-6                             # ~/arm_motion: resume_after = now
        r = self._tick(s, 'registered', True)
        self.assertEqual(published(r), [])                           # pre-arm command never replayed
        s.mc.source = OURS
        r = self._tick(s, 'registered', True, resend=(.3, 0, 0))     # fresh authorized command
        self.assertEqual(published(r)[0], (.3, 0., 0.))

    def test_dry_run_unaffected_by_registration(self):
        self.assertEqual(driver_input(False, 'dry_run', True, (.3, 0, 0), 1.0), ((.3, 0, 0), 1.0, ''))
        self.assertEqual(driver_input(False, 'dry_run', False, (.3, 0, 0), 1.0)[0], None)


class LatchKeepsMotionUnarmed(unittest.TestCase):
    """Item 4 (existing behaviour): after a takeover latch and ~/resume the node is unarmed, so an old goal's commands
    are dropped by driver_input even though the core itself is IDLE again."""

    def test_resume_without_new_arm_drops_old_goal_commands(self):
        core = AdapterCore(Config(live=True))
        core.stop('operator_takeover')
        self.assertTrue(core.resume(now=10.0))
        armed = False                                                # _resume() sets motion_armed = False
        self.assertEqual(driver_input(True, 'registered', armed, (.3, 0, 0), 11.0), (None, float('-inf'), ''))


if __name__ == '__main__':
    unittest.main()
