"""Unit tests for the X2 motion adapter decision core. No ROS, no robot, no MC.

Run: python3 -m unittest -v tests/motion_adapter/test_adapter_core.py
"""
import math
import os
import sys
import tempfile
import unittest

def _nav_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in (('..', '..', 'ws', 'src', 'navigation'), ('..', '..', 'deploy', 'navigation')):
        p = os.path.normpath(os.path.join(here, *rel))
        if os.path.isdir(p):
            return p
    raise RuntimeError('navigation directory not found')


sys.path.insert(0, _nav_dir())
from x2_motion_adapter import (ACTIVE, BODY_STAND, DRY_RUN, FSM_MOVING, FSM_STABLE, IDLE, LATCHED,  # noqa: E402
                               AdapterCore, Config, McState, OwnerLock, validate_source_name)

OURS = 'sdk_qazbot_nav2'


class Sim:
    """Drives the core at 50 Hz with a controllable MC state and Nav2 command."""

    def __init__(self, live=True, **cfg):
        self.cfg = Config(live=live, **cfg)
        self.core = AdapterCore(self.cfg)
        self.t = 100.0
        self.cmd, self.cmd_t = None, float('-inf')
        self.mc = McState(stamp=self.t, source='rc', body=BODY_STAND, fsm=FSM_STABLE)
        self.localized = True
        self.loc_reason, self.foreign_pub = '', ''
        self.out = []                                  # (t, publish, state, reason)

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmd, self.cmd_t = (vx, vy, wz), self.t

    def run(self, seconds, resend=None, mc_source=None):
        n = int(round(seconds * self.cfg.publish_hz))
        res = []
        for _ in range(n):
            self.t += 1.0 / self.cfg.publish_hz
            self.mc.stamp = self.t                      # MC state keeps arriving (10 Hz is enough for these tests)
            if mc_source is not None:
                self.mc.source = mc_source
            if resend is not None:
                self.send(*resend)
            d = self.core.step(self.t, self.cmd, self.cmd_t, self.mc, self.localized, self.loc_reason, self.foreign_pub)
            res.append(d)
            self.out.append((self.t, d.publish, d.state, d.reason))
        return res


def published(decisions):
    return [d.publish for d in decisions if d.publish is not None]


class Nominal(unittest.TestCase):
    def test_route_passes_commands_through(self):
        s = Sim()
        r = s.run(1.0, resend=(0.30, 0.0, 0.10), mc_source=OURS)
        self.assertTrue(all(d.state == ACTIVE for d in r))
        self.assertTrue(all(p == (0.30, 0.0, 0.10) for p in published(r)))

    def test_small_speed_is_not_raised_to_a_minimum(self):
        # Nova-Valley raises 0.05 m/s to 0.2 m/s; the X2 SDK example rejects it. We pass it through.
        s = Sim()
        r = s.run(0.5, resend=(0.05, 0.0, 0.02), mc_source=OURS)
        self.assertEqual(published(r)[0], (0.05, 0.0, 0.02))

    def test_turn_in_place(self):
        s = Sim()
        r = s.run(0.5, resend=(0.0, 0.0, 0.4), mc_source=OURS)
        self.assertEqual(published(r)[0], (0.0, 0.0, 0.4))

    def test_saturates_to_live_mc_bounds_and_counts(self):
        s = Sim()
        r = s.run(0.2, resend=(1.5, 0.0, -3.0), mc_source=OURS)
        self.assertEqual(published(r)[0], (0.8, 0.0, -1.0))
        self.assertGreater(s.core.counters['saturated'], 0)

    def test_follows_a_change_of_mc_speed_mode(self):
        s = Sim()
        s.mc.fwd = (-1.0, 1.2)                          # MEDIUM
        r = s.run(0.2, resend=(1.1, 0.0, 0.0), mc_source=OURS)
        self.assertEqual(published(r)[0][0], 1.1)


class EndOfMotion(unittest.TestCase):
    def test_cancel_gives_short_zero_burst_then_silence(self):
        s = Sim()
        s.run(0.5, resend=(0.3, 0, 0), mc_source=OURS)
        r = s.run(1.0, resend=(0.0, 0.0, 0.0))          # Nav2 cancel: controller sends zero
        self.assertEqual(published(r), [(0.0, 0.0, 0.0)] * s.cfg.zero_burst)
        self.assertEqual(s.core.state, IDLE)

    def test_lost_commands_stop_within_three_controller_periods(self):
        s = Sim()
        s.run(0.5, resend=(0.3, 0, 0), mc_source=OURS)
        r = s.run(1.0)                                   # Nav2 stops publishing
        sent = published(r)
        first_zero = sent.index((0.0, 0.0, 0.0))
        # the last command is repeated only while it is still "fresh" (3 controller periods), then zeros
        self.assertLessEqual(first_zero, round(s.cfg.cmd_timeout_s * s.cfg.publish_hz))
        self.assertTrue(all(p == (0.3, 0.0, 0.0) for p in sent[:first_zero]))
        self.assertEqual(sent[first_zero:], [(0.0, 0.0, 0.0)] * s.cfg.zero_burst)
        self.assertEqual(s.core.state, IDLE)

    def test_idle_is_silent(self):
        s = Sim()
        self.assertEqual(published(s.run(5.0)), [])      # no zero stream while nothing is happening

    def test_impossible_goal_means_no_command_and_no_motion(self):
        s = Sim()
        s.send(0, 0, 0)
        self.assertEqual(published(s.run(2.0, resend=(0, 0, 0))), [])
        self.assertEqual(s.core.state, IDLE)


class Latching(unittest.TestCase):
    def _driving(self):
        s = Sim()
        s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.state, ACTIVE)
        return s

    def test_localization_loss_latches_and_needs_resume(self):
        s = self._driving()
        s.localized = False
        r = s.run(0.3, resend=(0.3, 0, 0))
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'localization_lost')
        self.assertEqual(published(r), [(0.0, 0.0, 0.0)] * s.cfg.zero_burst)
        s.localized = True
        self.assertEqual(published(s.run(2.0, resend=(0.3, 0, 0))), [])     # recovered pose alone is not enough
        self.assertTrue(s.core.resume(now=s.t))
        self.assertEqual(published(s.run(0.2)), [])                         # the pre-resume command is ignored
        self.assertEqual(published(s.run(0.2, resend=(0.3, 0, 0), mc_source=OURS))[0], (0.3, 0.0, 0.0))

    def test_operator_takeover_latches_and_never_auto_resumes(self):
        s = self._driving()
        s.run(1.0, resend=(0.3, 0, 0), mc_source='rc')   # PS5 takes control (rc priority 80)
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'operator_takeover')
        # rc times out (1000 ms) and Nav2 keeps commanding: MC would hand control back; we must stay silent
        self.assertEqual(published(s.run(3.0, resend=(0.3, 0, 0), mc_source=OURS)), [])
        self.assertEqual(s.core.state, LATCHED)

    def test_control_never_acquired_when_operator_holds_control(self):
        s = Sim()
        r = s.run(1.0, resend=(0.3, 0, 0), mc_source='rc')
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'control_not_acquired')
        self.assertLessEqual(len(published(r)), int(s.cfg.acquire_grace_s * s.cfg.publish_hz) + 2 + s.cfg.zero_burst)

    def test_stop_service_latches(self):
        s = self._driving()
        s.core.stop()
        r = s.run(0.3, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'stop_requested')
        self.assertEqual(published(r), [(0.0, 0.0, 0.0)] * s.cfg.zero_burst)

    def test_robot_not_standing_refuses_motion(self):
        s = Sim()
        s.mc.body = 4                                    # SIT
        self.assertEqual(published(s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)), [])
        self.assertEqual(s.core.latch_reason, 'robot_not_standing')

    def test_falling_while_driving_latches(self):
        s = self._driving()
        s.mc.body = 6                                    # FALLING
        s.run(0.2, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'robot_not_standing')

    def test_fsm_safe_refuses_motion(self):
        s = Sim()
        s.mc.fsm = 6                                     # SAFE
        s.run(0.5, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'mc_fsm_not_stable')

    def test_moving_fsm_is_allowed(self):
        s = Sim()
        s.mc.fsm = FSM_MOVING
        self.assertTrue(published(s.run(0.3, resend=(0.3, 0, 0), mc_source=OURS)))

    def test_stale_mc_state_refuses_motion(self):
        s = Sim()
        s.send(0.3, 0, 0)
        s.t += 5.0
        s.send(0.3, 0, 0)
        d = s.core.step(s.t, s.cmd, s.cmd_t, s.mc, True)                # mc.stamp is 5 s old
        self.assertIsNone(d.publish)
        self.assertEqual(s.core.latch_reason, 'mc_state_stale')

    def test_foreign_sdk_source_is_refused(self):
        s = Sim()
        s.run(0.5, resend=(0.3, 0, 0), mc_source='sdk_continuous_walker')
        self.assertEqual(s.core.latch_reason, 'foreign_sdk_source_active')

    def test_lateral_is_refused_by_default(self):
        s = Sim()
        s.run(0.3, resend=(0.2, 0.3, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'lateral_not_allowed')

    def test_non_finite_command_latches(self):
        s = self._driving()
        s.run(0.2, resend=(math.nan, 0, 0), mc_source=OURS)
        self.assertEqual(s.core.latch_reason, 'non_finite_command')


class DryRun(unittest.TestCase):
    """Dry run runs the same shadow state machine as live; only the output topic differs (node level).
    Superseded 2026-10-21 (task 20261021-x2-nav2-e2e-dryrun G1): the old tests asserted that dry run
    never enters live states, never latches and sends no zero burst."""

    def test_dry_run_runs_shadow_states(self):
        s = Sim(live=False)
        r = s.run(1.0, resend=(0.3, 0, 0.1), mc_source=OURS)
        self.assertTrue(all(d.state == ACTIVE for d in r))
        self.assertEqual(published(r)[0], (0.3, 0.0, 0.1))
        self.assertTrue(s.core.dry_run)

    def test_dry_run_mirrors_live_zero_burst(self):
        s = Sim(live=False)
        s.run(0.5, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(published(s.run(1.0, resend=(0, 0, 0))), [(0.0, 0.0, 0.0)] * s.cfg.zero_burst)
        self.assertEqual(s.core.state, IDLE)

    def test_dry_run_latches_on_faults_like_live(self):
        s = Sim(live=False)
        s.localized = False
        r = s.run(0.5, resend=(0.3, 0, 0), mc_source=OURS)
        self.assertEqual(published(r), [])
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(s.core.latch_reason, 'localization_lost')


class Ownership(unittest.TestCase):
    def test_owner_lock_is_exclusive(self):
        path = os.path.join(tempfile.mkdtemp(), 'x2_motion_owner.lock')
        a, b = OwnerLock(path), OwnerLock(path)
        a.acquire()
        with self.assertRaises(RuntimeError):
            b.acquire()
        a.release()
        b.acquire()                                       # free again after release
        b.release()

    def test_builtin_and_unprefixed_source_names_are_refused(self):
        for bad in ('rc', 'pnc', 'linkswarm', 'app_proxy', 'nav2', 'node.locomotion_velocity'):
            with self.assertRaises(ValueError):
                validate_source_name(bad)
        self.assertEqual(validate_source_name(OURS), OURS)

    def test_default_cmd_timeout_is_below_mc_silence_zeroing(self):
        self.assertLess(Config().cmd_timeout_s, 0.2)      # MC zeroes silent commands after 200 ms


class TrustAndOwnership(unittest.TestCase):
    def _driving(self):
        s = Sim()
        s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)
        return s

    def test_untrusted_localization_latches_with_its_own_reason(self):
        s = self._driving()
        s.localized, s.loc_reason = False, 'localization_untrusted'
        s.run(0.3, resend=(0.3, 0, 0))
        self.assertEqual(s.core.latch_reason, 'localization_untrusted')

    def test_untrusted_refuses_to_start(self):
        s = Sim()
        s.localized, s.loc_reason = False, 'localization_untrusted'
        self.assertEqual(published(s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)), [])
        self.assertEqual(s.core.state, LATCHED)

    def test_foreign_publisher_latches_while_driving(self):
        s = self._driving()
        s.foreign_pub = 'continuous_walker'
        s.run(0.3, resend=(0.3, 0, 0))
        self.assertEqual(s.core.latch_reason, 'foreign_velocity_publisher')

    def test_foreign_publisher_blocks_start_and_clearing_it_alone_does_not_resume(self):
        s = Sim()
        s.foreign_pub = 'continuous_walker'
        self.assertEqual(published(s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)), [])
        self.assertEqual(s.core.state, LATCHED)
        s.foreign_pub = ''
        self.assertEqual(published(s.run(1.0, resend=(0.3, 0, 0), mc_source=OURS)), [])

    def test_vendor_autonomy_blocks_nav2(self):
        s = Sim()
        self.assertEqual(published(s.run(1.0, resend=(0.3, 0, 0), mc_source='pnc')), [])
        self.assertEqual(s.core.latch_reason, 'vendor_autonomy_active')

    def test_dry_run_foreign_publisher_latches(self):
        # superseded: test_dry_run_reports_but_never_latches (dry run now latches like live)
        s = Sim(live=False)
        s.foreign_pub = 'x'
        r = s.run(0.3, resend=(0.3, 0, 0))
        self.assertEqual(r[-1].reason, 'foreign_velocity_publisher')
        self.assertEqual(s.core.state, LATCHED)
        self.assertEqual(published(r), [])


if __name__ == '__main__':
    unittest.main()
