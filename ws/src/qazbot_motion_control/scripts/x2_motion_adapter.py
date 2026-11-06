#!/usr/bin/env python3
"""Nav2 /cmd_vel -> X2 MC (/aima/mc/locomotion/velocity) motion adapter.

Design rules (see docs/X2_MOTION_ADAPTER.md for the evidence behind each):
  * Dry run is the default. In dry run no publisher to the MC topic is created,
    the owner lock is not taken and no input source is registered, so the node
    cannot move the robot. Dry run runs the SAME idle/active/latched state machine
    ("shadow"): arming, fault refusals, latches, zero bursts and takeover detection
    behave as live; the only output is ~/dry_run_cmd (Twist). One unavoidable
    difference: MC cannot select a source that was never registered, so in dry run
    "control acquired" is not required; instead the MC input source observed when
    the shadow became active (or our own source name) is accepted, and any other
    source appearing while active is an operator takeover. On the stock robot the
    idle source is already 'rc' (docs/CURRENT_CONSTRAINTS.md), so the source name alone
    is no evidence; see TakeoverMonitor (non-zero foreign MC command or loss of our source,
    in IDLE as well as ACTIVE while a goal owns the driver).
  * Live mode needs live:=true. It then takes an exclusive owner lock, registers a
    source named sdk_* once, automatically, as soon as the SDK service is discovered (ADD, on failure one ENABLE:
    RegistrationSequence), and publishes only while armed with a fresh, valid, non-zero command.
  * Values are passed through, never raised to a minimum. Out-of-bounds values are
    saturated to the MC bounds and counted; non-finite or lateral values stop the robot.
    Exception (P20, 2026-10-29): a PURE in-place turn (|vx| < 0.02, |wz| > 0.05) is closed-loop on the measured
    yaw rate (/qazbot_native/odom yaw), because MC has a sustained-turn deadband and overspeeds above it; the
    output is sign(wz) * [u_min_dir, u_max]. Zero / stale input still gives zero output; see YawRateLoop.
  * Anything that is not the normal end of a goal (operator takeover, control not
    acquired, robot not standing, stale MC state, lost localization, stop request)
    LATCHES: no motion until resume() is called. MC otherwise lets an SDK source
    retake control after the operator releases the joystick.
  * Idle is silent. A zero stream would be "accepted" by MC and its effect on rc
    arbitration is unverified, so after a short zero burst nothing is published.

The decision logic is in AdapterCore (no ROS imports) so it is unit-tested without a robot.
"""
import fcntl
import json
import math
import os
import re
import time
from dataclasses import dataclass, field

BUILTIN_SOURCES = frozenset({'linkswarm', 'rc', 'app_proxy', 'vr', 'interaction', 'pnc'})
#: McBodyPoseStatus.value and McFsmState.current_state, AimDK X2 1.1.0 docs 5.1.4.
BODY_STAND = 1
FSM_STABLE, FSM_MOVING = 4, 5

DRY_RUN, IDLE, ACTIVE, LATCHED = 'dry_run', 'idle', 'active', 'latched'
#: Nodes seen publishing /aima/mc/locomotion/velocity on the stock robot (vendor sources, arbitrated by MC).
#: Anything else publishing there is a foreign motion owner. Live check 2026-10-21: aimrt_planning_planning_node4571,
#: soc0_rc3107, app_proxy, soc2_interaction_slave1577.
VENDOR_PUBLISHERS = (r'^aimrt_planning_planning_node\d*$', r'^soc\d_rc\d*$', r'^app_proxy$', r'^soc\d_interaction_slave\d*$')


@dataclass
class Config:
    source_name: str = 'sdk_qazbot_nav2'
    live: bool = False
    controller_hz: float = 20.0        # Nav2 controller_frequency
    publish_hz: float = 50.0           # AimDK recommended
    zero_burst: int = 5                # zero messages after a motion ends, then silence
    eps: float = 0.005                 # |v| below this is zero (AimDK example)
    allow_lateral: bool = False        # X2 lateral start threshold ~0.5-0.7 m/s: Nav2 should not ask
    acquire_grace_s: float = 0.5       # time for MC to select our source (state is 10 Hz)
    state_stale_s: float = 0.5         # MC state older than this means authority is unknown
    tf_max_age_s: float = 1.0          # map->odom older than this means localization is not live
    require_localization: bool = True
    trust_max_age_s: float = 0.6       # localization_trust verdict older than this counts as untrusted
    require_trust: bool = True         # fresh map->odom alone does not prove the pose is right (docs/X2_LOCALIZATION_TRUST.md)
    t1_pause_max_s: float = 3.0        # operator decision 2026-10-28: T1-only loss pauses (no latch) up to this long
    # P20 2026-10-29 closed-loop yaw rate for PURE in-place turns (MC deadband/overspeed, reports p19/p20)
    yaw_loop: bool = True
    yaw_lin_eps: float = 0.02          # |vx| below this (and vy=0) counts as pure rotation
    # P23 2026-10-30: was 0.05. RPP ramps wz at ~1.5 rad/s^2 (0.075 per 20 Hz cycle), so |wz| < 0.15 only occurs for
    # <= 2 cycles while ramping/reversing; lifting it to u_min (0.6) kicked the robot at full turn speed, possibly
    # against the current motion (live 1030: target +0.07 meas -0.35 -> u +0.60). Below this: raw pass-through.
    yaw_w_min: float = 0.15            # |wz| at or below this is passed through unchanged (no u_min lift)
    yaw_u_min_ccw: float = 0.6         # just above the sustained-turn threshold (calibration 18:40/18:50)
    yaw_u_min_cw: float = 0.45
    yaw_u_max: float = 1.0             # MC angular bound
    yaw_ki: float = 0.4                # integral gain [u per (rad/s * s)]
    yaw_du_max: float = 1.5            # rate limit on u [1/s] (no limit on the first sample)
    yaw_settle_s: float = 0.6          # MC dead time from stand ~0.5 s: no integration right after entry
    yaw_overspeed: float = 1.3         # measured > this * target -> back off twice as fast
    yaw_meas_max_age_s: float = 0.3    # older yaw-rate measurement -> pass-through
    # P23: RPP interleaves pure rotation with vx ramps, the trust gate zeroes for 0.6-0.9 s, odom has 0.7 s gaps; the
    # P20 loop reset on each of these, no pure-rotation run outlived yaw_settle_s and I stayed 0.00 (live 1030).
    yaw_hold_s: float = 1.0            # keep I across interruptions up to this long when the target sign is unchanged

    @property
    def cmd_timeout_s(self):
        # Three controller periods, below MC's own 200 ms zeroing of silent commands.
        return 3.0 / self.controller_hz


@dataclass
class McState:
    """Snapshot of /aima/mc/common/state; stamp is local receive time."""
    stamp: float = float('-inf')
    source: str = ''
    body: int = 0
    fsm: int = 0
    fwd: tuple = (-1.0, 0.8)           # live bounds, GetMcLocomotionSpeedStatus / speed_status
    lat: tuple = (-1.0, 1.0)
    ang: tuple = (-1.0, 1.0)


@dataclass
class Decision:
    state: str
    publish: tuple = None              # (forward, lateral, angular) to send now, or None
    reason: str = ''
    notes: list = field(default_factory=list)


#: Static MC curve, sustained chest yaw rate [rad/s] -> command |u| (from stand; p19 calibration). Feedforward only:
#: after walking the same u gives 0.2-0.37 rad/s, which the integral term has to make up.
YAW_FF = {1: ((0.45, 0.6), (1.25, 0.8), (1.87, 1.0)),
          -1: ((0.27, 0.4), (0.54, 0.6), (1.2, 0.8), (1.8, 1.0))}


def _interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


class YawRateEstimator:
    """Yaw rate from successive yaw samples (stamp-based dt), first-order low-pass. The native-odom twist is not
    used: live check 2026-10-29 showed angular.z 1.78 rad/s while standing still."""

    def __init__(self, tau=0.25):
        self.tau, self.last, self.rate, self.stamp = tau, None, None, float('-inf')

    def update(self, t_msg, yaw, t_recv):
        if self.last is not None:
            dt = t_msg - self.last[0]
            if 1e-3 < dt <= 0.5:
                r = math.remainder(yaw - self.last[1], 2 * math.pi) / dt
                a = dt / (self.tau + dt)
                self.rate = r if self.rate is None else self.rate + a * (r - self.rate)
                self.stamp = t_recv
            elif dt > 0.5 or dt < 0:
                self.rate = None                 # gap or time jump: restart the filter
        self.last = (t_msg, yaw)

    def get(self, now, max_age):
        return self.rate if self.rate is not None and now - self.stamp <= max_age else None


class YawRateLoop:
    """Inner loop for pure in-place turns: u = sign(w) * clamp(u_ff(|w|) + I, u_min_dir, u_max), rate-limited.
    Integrator: anti-windup (conditional + clamp), frozen during the MC dead time. P23: an exit only PAUSES the loop
    (output handed back, I kept and frozen); I is reset on a target sign change, a pause longer than yaw_hold_s, or
    reset() (latch). After a stop-type pause the MC dead time applies again (I frozen for yaw_settle_s)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.sign, self.integ, self.u, self.t_last, self.t_enter = 0, 0.0, None, None, None
        self.active, self.target, self.meas = False, 0.0, None
        self.t_pause, self.stopped = None, False

    def pause(self, now, stopped):
        """Loop not in control this tick. stopped: MC got zero / sub-deadband (dead time on resume)."""
        if self.sign == 0:
            return
        if self.t_pause is None:
            self.t_pause = now
        self.stopped = self.stopped or stopped
        self.active, self.u, self.t_last = False, None, None

    def update(self, now, w_cmd, meas):
        cfg = self.cfg
        s = 1 if w_cmd > 0 else -1
        if s != self.sign or (self.t_pause is not None and now - self.t_pause > cfg.yaw_hold_s):
            self.reset()
            self.sign, self.t_enter = s, now
        elif self.t_pause is not None:              # resume within hold, same sign: keep I
            if self.stopped:
                self.t_enter = now                  # robot stopped -> MC dead time again, I frozen meanwhile
            self.t_pause, self.stopped = None, False
        r = abs(w_cmd)
        u_min = cfg.yaw_u_min_ccw if s > 0 else cfg.yaw_u_min_cw
        u_max = max(cfg.yaw_u_max, u_min)
        ff = min(max(_interp(YAW_FF[s], r), u_min), u_max)
        dt = 0.0 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.1)
        self.t_last = now
        m = s * meas
        if now - self.t_enter >= cfg.yaw_settle_s:
            e = r - m
            u_raw = ff + self.integ
            if m > max(cfg.yaw_overspeed * r, r + 0.25):     # +0.25: filtered gait ripple is ~+-0.2 rad/s
                # overspeed (e.g. MC releases after a stall with a wound-up u): bleed u toward u_min at du_max
                self.integ -= cfg.yaw_du_max * dt
            elif not ((u_raw >= u_max and e > 0) or (u_raw <= u_min and e < 0)):
                self.integ += cfg.yaw_ki * e * dt
            self.integ = min(max(self.integ, u_min - ff), u_max - ff)
        u_t = min(max(ff + self.integ, u_min), u_max)
        self.u = u_t if self.u is None else self.u + min(max(u_t - self.u, -cfg.yaw_du_max * dt), cfg.yaw_du_max * dt)
        self.active, self.target, self.meas = True, w_cmd, meas
        return s * self.u


class AdapterCore:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.state = IDLE                  # same state machine in live and dry run (shadow)
        self.dry_run = not cfg.live
        self.shadow_source = None          # dry run only: MC source observed when the shadow became active
        self.latch_reason = ''
        self.active_since = None
        self.burst_left = 0
        self.resume_after = float('-inf')
        self.pause_since = None            # T1-only localization loss: paused (zero, not latched) since
        self.counters = {'saturated': 0, 'rejected': 0, 'latched': 0, 'published': 0, 'yaw_loop': 0, 'yaw_meas_stale': 0}
        self.yaw = YawRateLoop(cfg)

    # -- operator / service inputs ---------------------------------------------------
    def stop(self, reason='stop_requested'):
        self._latch(reason)

    def resume(self, now=None):
        """Explicit operator acknowledgement; only clears a latch.

        Commands stamped before this moment are ignored, so a Nav2 goal that kept
        running through the latch cannot restart the robot on its own."""
        if self.state == LATCHED:
            self.state, self.latch_reason = IDLE, ''
            self.resume_after = time.monotonic() if now is None else now
            return True
        return False

    def _latch(self, reason):
        if self.state != LATCHED:
            self.counters['latched'] += 1
            self.burst_left = self.cfg.zero_burst if self.state == ACTIVE else 0
        self.state, self.latch_reason, self.active_since, self.pause_since = LATCHED, reason, None, None

    # -- one control tick --------------------------------------------------------------
    def step(self, now, cmd, cmd_stamp, mc: McState, localized=True, loc_reason='', foreign_pub='', t1_ok=False,
             yaw_rate=None):
        """cmd = (vx, vy, wz) from Nav2 or None; returns a Decision. Call at publish_hz.
        yaw_rate: fresh filtered measured yaw rate [rad/s] or None (stale) for the pure-rotation loop.
        t1_ok: the trust status reports T1-only (freshness) loss with READY kept (or has not yet classified a new
        loss): a localization-only fault then PAUSES (zero, IDLE) for up to t1_pause_max_s instead of latching."""
        cfg, notes = self.cfg, []
        self._t_step = now
        was_active = self.state == ACTIVE
        # 1. a latch persists, but flushes a zero burst once if we were driving
        if self.state == LATCHED:
            return self._drain(Decision(LATCHED, reason=self.latch_reason))

        # 2. hard faults: latch while driving, refuse to start otherwise
        fault = self._fault(now, mc, localized, loc_reason, foreign_pub)
        have_cmd = (cmd is not None and (now - cmd_stamp) <= cfg.cmd_timeout_s
                    and cmd_stamp > self.resume_after)
        vx, vy, wz = cmd if cmd is not None else (0.0, 0.0, 0.0)
        if cmd is not None and not all(math.isfinite(v) for v in cmd):
            fault, have_cmd = fault or 'non_finite_command', False
        wants_motion = have_cmd and any(abs(v) >= cfg.eps for v in (vx, vy, wz))
        if not fault:
            self.pause_since = None
        elif (t1_ok and fault != 'non_finite_command' and not self._fault(now, mc, True, '', foreign_pub)
              and (wants_motion or was_active or self.pause_since is not None)):
            self.pause_since = now if self.pause_since is None else self.pause_since
            if now - self.pause_since <= cfg.t1_pause_max_s:
                if was_active:
                    self.state, self.active_since, self.burst_left = IDLE, None, cfg.zero_burst
                return self._drain(Decision(IDLE, reason='t1_pause:' + fault))
        if fault and (wants_motion or was_active or self.pause_since is not None):
            self.counters['rejected'] += 1
            self._latch(fault)
            return self._drain(Decision(LATCHED, reason=fault))
        if cmd is not None and abs(vy) >= cfg.eps and not cfg.allow_lateral:
            self.counters['rejected'] += 1
            self._latch('lateral_not_allowed')
            return self._drain(Decision(LATCHED, reason='lateral_not_allowed'))

        # 3. normal end of a goal: stale or zero command
        if not wants_motion:
            if was_active:
                self.state, self.active_since, self.burst_left = IDLE, None, cfg.zero_burst
            return self._drain(Decision(IDLE, reason='no_command'))

        # 4. motion: saturate to the live MC bounds instead of rescaling or lifting
        out = []
        for v, (lo, hi), name in ((vx, mc.fwd, 'fwd'), (vy, mc.lat, 'lat'), (wz, mc.ang, 'ang')):
            v = 0.0 if abs(v) < cfg.eps else v
            if v > hi or v < lo:
                self.counters['saturated'] += 1
                notes.append(f'{name}_saturated')
                v = min(max(v, lo), hi)
            out.append(v)
        cmdout = self._yaw_loop(now, tuple(out), yaw_rate, mc, notes)

        if not cfg.live:
            # 5'. dry-run shadow: no registration exists, so acquisition cannot be observed.
            if not was_active:
                self.state, self.active_since, self.shadow_source = ACTIVE, now, mc.source
            elif mc.source not in (self.shadow_source, cfg.source_name):
                self._latch('operator_takeover')
                return self._drain(Decision(LATCHED, reason=self.latch_reason))
            self.counters['published'] += 1
            return Decision(ACTIVE, publish=cmdout, reason='dry_run', notes=notes)

        # 5. control acquisition and takeover detection (state is 10 Hz)
        if not was_active:
            self.state, self.active_since = ACTIVE, now
        elif mc.source != cfg.source_name:
            if self._ever_ours or now - self.active_since >= cfg.acquire_grace_s:
                # never acquired -> operator/higher priority owns control; acquired then lost -> takeover
                self._latch('operator_takeover' if self._ever_ours else 'control_not_acquired')
                return self._drain(Decision(LATCHED, reason=self.latch_reason))
        if mc.source == cfg.source_name:
            self._ever_ours = True
        self.counters['published'] += 1
        return Decision(ACTIVE, publish=cmdout, reason='driving', notes=notes)

    _ever_ours = False
    _t_step = None

    def _fault(self, now, mc, localized, loc_reason='', foreign_pub=''):
        cfg = self.cfg
        if now - mc.stamp > cfg.state_stale_s:
            return 'mc_state_stale'
        if mc.body != BODY_STAND:
            return 'robot_not_standing'
        if mc.fsm not in (FSM_STABLE, FSM_MOVING):
            return 'mc_fsm_not_stable'
        if cfg.require_localization and not localized:
            return loc_reason or 'localization_lost'
        if foreign_pub:
            return 'foreign_velocity_publisher'      # another node publishes on the MC velocity topic (e.g. ContinuousWalker)
        if mc.source == 'pnc':
            return 'vendor_autonomy_active'          # vendor navigation owns the robot; Nav2 must not compete
        foreign = mc.source and mc.source != cfg.source_name and mc.source not in BUILTIN_SOURCES
        if foreign:
            return 'foreign_sdk_source_active'
        return ''

    def _yaw_loop(self, now, cmdout, yaw_rate, mc, notes):
        """Pure in-place rotation only (|vx| < yaw_lin_eps, vy = 0, |wz| > yaw_w_min, fresh measurement): replace wz
        by the closed-loop command. Otherwise pass-through and reset (no lingering integral)."""
        cfg = self.cfg
        vx, vy, wz = cmdout
        if not cfg.yaw_loop:
            self.yaw.reset()
            return cmdout
        if not (abs(vx) < cfg.yaw_lin_eps and vy == 0.0 and abs(wz) > cfg.yaw_w_min):
            # P23: pause, not reset. Robot still turning under a vx ramp -> no new dead time on resume.
            self.yaw.pause(now, stopped=abs(wz) <= cfg.yaw_w_min)
            return cmdout
        if yaw_rate is None or not math.isfinite(yaw_rate):
            self.counters['yaw_meas_stale'] += 1
            notes.append('yaw_meas_stale')
            self.yaw.pause(now, stopped=True)
            return cmdout
        u = min(max(self.yaw.update(now, wz, yaw_rate), mc.ang[0]), mc.ang[1])
        self.counters['yaw_loop'] += 1
        notes.append('yaw_loop')
        return (vx, vy, u)

    def _drain(self, d):
        """Attach one zero message while a burst is pending (live: MC topic; dry run: ~/dry_run_cmd)."""
        # every non-driving tick hands the output back: zero in -> zero out. P23: I is kept (frozen) for yaw_hold_s
        # across short stops (trust-gate pauses, command gaps); a latch clears it.
        if self.state == LATCHED or self._t_step is None:
            self.yaw.reset()
        else:
            self.yaw.pause(self._t_step, stopped=True)
        if self.burst_left > 0:
            self.burst_left -= 1
            d.publish = (0.0, 0.0, 0.0)
        if self.state != ACTIVE:
            self._ever_ours = False
            self.shadow_source = None
        return d


REG_ADD, REG_ENABLE = 'ADD', 'ENABLE'
#: Only accepted value of the registration_action parameter (the former ADD / ENABLE values are refused so a stale
#: config fails closed instead of being silently ignored).
REGISTRATION_ADD_THEN_ENABLE = 'add_then_enable'


class RegistrationSequence:
    """The one SetMcInputSource registration sequence of a process (no ROS imports).

    AimDK X2 1.1.0 5.1.3 (input sources are reset by a system restart; header.code 0 = success, anything else =
    failure, no distinct "already exists" code and no query of registered sources) and the official example
    py_examples/mc_locomotion_velocity.py:89-97 (src/mc/mc_locomotion_velocity.cpp:61-76): ADD first; if ADD fails
    ("likely already registered") exactly one ENABLE. ADD code 0 -> registered (cold start after reboot, ENABLE never
    sent); ADD code != 0 -> one ENABLE; ENABLE code 0 -> registered; anything else -> failed, no further call. A
    transport exception fails the sequence immediately (it is not an SDK answer). No retry, never DELETE/MODIFY."""

    def __init__(self):
        self.state = 'not_requested'       # -> pending -> registered | failed
        self.action = None                 # request in flight / last sent
        self.error = ''
        self.add = None                    # (code, task_id, state.value) of the ADD response

    @property
    def registered(self):
        """The only condition under which an MC publisher may exist."""
        return self.state == 'registered'

    def start(self):
        """Returns REG_ADD exactly once per process, None afterwards."""
        if self.state != 'not_requested':
            return None
        self.state, self.action = 'pending', REG_ADD
        return REG_ADD

    def on_response(self, code, task_id=0, state_value=0):
        """SDK answer to the request in flight. Returns (next_action or None, log level, message)."""
        if self.state != 'pending':
            raise RuntimeError('registration response without a pending request')
        resp = f'code={code} task_id={task_id} state={state_value}'
        if self.action == REG_ADD:
            self.add = (code, task_id, state_value)
            if code == 0:
                self.state = 'registered'
                return None, 'info', f'SDK source registered by ADD ({resp}); ENABLE not sent'
            self.action = REG_ENABLE
            return REG_ENABLE, 'warn', (
                f'ADD failed ({resp}). Following the AgiBot example mc_locomotion_velocity.py (ADD failed -> "likely '
                f'already registered"): sending exactly one ENABLE; the SDK code does not distinguish this case')
        add = 'ADD code={} task_id={} state={}'.format(*self.add)
        if code == 0:
            self.state = 'registered'
            return None, 'info', (f'SDK source registered by ENABLE ({resp}) after {add}; ENABLE does not re-assert '
                                  f'priority/timeout of an existing source')
        self.state = 'failed'
        self.error = f'{add}; ENABLE {resp}'
        return None, 'error', (f'SDK registration failed: {self.error}. No further call; Nav2 motion forbidden '
                               f'(no MC publisher). Action: check the MC input sources, restart the adapter')

    def on_exception(self, exc):
        """The request in flight raised (service vanished, transport error). Returns the same tuple as on_response."""
        if self.state != 'pending':
            raise RuntimeError('registration exception without a pending request')
        add = '' if self.add is None else ' after ADD code={} task_id={} state={}'.format(*self.add)
        self.state = 'failed'
        self.error = f'{self.action} raised{add}: {exc}'
        return None, 'error', (f'SDK registration failed: {self.error}. No further call; Nav2 motion forbidden '
                               f'(no MC publisher). Action: check the SDK service, restart the adapter')


def driver_input(live, registration_state, motion_armed, cmd, cmd_stamp):
    """Command the core may see this tick: (cmd, cmd_stamp, blocked_reason). Live needs a registered source; both
    modes need motion_armed (set only by ~/arm_motion for a NEW goal). Blocked -> (None, -inf, reason)."""
    if live and registration_state != 'registered':
        return None, float('-inf'), 'registration_' + registration_state
    if not motion_armed:
        return None, float('-inf'), ''
    return cmd, cmd_stamp, ''


class TakeoverMonitor:
    """Evidence of a manual (or any other source's) intervention while a goal owns the driver (no ROS imports).

    AimDK X2 1.1.0 5.1.3 arbitration: another source takes control only by sending a NON-ZERO command; zero commands
    do not reset the timeout; after a timeout "command data is cleared (robot stops), but the input source remains
    unchanged". So McCommonState.input_source == 'rc' is NOT evidence (it stays 'rc' after the stick is released and
    is the idle source after boot). Evidence used instead, each sufficient on its own:
      * command: a non-zero McLocomotionVelocity on /aima/mc/locomotion/velocity whose source is not ours. 5.1.2: the
        remote controller (and other native modules) command on this topic; live 2026-11-06 soc0_rc streams
        source='rc' zeros at ~45 Hz while the stick is untouched. Non-zero is exactly MC's "valid command".
      * edge: the MC input source leaving OUR source after it was observed (only another source's non-zero command
        can cause it, 5.1.3), independent of the command stream.
    Armed = a goal owns the driver (~/arm_motion .. ~/disarm_motion / latch). Both checks run whatever the adapter
    state is, so a takeover while IDLE inside a busy goal (T1 pause, Nav2 command gap) is caught (T6 2026-11-06)."""

    def __init__(self, source_name):
        self.source_name = source_name
        self.ours_seen = False             # MC reported our source since arm()
        self.evidence = ''                 # last takeover evidence (diagnostics)

    def arm(self, mc_source):
        self.ours_seen, self.evidence = mc_source == self.source_name, ''

    def on_mc_source(self, armed, source):
        """McCommonState.input_source.name; returns evidence text or ''."""
        if source == self.source_name:
            self.ours_seen = True
            return ''
        was_ours, self.ours_seen = self.ours_seen, False
        if armed and was_ours:
            return self._found(f'MC input source {self.source_name} -> {source or "<empty>"}')
        return ''

    def on_command(self, armed, source, values):
        """One McLocomotionVelocity (source, (forward, lateral, angular)); returns evidence text or ''."""
        if not armed or source == self.source_name or all(v == 0.0 for v in values):
            return ''
        return self._found('non-zero MC command from source {!r} ({})'.format(
            source, ', '.join(f'{v:+.3f}' for v in values)))

    def _found(self, text):
        self.evidence = text
        return text


def validate_source_name(name):
    """Never operate on built-in sources; AimDK asks SDK sources to use an sdk_ prefix."""
    if name in BUILTIN_SOURCES or not name.startswith('sdk_'):
        raise ValueError(f'input source {name!r} must start with sdk_ and not be a built-in source')
    return name


class OwnerLock:
    """Cooperative exclusive lock so Nav2 and ContinuousWalker cannot both drive."""

    def __init__(self, path):
        self.path, self.fd = os.path.expanduser(path), None

    def acquire(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            holder = os.pread(self.fd, 200, 0).decode(errors='replace').strip()
            os.close(self.fd)
            self.fd = None
            raise RuntimeError(f'motion owner lock held by another process: {holder or "unknown"}')
        os.ftruncate(self.fd, 0)
        os.pwrite(self.fd, f'pid={os.getpid()} since={time.strftime("%F %T")}'.encode(), 0)

    def release(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


# ============================== ROS node ==============================================
def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    from aimdk_msgs.msg import McCommonState, McInputAction, McLocomotionVelocity, MessageHeader
    from aimdk_msgs.srv import SetMcInputSource

    class Node_(Node):
        def __init__(self):
            super().__init__('motion_adapter')
            p = self.declare_parameter
            self.cfg = Config(
                source_name=validate_source_name(p('source_name', Config.source_name).value),
                live=bool(p('live', False).value),
                controller_hz=float(p('controller_hz', Config.controller_hz).value),
                publish_hz=float(p('publish_hz', Config.publish_hz).value),
                allow_lateral=bool(p('allow_lateral', False).value),
                tf_max_age_s=float(p('tf_max_age_s', Config.tf_max_age_s).value),
                require_localization=bool(p('require_localization', True).value),
                require_trust=bool(p('require_trust', True).value),
                t1_pause_max_s=float(p('t1_pause_max_s', Config.t1_pause_max_s).value),
                yaw_loop=bool(p('yaw_loop', Config.yaw_loop).value),
                yaw_u_min_ccw=float(p('yaw_u_min_ccw', Config.yaw_u_min_ccw).value),
                yaw_u_min_cw=float(p('yaw_u_min_cw', Config.yaw_u_min_cw).value),
                yaw_u_max=float(p('yaw_u_max', Config.yaw_u_max).value),
                yaw_ki=float(p('yaw_ki', Config.yaw_ki).value))
            self.priority = int(p('priority', 30).value)       # AimDK range for SDK sources: 20-39
            if not 20 <= self.priority <= 39:
                raise ValueError('SDK source priority must be in the documented range 20..39')
            cmd_topic = p('cmd_vel_topic', '/qazbot_d1_map/cmd_vel_authorized').value   # never the vendor /cmd_vel
            tf_topic = p('tf_topic', '/qazbot_d1_map/tf').value
            lock_path = p('lock_path', '~/control_backend/run/x2_motion_owner.lock').value
            self.core = AdapterCore(self.cfg)
            self.cmd, self.cmd_stamp, self.mc, self.tf_seen = None, float('-inf'), McState(), float('-inf')
            self.lock = OwnerLock(lock_path)
            self.pub = None
            self.trust, self.trust_stamp, self.foreign = False, float('-inf'), []
            self.reg = RegistrationSequence()        # live only; one sequence per process
            self.motion_armed = False
            self.foreign = ['graph_unobserved']
            registration_action = p('registration_action', REGISTRATION_ADD_THEN_ENABLE).value
            if registration_action != REGISTRATION_ADD_THEN_ENABLE:   # stale ADD / ENABLE configs fail closed
                raise ValueError(f'registration_action must be {REGISTRATION_ADD_THEN_ENABLE!r} (got '
                                 f'{registration_action!r}); single ADD / ENABLE modes were removed')
            self.vendor_pub_re = [re.compile(x) for x in p('vendor_publishers', list(VENDOR_PUBLISHERS)).value]
            be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
            self.create_subscription(McCommonState, '/aima/mc/common/state', self._on_state,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                durability=DurabilityPolicy.VOLATILE))
            # Read-only: manual-takeover evidence (TakeoverMonitor). VOLATILE: no history of vendor publishers.
            self.takeover = TakeoverMonitor(self.cfg.source_name)
            self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self._on_mc_command,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                durability=DurabilityPolicy.VOLATILE))
            self.create_subscription(TFMessage, tf_topic, self._on_tf, QoSProfile(depth=50))
            self.create_subscription(Bool, p('trust_topic', '/qazbot_d1_map/localization_trust/trusted').value,
                                     self._on_trust, 10)
            self.trust_status, self.trust_status_stamp = {}, float('-inf')
            self.create_subscription(String, p('trust_status_topic', '/qazbot_d1_map/localization_trust/status').value,
                                     self._on_trust_status, 10)
            self.yaw_est, self.yaw_log_t = YawRateEstimator(), float('-inf')
            self.create_subscription(Odometry, p('odom_topic', '/qazbot_native/odom').value, self._on_odom,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(1.0, self._scan_publishers)
            self.status = self.create_publisher(String, '~/status', 10)
            self.dry = self.create_publisher(Twist, '~/dry_run_cmd', 10)
            self.create_service(Trigger, '~/stop', lambda q, r: self._svc(r, self.core.stop, 'stopped'))
            self.create_service(Trigger, '~/resume', self._resume)
            self.create_service(Trigger, '~/arm_motion', self._arm_motion)
            self.create_service(Trigger, '~/disarm_motion', self._disarm_motion)
            if self.cfg.live:
                self.lock.acquire()
                self.registration_client = self.create_client(
                    SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
                self.create_service(Trigger, '~/register', self._request_registration)
                # Automatic, once: as soon as the SDK service is discovered (event check, no sleep, no deadline).
                self.registration_timer = self.create_timer(0.5, self._auto_register)
            self.mc_qos = be
            self.create_timer(1.0 / self.cfg.publish_hz, self._tick)
            self.create_timer(0.5, self._report)
            self.get_logger().info(f'x2_motion_adapter started: live={self.cfg.live} cmd_topic={cmd_topic} '
                                   f'source={self.cfg.source_name} priority={self.priority} yaw_loop={self.cfg.yaw_loop} '
                                   f'u_min ccw/cw={self.cfg.yaw_u_min_ccw}/{self.cfg.yaw_u_min_cw} '
                                   f'u_max={self.cfg.yaw_u_max} ki={self.cfg.yaw_ki}')

        # -- inputs
        def _now(self):
            return time.monotonic()

        def _on_cmd(self, m):
            self.cmd, self.cmd_stamp = (m.linear.x, m.linear.y, m.angular.z), self._now()

        def _on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.yaw_est.update(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, yaw, self._now())

        def _on_trust(self, m):
            self.trust, self.trust_stamp = bool(m.data), self._now()

        def _on_trust_status(self, m):
            try:
                self.trust_status, self.trust_status_stamp = json.loads(m.data), self._now()
            except ValueError:
                self.trust_status = {}

        def _t1_ok(self, now):
            # Fresh status: T1-only loss with READY kept, or (Bool/status ordering) not yet classified as a fault.
            s = self.trust_status
            return now - self.trust_status_stamp <= self.cfg.trust_max_age_s and bool(s.get('t1_transient') or s.get('trusted'))

        def _scan_publishers(self):
            """Graph-based ownership check: works even if the other process never heard of our flock."""
            try:
                infos = self.get_publishers_info_by_topic('/aima/mc/locomotion/velocity')
            except Exception as e:                      # a graph query must never take the node down
                self.foreign=['graph_query_unavailable']
                self.get_logger().error(f'publisher scan unavailable; motion refused: {e}')
                return
            own = self.get_name()
            # Unknown identity is unavailable evidence, never a timed exemption.
            foreign = {i.node_name for i in infos if i.node_name != own
                       and not any(r.fullmatch(i.node_name) for r in self.vendor_pub_re)}
            self.foreign = sorted(foreign)

        def _on_tf(self, m):
            if any((t.header.frame_id, t.child_frame_id) == ('map', 'odom') for t in m.transforms):
                self.tf_seen = self._now()

        def _on_state(self, m):
            s = self.mc
            s.stamp, s.source = self._now(), m.input_source.name
            s.body, s.fsm = m.body_status.value, m.fsm_state.current_state
            sp = m.speed_status
            if sp.speed_mode.value:
                s.fwd = (sp.forward_bounds.min_value, sp.forward_bounds.max_value)
                s.lat = (sp.lateral_bounds.min_value, sp.lateral_bounds.max_value)
                s.ang = (sp.angular_bounds.min_value, sp.angular_bounds.max_value)
            self._takeover(self.takeover.on_mc_source(self.motion_armed, s.source))

        def _on_mc_command(self, m):
            self._takeover(self.takeover.on_command(
                self.motion_armed, m.source, (m.forward_velocity, m.lateral_velocity, m.angular_velocity)))

        def _takeover(self, evidence):
            # Latch in IDLE as well as ACTIVE; only ~/resume clears it and only a NEW goal (~/arm_motion) re-arms.
            if not evidence:
                return
            self.motion_armed = False
            self.cmd, self.cmd_stamp = None, float('-inf')
            self.core.stop('operator_takeover')
            self.get_logger().warn(f'operator takeover while a goal owns the driver: {evidence}. Latched; the goal '
                                   f'does not resume. Needs ~/resume and a NEW goal')

        # -- services
        def _svc(self, resp, fn, text):
            self.motion_armed = False
            fn()
            resp.success, resp.message = True, text
            return resp

        def _resume(self, req, resp):
            resp.success = self.core.resume()
            if resp.success:
                self.motion_armed = False
            resp.message = 'resumed' if resp.success else 'not latched'
            return resp

        def _health_fault(self, now):
            tf_ok = now - self.tf_seen <= self.cfg.tf_max_age_s
            trust_ok = now - self.trust_stamp <= self.cfg.trust_max_age_s and self.trust
            localized = tf_ok and (trust_ok or not self.cfg.require_trust)
            reason = '' if not tf_ok else 'localization_untrusted'
            return self.core._fault(now, self.mc, localized, reason, ','.join(self.foreign))

        def _driver_path_ok(self):
            # Live: only after a successful registration sequence. Dry run: the explicit
            # dry-run driver path (no MC publisher, no lock, no registration ever exists).
            return self.registration_state == 'registered' if self.cfg.live else self.registration_state == 'dry_run'

        def _arm_motion(self, req, resp):
            # This service is called only for a NEW, validated public navigation goal.
            fault = self._health_fault(self._now())
            resp.success = bool(self._driver_path_ok()
                                and self.core.state == IDLE and self.core.burst_left == 0 and not fault)
            if resp.success:
                self.motion_armed = True
                self.cmd, self.cmd_stamp = None, float('-inf')
                self.core.resume_after = self._now()
                self.takeover.arm(self.mc.source)
                resp.message = 'armed for fresh authorized commands'
            else:
                resp.message = fault or ('requires registered (live) or dry-run, idle, unlatched adapter')
            return resp

        def _disarm_motion(self, req, resp):
            # Called by the coordinator when its goal is terminal (or rejected after arming): the goal no longer
            # owns the driver, so operator RC use between goals does not latch.
            self.motion_armed = False
            self.cmd, self.cmd_stamp = None, float('-inf')
            resp.success, resp.message = True, 'disarmed'
            return resp

        # -- one registration sequence per process (RegistrationSequence: ADD, on failure one ENABLE; no retry)
        @property
        def registration_state(self):
            return self.reg.state if self.cfg.live else 'dry_run'

        @property
        def registration_error(self):
            return self.reg.error

        def _registration_blocker(self):
            if self.reg.state != 'not_requested':
                return 'registration sequence already ran: ' + self.reg.state
            if not self.registration_client.service_is_ready():
                return 'SDK registration service is not discovered; no request sent'
            if self.core.state == LATCHED:
                return 'adapter is latched; no request sent'
            return ''

        def _auto_register(self):
            if self.reg.state != 'not_requested':
                self.registration_timer.cancel()
            elif not self._registration_blocker():
                self.registration_timer.cancel()
                self._start_registration('automatic at start')

        def _request_registration(self, req, resp):
            # Kept for compatibility; normally the automatic sequence has already run and this is refused.
            blocker = self._registration_blocker()
            resp.success = not blocker
            if blocker:
                resp.message = blocker
                return resp
            self._start_registration('~/register')
            resp.message = 'registration sequence started; check status.registration for result'
            return resp

        def _start_registration(self, why):
            self.get_logger().info(f'SDK source registration ({why}): {self.cfg.source_name} priority={self.priority}')
            self._send_registration(self.reg.start())

        def _send_registration(self, action):
            request = SetMcInputSource.Request()
            request.action.value = (McInputAction.INPUTACTION_ADD if action == REG_ADD
                                    else McInputAction.INPUTACTION_ENABLE)
            request.input_source.name = self.cfg.source_name
            request.input_source.priority = self.priority
            request.input_source.timeout = 1000  # SDK input-source lease, official example
            request.request.header.stamp = self.get_clock().now().to_msg()
            self.registration_client.call_async(request).add_done_callback(self._registration_result)

        def _registration_result(self, future):
            if not self.cfg.live:                       # unreachable by construction; never create an MC publisher in dry run
                raise RuntimeError('registration result in dry run')
            try:
                r = future.result().response
                nxt, level, text = self.reg.on_response(r.header.code, r.task_id, r.state.value)
            except Exception as exc:
                nxt, level, text = self.reg.on_exception(exc)
            # One call site per severity: rclpy refuses a severity change at the same call site ("Logger severity
            # cannot be changed between calls", G2b 2026-11-06: ADD warn -> ENABLE info crashed the node).
            if level == 'error':
                self.get_logger().error(text)
            elif level == 'warn':
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
            if nxt is not None:
                self._send_registration(nxt)
                return
            if not self.reg.registered:
                return
            self.motion_armed = False
            # Discard commands queued before registration completed.
            self.cmd, self.cmd_stamp = None, float('-inf')
            self.core.resume_after = self._now()
            self.pub = self.create_publisher(
                McLocomotionVelocity, '/aima/mc/locomotion/velocity', self.mc_qos)
            self.get_logger().info('SDK source registered; waiting for fresh authorized command')

        # -- control tick
        def _tick(self):
            now = self._now()
            cmd, stamp, blocked = driver_input(self.cfg.live, self.registration_state, self.motion_armed,
                                               self.cmd, self.cmd_stamp)
            self.cmd, self.cmd_stamp = cmd, stamp            # never queue a command while blocked / not armed
            if blocked:
                self._last = Decision(self.core.state, reason=blocked)
                return
            tf_ok = (now - self.tf_seen) <= self.cfg.tf_max_age_s
            trust_ok = (now - self.trust_stamp) <= self.cfg.trust_max_age_s and self.trust
            localized = tf_ok and (trust_ok or not self.cfg.require_trust)
            loc_reason = '' if not tf_ok else 'localization_untrusted'
            d = self.core.step(now, self.cmd, self.cmd_stamp, self.mc, localized, loc_reason, ','.join(self.foreign),
                               self._t1_ok(now), self.yaw_est.get(now, self.cfg.yaw_meas_max_age_s))
            y = self.core.yaw
            if y.active and 'yaw_loop' in d.notes and now - self.yaw_log_t >= 1.0:
                self.yaw_log_t = now
                self.get_logger().info(f'yaw_loop target={y.target:+.2f} meas={y.meas:+.2f} u={y.sign * y.u:+.2f} '
                                       f'I={y.integ:+.2f}')
            if d.state == LATCHED:
                self.motion_armed = False
            elif not self.motion_armed:
                d.reason = 'driver_not_armed'
            if d.publish is not None:
                if self.cfg.live and self.pub is not None:
                    m = McLocomotionVelocity()
                    m.header = MessageHeader()
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.source = self.cfg.source_name
                    m.forward_velocity, m.lateral_velocity, m.angular_velocity = d.publish
                    self.pub.publish(m)
                else:
                    t = Twist()
                    t.linear.x, t.linear.y, t.angular.z = d.publish
                    self.dry.publish(t)
            self._last = d

        def _report(self):
            d = getattr(self, '_last', None)
            self.status.publish(String(data=json.dumps({
                'state': self.core.state, 'reason': getattr(d, 'reason', ''), 'latch': self.core.latch_reason,
                'live': self.cfg.live, 'mode': 'live' if self.cfg.live else DRY_RUN, 'dry_run': not self.cfg.live,
                'mc_source': self.mc.source, 'trusted': self.trust,
                'registration': self.registration_state, 'registration_error': self.registration_error,
                'sdk_service_ready': bool(self.cfg.live and self.registration_client.service_is_ready()),
                'motion_armed': self.motion_armed, 'takeover_evidence': self.takeover.evidence,
                't1_pause': bool(self.core.pause_since is not None or (self._health_fault(self._now()) and self._t1_ok(self._now())
                                 and not self.core._fault(self._now(), self.mc, True, '', ','.join(self.foreign)))),
                'ready': bool(self._driver_path_ok()
                              and self.core.state in (IDLE, ACTIVE) and not self._health_fault(self._now())),
                'foreign_publishers': self.foreign, 'counters': self.core.counters})))

        def destroy_node(self):
            if self.cfg.live and self.pub is not None and (self.core.state == ACTIVE or self.core.burst_left > 0):
                z = McLocomotionVelocity()                      # last word: stop, then release
                z.header = MessageHeader()
                z.header.stamp = self.get_clock().now().to_msg()
                z.source = self.cfg.source_name
                for _ in range(self.cfg.zero_burst):
                    self.pub.publish(z)
            self.lock.release()
            super().destroy_node()

    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    shutting_down=[False]
    def terminate(signum,frame):
        shutting_down[0]=True
    signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
    node = Node_()
    try:
        while not shutting_down[0] and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=1.0 / node.cfg.publish_hz)
    except KeyboardInterrupt:
        pass
    finally:
        shutting_down[0]=True
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
