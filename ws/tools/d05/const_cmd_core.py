"""Pure logic of the D-05 fixed-command source (no ROS imports, no clocks, no sleeps).

All time is passed in by the caller, so every function is unit-testable without waiting.
Limits mirror the protocol P-D05 (docs/hardware/protocols/HW_PROTOCOLS_20261103.md).
"""
import json
import math
from dataclasses import dataclass

MAX_ABS_VX = 0.60          # m/s, hard cap (protocol: highest run 0.60)
MIN_ABS_VX = 0.02          # m/s, below this the adapter treats the command as (near) zero / pure rotation
MAX_DURATION_S = 12.0      # s, hard cap (protocol: longest run 12 s)
DEFAULT_SETTLE_S = 2.0     # s, part of the duration (protocol: 2 s settle + steady segment)
NAV2_RATE_HZ = 20.0        # Nav2 controller_frequency and velocity_smoother smoothing_frequency (nav_control.launch.py)
MAX_RATE_HZ = 50.0         # adapter publish_hz; no point in feeding faster
ZERO_COUNT = 10            # explicit zero messages at the end (adapter then sends its own 5-zero burst to MC)
STATUS_MAX_AGE_S = 1.0     # adapter status is published at 2 Hz (start condition)
RUN_STATUS_MAX_AGE_S = 1.5  # during the run: status older than this means the adapter is gone -> stop
RC_QUIET_S = 1.0           # a non-zero 'rc' velocity message newer than this means the RC is in use
MIN_SETTLE_MARGIN_S = 1.0  # at least this much steady command after the settle

# adapter status reasons that mean the adapter itself stopped or refused (x2_motion_adapter.py / goal_coordinator hard set)
HARD_REASONS = frozenset({
    'operator_takeover', 'control_not_acquired', 'robot_not_standing', 'mc_fsm_not_stable', 'mc_state_stale',
    'foreign_sdk_source_active', 'foreign_velocity_publisher', 'vendor_autonomy_active', 'localization_untrusted',
    'localization_lost', 'stop_requested', 'non_finite_command', 'lateral_not_allowed', 'driver_not_armed'})


class LimitError(ValueError):
    """Requested parameters violate the hard limits."""


@dataclass(frozen=True)
class CmdParams:
    vx: float
    duration_s: float
    settle_s: float = DEFAULT_SETTLE_S
    rate_hz: float = NAV2_RATE_HZ


def validate_params(vx, duration_s, settle_s=DEFAULT_SETTLE_S, rate_hz=NAV2_RATE_HZ):
    """Return CmdParams or raise LimitError. NaN/inf are rejected (comparisons with NaN are False)."""
    for name, v in (('vx', vx), ('duration', duration_s), ('settle', settle_s), ('rate', rate_hz)):
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            raise LimitError(f'{name} must be a finite number, got {v!r}')
    if abs(vx) > MAX_ABS_VX:
        raise LimitError(f'|vx|={abs(vx):g} exceeds the hard limit {MAX_ABS_VX:g} m/s')
    if abs(vx) < MIN_ABS_VX:
        raise LimitError(f'|vx|={abs(vx):g} is below {MIN_ABS_VX:g} m/s (adapter would treat it as zero)')
    if duration_s <= 0 or duration_s > MAX_DURATION_S:
        raise LimitError(f'duration={duration_s:g} s outside (0, {MAX_DURATION_S:g}] s')
    if settle_s < 0 or settle_s + MIN_SETTLE_MARGIN_S > duration_s:
        raise LimitError(f'settle={settle_s:g} s leaves less than {MIN_SETTLE_MARGIN_S:g} s steady segment '
                         f'in duration={duration_s:g} s')
    if rate_hz < NAV2_RATE_HZ or rate_hz > MAX_RATE_HZ:
        raise LimitError(f'rate={rate_hz:g} Hz outside [{NAV2_RATE_HZ:g}, {MAX_RATE_HZ:g}] Hz '
                         '(adapter command timeout is 3 periods of 20 Hz = 0.15 s)')
    return CmdParams(float(vx), float(duration_s), float(settle_s), float(rate_hz))


class Schedule:
    """Constant-vx command profile over [0, duration): settle phase then steady phase, step profile (no ramp).
    Phase boundaries are relative to the start stamp chosen by the caller."""

    def __init__(self, params: CmdParams):
        self.p = params

    def phase(self, t):
        """t = seconds since start. Returns 'settle', 'steady' or 'done'."""
        if t < 0:
            raise ValueError('t must be >= 0')
        if t >= self.p.duration_s:
            return 'done'
        return 'settle' if t < self.p.settle_s else 'steady'

    def command(self, t):
        """(vx, vy, wz) to publish at t; zero once done."""
        return (self.p.vx, 0.0, 0.0) if self.phase(t) != 'done' else (0.0, 0.0, 0.0)

    def nominal_messages(self):
        """Number of command ticks a perfect clock gives in the duration (tick k at t = k/rate)."""
        return int(math.ceil(self.p.duration_s * self.p.rate_hz - 1e-9))


def parse_status(text):
    """Adapter/GC status String.data -> dict, or None if it is not a JSON object."""
    try:
        d = json.loads(text)
    except (TypeError, ValueError):
        return None
    return d if isinstance(d, dict) else None


def adapter_refusals(status, age_s):
    """Reasons to refuse to START given the latest adapter status (empty list = OK). Fail closed on missing data."""
    if status is None or age_s is None:
        return ['adapter status not received']
    r = []
    if age_s > STATUS_MAX_AGE_S:
        r.append(f'adapter status stale ({age_s:.2f} s > {STATUS_MAX_AGE_S:g} s)')
    if status.get('live') is not True or status.get('mode') != 'live':
        r.append('adapter not live')
    if status.get('registration') != 'registered':
        r.append(f"adapter registration={status.get('registration')!r} (need 'registered')")
    if status.get('motion_armed') is not True:
        r.append('adapter not armed (motion_armed != true; arm_motion not called)')
    if status.get('state') != 'idle':
        r.append(f"adapter state={status.get('state')!r} (need 'idle')")
    if status.get('latch'):
        r.append(f"adapter latched: {status.get('latch')!r}")
    if status.get('ready') is not True:
        r.append('adapter ready != true (health/trust/MC state/foreign publisher)')
    if status.get('foreign_publishers'):
        r.append(f"foreign publishers on MC velocity topic: {status.get('foreign_publishers')}")
    if status.get('mc_source') == 'pnc':
        r.append("MC input source is 'pnc' (vendor autonomy owns the robot)")
    if status.get('t1_pause'):
        r.append('adapter in t1_pause (localization input loss)')
    return r


def gc_refusals(gc_status):
    """Goal Coordinator status (None = GC not running, allowed): refuse if it is busy/latched/paused."""
    if gc_status is None:
        return []
    r = []
    if gc_status.get('busy'):
        r.append('goal coordinator busy (a Nav2 goal is active)')
    if gc_status.get('latched'):
        r.append(f"goal coordinator latched: {gc_status.get('fault')!r}")
    return r


def rc_active(last_nonzero_rc_age_s):
    """True if a non-zero 'rc' velocity message was seen within RC_QUIET_S (None = never seen)."""
    return last_nonzero_rc_age_s is not None and last_nonzero_rc_age_s <= RC_QUIET_S


def preflight_refusals(status, status_age_s, gc_status, other_cmd_publishers, last_nonzero_rc_age_s,
                       cmd_subscribers):
    """All reasons to refuse start. other_cmd_publishers: names of nodes (other than goal_coordinator) already
    publishing on the command topic. cmd_subscribers: number of subscribers on it (adapter must be listening)."""
    r = adapter_refusals(status, status_age_s) + gc_refusals(gc_status)
    if other_cmd_publishers:
        r.append(f'other publishers on the command topic: {sorted(other_cmd_publishers)}')
    if rc_active(last_nonzero_rc_age_s):
        r.append('RC active (non-zero rc velocity within the last %.1f s)' % RC_QUIET_S)
    if cmd_subscribers < 1:
        r.append('no subscriber on the command topic (adapter not listening)')
    return r


def abort_reason(t, status, status_age_s, last_nonzero_rc_age_s, mc_acquire_grace_s=2.0):
    """Reason to stop NOW during the run ('' = continue). t = seconds since start."""
    if status is None or status_age_s is None or status_age_s > RUN_STATUS_MAX_AGE_S:
        return 'adapter status lost'
    if status.get('state') == 'latched' or status.get('latch'):
        return f"adapter latched: {status.get('latch') or status.get('reason')}"
    if status.get('reason') in HARD_REASONS:
        return f"adapter reason: {status.get('reason')}"
    if rc_active(last_nonzero_rc_age_s):
        return 'RC activity'
    if t >= mc_acquire_grace_s and status.get('mc_source') == 'rc':
        return 'MC input source is rc (control not ours)'
    return ''
