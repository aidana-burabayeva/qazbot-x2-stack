"""Pure logic of the D-05 Nav2-goal calibration runner (no ROS imports, no clocks, no sleeps).

The runner never sends goals and never publishes velocity. It only lowers the RPP desired speed for one run through
the standard Nav2 interface (nav2_msgs/SpeedLimit on <ns>/nav2/speed_limit, percentage mode), records, waits for the
operator's RViz goal (observed via goal_coordinator status) and summarises commanded speed / ramps afterwards.

Humble 1.1.18 facts this relies on (verified on the installed binaries, see report 20261103-d05-nav2-calib.md):
* controller_server param speed_limit_topic (default "speed_limit") -> RPP setSpeedLimit(limit, percentage).
* percentage=True: desired = base_desired_linear_vel * pct / 100; speed_limit 0.0 == NO_SPEED_LIMIT (restore base).
* The limit is controller state: it persists across goals until NO_SPEED_LIMIT is published.
"""
import math
from dataclasses import dataclass, field

NO_SPEED_LIMIT = 0.0        # nav2_costmap_2d::NO_SPEED_LIMIT
MIN_TARGET_VX = 0.05        # m/s; below this the controller output is too close to the adapter's 0.02 zero threshold
STATUS_MAX_AGE_S = 1.0      # GC status 5 Hz, adapter status 2 Hz
STEADY_TOL = 0.005          # m/s band around the target that counts as "at the limit"
HONOR_TOL = 0.002           # m/s above the target that already counts as "limit not honoured"


class CalibError(ValueError):
    """Request is outside the configured Nav2 envelope or otherwise invalid."""


@dataclass(frozen=True)
class LiveParams:
    """Values read from the running controller_server / velocity_smoother (never invented)."""
    desired_linear_vel: float           # FollowPath.desired_linear_vel (RPP base)
    smoother_max_vx: float              # velocity_smoother max_velocity[0]
    smoother_accel_x: float             # max_accel[0]  (> 0)
    smoother_decel_x: float             # max_decel[0]  (< 0)
    approach_dist: float                # FollowPath.approach_velocity_scaling_dist
    min_approach_vel: float             # FollowPath.min_approach_linear_velocity
    xy_goal_tolerance: float            # goal_checker.xy_goal_tolerance


@dataclass(frozen=True)
class SpeedLimitPlan:
    percentage: float                   # value sent in SpeedLimit.speed_limit (percentage=True)
    expected_cmd_vx: float              # RPP output on a straight segment outside the approach zone


def _finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def validate_live(p: LiveParams):
    if not _finite(p.desired_linear_vel, p.smoother_max_vx, p.smoother_accel_x, p.smoother_decel_x,
                   p.approach_dist, p.min_approach_vel, p.xy_goal_tolerance):
        raise CalibError('live parameters not finite / missing')
    if p.desired_linear_vel <= 0 or p.smoother_max_vx <= 0 or p.smoother_accel_x <= 0 or p.smoother_decel_x >= 0:
        raise CalibError('live parameters implausible (desired/max_velocity/accel must be > 0, decel < 0)')


def envelope(p: LiveParams) -> float:
    """Highest command the configured stack can produce; the runner never raises it."""
    return min(p.desired_linear_vel, p.smoother_max_vx)


def plan_speed_limit(target_vx: float, p: LiveParams) -> SpeedLimitPlan:
    """Percentage limit that makes RPP command target_vx; refuses anything above the live envelope."""
    validate_live(p)
    if not _finite(target_vx):
        raise CalibError('target not finite')
    if target_vx < MIN_TARGET_VX:
        raise CalibError(f'target {target_vx} < {MIN_TARGET_VX} m/s')
    if target_vx > envelope(p) + 1e-9:
        raise CalibError(f'target {target_vx} > live envelope {envelope(p)} m/s (min of RPP desired and smoother '
                         'max_velocity); raising the envelope is a separate decision, not done by this tool')
    pct = 100.0 * target_vx / p.desired_linear_vel
    pct = min(pct, 100.0)
    return SpeedLimitPlan(percentage=pct, expected_cmd_vx=p.desired_linear_vel * pct / 100.0)


def approach_affects(target_vx: float, p: LiveParams) -> bool:
    """RPP approach scaling lowers the speed only if target*(d/approach_dist) can fall below target before
    the min_approach floor takes over, i.e. only when target > min_approach_linear_velocity."""
    return target_vx > p.min_approach_vel


def segment_estimate(target_vx: float, path_len_m: float, p: LiveParams, gain: float = 1.0) -> dict:
    """Expected command profile on a straight path. gain = real/commanded speed (D1: ~1.61 at 0.30);
    distances are along the real path (localization pose drives RPP), times use target*gain."""
    if not _finite(target_vx, path_len_m, gain) or target_vx <= 0 or path_len_m <= 0 or gain <= 0:
        raise CalibError('invalid estimate inputs')
    t_up = target_vx / p.smoother_accel_x
    t_down = target_vx / -p.smoother_decel_x
    v_real = target_vx * gain
    d_up = 0.5 * v_real * t_up
    d_tail = p.approach_dist if approach_affects(target_vx, p) else p.xy_goal_tolerance
    d_steady = path_len_m - d_up - d_tail
    return {'ramp_up_s': t_up, 'ramp_down_s': t_down, 'ramp_up_m': d_up, 'tail_m': d_tail,
            'steady_m': d_steady, 'steady_s': max(d_steady, 0.0) / v_real}


def min_path_for_steady(target_vx: float, steady_s: float, p: LiveParams, gain: float) -> float:
    """Shortest straight goal distance that still leaves steady_s of constant command."""
    est = segment_estimate(target_vx, 1.0, p, gain)
    return steady_s * target_vx * gain + est['ramp_up_m'] + est['tail_m']


def preflight(gc, gc_age, adapter, adapter_age, limit_subs, limit_foreign_pubs):
    """Read-only start conditions. Returns a list of refusal reasons (empty = OK).
    gc/adapter: decoded status JSON dicts (or None); ages in s; limit_subs: subscribers on the speed limit topic;
    limit_foreign_pubs: other publishers on it (e.g. a speed filter would fight this tool)."""
    why = []
    if gc is None or gc_age is None or gc_age > STATUS_MAX_AGE_S:
        why.append('goal_coordinator status missing/stale')
    else:
        if gc.get('busy'):
            why.append('goal_coordinator busy (a goal is running)')
        if gc.get('latched'):
            why.append(f"goal_coordinator latched ({gc.get('fault')!r})")
        if not gc.get('ready'):
            why.append('goal_coordinator not ready (admission would reject a goal)')
    if adapter is None or adapter_age is None or adapter_age > STATUS_MAX_AGE_S:
        why.append('motion_adapter status missing/stale')
    else:
        if adapter.get('live') is not True or adapter.get('registration') != 'registered':
            why.append('motion_adapter not live+registered')
        if adapter.get('latch'):
            why.append(f"motion_adapter latched ({adapter.get('latch')!r})")
        if adapter.get('foreign_publishers'):
            why.append('foreign MC velocity publisher present')
    if limit_subs < 1:
        why.append('no subscriber on speed_limit topic (controller_server not up / wrong namespace)')
    if limit_foreign_pubs > 0:
        why.append('another publisher on speed_limit topic')
    return why


@dataclass
class GoalTracker:
    """Follows goal_coordinator status: WAIT -> ACTIVE (busy) -> POST (not busy, post-roll) -> DONE."""
    post_roll_s: float
    state: str = 'WAIT'
    t_start: float = None
    t_end: float = None
    fault_end: str = ''
    latched_end: bool = False

    def update(self, now, gc):
        if self.state == 'WAIT' and gc.get('busy'):
            self.state, self.t_start = 'ACTIVE', now
        elif self.state == 'ACTIVE' and not gc.get('busy'):
            self.state, self.t_end = 'POST', now
            self.fault_end, self.latched_end = str(gc.get('fault') or ''), bool(gc.get('latched'))
        if self.state == 'POST' and now - self.t_end >= self.post_roll_s:
            self.state = 'DONE'
        return self.state


def honored(samples, target_vx, tol=HONOR_TOL):
    """samples: [(t, vx)] of controller output during the goal. True if no sample exceeds target+tol."""
    return all(vx <= target_vx + tol for _, vx in samples)


def longest_steady(samples, target_vx, tol=STEADY_TOL):
    """Longest contiguous time span with |vx - target| <= tol; returns (duration_s, t0, t1)."""
    best = (0.0, None, None)
    t0 = None
    prev = None
    for t, vx in samples:
        if abs(vx - target_vx) <= tol:
            if t0 is None:
                t0 = t
            prev = t
            if prev - t0 > best[0]:
                best = (prev - t0, t0, prev)
        else:
            t0 = None
    return best


def ramp_stats(samples):
    """Max positive / most negative dvx/dt between consecutive samples [(t, vx)] (smoother output)."""
    acc, dec = 0.0, 0.0
    for (ta, va), (tb, vb) in zip(samples, samples[1:]):
        dt = tb - ta
        if dt <= 1e-6:
            continue
        a = (vb - va) / dt
        acc, dec = max(acc, a), min(dec, a)
    return {'max_accel': acc, 'max_decel': dec}


def ramp_within(stats, p: LiveParams, margin=1.10):
    """Observed smoother ramps must not exceed the configured limits (10 % margin for timestamp jitter)."""
    return stats['max_accel'] <= p.smoother_accel_x * margin and stats['max_decel'] >= p.smoother_decel_x * margin


@dataclass
class RunSummary:
    target_vx: float
    plan: SpeedLimitPlan
    nav_samples: list = field(default_factory=list)       # controller_server output (cmd_vel_nav)
    smooth_samples: list = field(default_factory=list)    # velocity_smoother output (<ns>/cmd_vel)

    def to_dict(self, p: LiveParams):
        nav_max = max((v for _, v in self.nav_samples), default=None)
        sm_max = max((v for _, v in self.smooth_samples), default=None)
        steady = longest_steady(self.smooth_samples, self.plan.expected_cmd_vx)
        ramps = ramp_stats(self.smooth_samples)
        return {
            'nav_cmd_samples': len(self.nav_samples), 'smoothed_cmd_samples': len(self.smooth_samples),
            'nav_cmd_max_vx': nav_max, 'smoothed_cmd_max_vx': sm_max,
            'limit_honored': bool(self.nav_samples) and honored(self.nav_samples, self.plan.expected_cmd_vx),
            'steady_cmd_s': steady[0], 'steady_cmd_t0': steady[1], 'steady_cmd_t1': steady[2],
            'ramp': ramps, 'ramp_within_limits': bool(self.smooth_samples) and ramp_within(ramps, p),
        }
