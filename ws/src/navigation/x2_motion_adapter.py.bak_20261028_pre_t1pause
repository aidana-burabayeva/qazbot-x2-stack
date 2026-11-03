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
    idle source is already 'rc' (docs/CURRENT_CONSTRAINTS.md), so a PS5 takeover
    is only observable in dry run when MC reports a source change.
  * Live mode needs live:=true. It then takes an exclusive owner lock, registers a
    source named sdk_*, and publishes only while a fresh, valid, non-zero command exists.
  * Values are passed through, never raised to a minimum. Out-of-bounds values are
    saturated to the MC bounds and counted; non-finite or lateral values stop the robot.
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
        self.counters = {'saturated': 0, 'rejected': 0, 'latched': 0, 'published': 0}

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
        self.state, self.latch_reason, self.active_since = LATCHED, reason, None

    # -- one control tick --------------------------------------------------------------
    def step(self, now, cmd, cmd_stamp, mc: McState, localized=True, loc_reason='', foreign_pub=''):
        """cmd = (vx, vy, wz) from Nav2 or None; returns a Decision. Call at publish_hz."""
        cfg, notes = self.cfg, []
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
        if fault and (wants_motion or was_active):
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
        cmdout = tuple(out)

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

    def _drain(self, d):
        """Attach one zero message while a burst is pending (live: MC topic; dry run: ~/dry_run_cmd)."""
        if self.burst_left > 0:
            self.burst_left -= 1
            d.publish = (0.0, 0.0, 0.0)
        if self.state != ACTIVE:
            self._ever_ours = False
            self.shadow_source = None
        return d


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
                require_trust=bool(p('require_trust', True).value))
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
            self.registration_state = 'not_requested' if self.cfg.live else 'dry_run'
            self.registration_error = ''
            self.registration_future = None
            self.motion_armed = False
            self.foreign = ['graph_unobserved']
            self.registration_action = p('registration_action', 'ADD').value
            if self.registration_action not in ('ADD', 'ENABLE'):
                raise ValueError('registration_action must be ADD or explicit ENABLE')
            self.vendor_pub_re = [re.compile(x) for x in p('vendor_publishers', list(VENDOR_PUBLISHERS)).value]
            be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
            self.create_subscription(McCommonState, '/aima/mc/common/state', self._on_state,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                durability=DurabilityPolicy.VOLATILE))
            self.create_subscription(TFMessage, tf_topic, self._on_tf, QoSProfile(depth=50))
            self.create_subscription(Bool, p('trust_topic', '/qazbot_d1_map/localization_trust/trusted').value,
                                     self._on_trust, 10)
            self.create_timer(1.0, self._scan_publishers)
            self.status = self.create_publisher(String, '~/status', 10)
            self.dry = self.create_publisher(Twist, '~/dry_run_cmd', 10)
            self.create_service(Trigger, '~/stop', lambda q, r: self._svc(r, self.core.stop, 'stopped'))
            self.create_service(Trigger, '~/resume', self._resume)
            self.create_service(Trigger, '~/arm_motion', self._arm_motion)
            if self.cfg.live:
                self.lock.acquire()
                self.registration_client = self.create_client(
                    SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
                self.create_service(Trigger, '~/register', self._request_registration)
            self.mc_qos = be
            self.create_timer(1.0 / self.cfg.publish_hz, self._tick)
            self.create_timer(0.5, self._report)
            self.get_logger().info(f'x2_motion_adapter started: live={self.cfg.live} cmd_topic={cmd_topic} '
                                   f'source={self.cfg.source_name} priority={self.priority}')

        # -- inputs
        def _now(self):
            return time.monotonic()

        def _on_cmd(self, m):
            self.cmd, self.cmd_stamp = (m.linear.x, m.linear.y, m.angular.z), self._now()

        def _on_trust(self, m):
            self.trust, self.trust_stamp = bool(m.data), self._now()

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
            # Live: only after an explicit, acknowledged registration. Dry run: the explicit
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
                resp.message = 'armed for fresh authorized commands'
            else:
                resp.message = fault or ('requires registered (live) or dry-run, idle, unlatched adapter')
            return resp

        # -- one explicit registration request per process, no retry or fallback
        def _request_registration(self, req, resp):
            if self.registration_state != 'not_requested':
                resp.success = False
                resp.message = 'registration already requested: ' + self.registration_state
                return resp
            if not self.registration_client.service_is_ready():
                resp.success = False
                resp.message = 'SDK registration service is not discovered; no request sent'
                return resp
            if self.core.state == LATCHED:
                resp.success = False
                resp.message = 'adapter is latched; no request sent'
                return resp
            request = SetMcInputSource.Request()
            request.action.value = (McInputAction.INPUTACTION_ADD if self.registration_action == 'ADD'
                                    else McInputAction.INPUTACTION_ENABLE)
            request.input_source.name = self.cfg.source_name
            request.input_source.priority = self.priority
            request.input_source.timeout = 1000  # SDK input-source lease, official example
            request.request.header.stamp = self.get_clock().now().to_msg()
            self.registration_state = 'pending'
            self.registration_future = self.registration_client.call_async(request)
            self.registration_future.add_done_callback(self._registration_result)
            resp.success = True
            resp.message = 'one request submitted; check status.registration for result'
            return resp

        def _registration_result(self, future):
            if not self.cfg.live:                       # unreachable by construction; never create an MC publisher in dry run
                raise RuntimeError('registration result in dry run')
            try:
                code = future.result().response.header.code
            except Exception as exc:
                self.registration_state = 'failed'
                self.registration_error = str(exc)
                self.get_logger().error('SDK registration failed: ' + str(exc))
                return
            if code != 0:
                self.registration_state = 'failed'
                self.registration_error = 'SDK response code=' + str(code)
                self.get_logger().error(self.registration_error + '; no retry or ENABLE fallback')
                return
            self.registration_state = 'registered'
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
            if self.cfg.live and self.registration_state != 'registered':
                self.cmd, self.cmd_stamp = None, float('-inf')
                self._last = Decision(self.core.state, reason='registration_' + self.registration_state)
                return
            if not self.motion_armed:                        # live and dry-run shadow alike
                self.cmd, self.cmd_stamp = None, float('-inf')
            tf_ok = (now - self.tf_seen) <= self.cfg.tf_max_age_s
            trust_ok = (now - self.trust_stamp) <= self.cfg.trust_max_age_s and self.trust
            localized = tf_ok and (trust_ok or not self.cfg.require_trust)
            loc_reason = '' if not tf_ok else 'localization_untrusted'
            d = self.core.step(now, self.cmd, self.cmd_stamp, self.mc, localized, loc_reason, ','.join(self.foreign))
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
                'motion_armed': self.motion_armed,
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
