#!/usr/bin/env python3
"""P20 live in-place test of the adapter yaw-rate loop (operator-approved 2026-10-29). Path: Twist ->
/qazbot_d1_map/cmd_vel_authorized -> x2_motion_adapter (P20 loop) -> MC. Needs CONFIRM=1.
Steps: CCW 0.5 rad/s 6 s; CW 0.5 6 s; 90 deg CCW; 90 deg CW (P on yaw error |w| = clamp(|e|, 0.1, 0.5) until |e| < 5 deg,
re-command after 1.5 s settle if |e| > 5 deg, max 3 corrections, 20 s cap; final error after 2 s settle). 3 s zero rest between.
Abort: per-step translation > 0.3 m, adapter latch / not registered, MC source not ours/rc, MC action not STAND/LOCOMOTION.
Logs odom chest yaw, commands, the adapter's MC output (u) and status."""
import json, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from aimdk_msgs.msg import McCommonState, McLocomotionVelocity

NS, SRC = '/qazbot_d1_map', os.environ.get('SRC', 'sdk_qazbot_nav2_1029a')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), time.strftime('p20_yaw_test_%Y%m%d_%H%M%S'))
OK_ACTIONS = ('STAND_DEFAULT', 'LOCOMOTION_DEFAULT')
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class T(Node):
    def __init__(self, raw):
        super().__init__('p20_yaw_test')
        self.raw, self.od, self.u, self.ad, self.gc, self.mc = raw, [], [], {}, {}, None
        self.pub = self.create_publisher(Twist, NS + '/cmd_vel_authorized', 10)
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/qazbot_native/odom', self._odom, be)
        self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self._u, be)
        self.create_subscription(McCommonState, '/aima/mc/common/state', self._mc,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(String, NS + '/motion_adapter/status', lambda m: setattr(self, 'ad', json.loads(m.data)), 10)
        self.create_subscription(String, NS + '/goal_coordinator/status', lambda m: setattr(self, 'gc', json.loads(m.data)), 10)
        self.arm = self.create_client(Trigger, NS + '/motion_adapter/arm_motion')

    def log(self, **k):
        k['t'] = time.monotonic(); self.raw.write(json.dumps(k) + '\n')

    def _odom(self, m):
        p = m.pose.pose; s = (time.monotonic(), yawq(p.orientation), p.position.x, p.position.y)
        self.od.append(s); self.log(k='odom', yaw=s[1], x=s[2], y=s[3])

    def _u(self, m):
        if m.source == SRC:
            self.u.append((time.monotonic(), m.angular_velocity, m.forward_velocity)); self.log(k='u', w=m.angular_velocity, v=m.forward_velocity)

    def _mc(self, m):
        self.mc = (m.input_source.name, m.action_info.action_desc, m.fsm_state.current_state, m.body_status.value)
        self.log(k='mc', source=self.mc[0], action=self.mc[1], fsm=self.mc[2], body=self.mc[3])

    def yaw(self):
        return self.od[-1][1]

    def rate(self, a, b):
        ys = [s for s in self.od if a <= s[0] <= b]
        if len(ys) < 3: return None
        return sum(math.remainder(ys[i + 1][1] - ys[i][1], 2 * math.pi) for i in range(len(ys) - 1)) / (ys[-1][0] - ys[0][0])

    def umean(self, a, b):
        us = [x[1] for x in self.u if a <= x[0] <= b]
        return (round(sum(us) / len(us), 3), round(min(us), 3), round(max(us), 3)) if us else None

    def check(self, x0, y0):
        a = self.ad
        if a.get('latch') or a.get('registration') != 'registered': return f'adapter {a.get("state")} latch={a.get("latch")}'
        if self.mc is None or self.mc[0] not in (SRC, 'rc'): return f'mc source {self.mc}'
        if self.mc[1] not in OK_ACTIONS: return f'mc action {self.mc}'
        if math.hypot(self.od[-1][2] - x0, self.od[-1][3] - y0) > float(os.environ.get('MAX_DRIFT','0.3')): return 'translated >MAX_DRIFT (step)'
        return None

    def send(self, w, x0, y0):
        t = Twist(); t.angular.z = float(w); self.pub.publish(t); self.log(k='cmd', w=w)
        rclpy.spin_once(self, timeout_sec=0.05)
        return self.check(x0, y0)

    def hold(self, s, w, x0, y0):
        end = time.monotonic() + s
        while time.monotonic() < end:
            why = self.send(w, x0, y0)
            if why: return why
        return None


def step_rate(n, w):
    x0, y0 = n.od[-1][2], n.od[-1][3]; t0 = time.monotonic(); y0w = n.yaw(); n.log(k='step', kind='rate', w=w)
    ab = n.hold(6.0, w, x0, y0); t1 = time.monotonic()
    z = n.hold(3.0, 0.0, x0, y0) if not ab else None
    turned = sum(math.remainder(n.od[i + 1][1] - n.od[i][1], 2 * math.pi) for i in range(len(n.od) - 1) if n.od[i][0] >= t0 and n.od[i + 1][0] <= t1)
    return {'kind': 'rate', 'w_cmd': w, 'rate_0_1.5': n.rate(t0, t0 + 1.5), 'rate_1.5_6': n.rate(t0 + 1.5, t1),
            'u_1.5_6(mean,min,max)': n.umean(t0 + 1.5, t1), 'turn_deg': math.degrees(turned),
            'coast_deg': math.degrees((n.rate(t1, t1 + 2.0) or 0) * 2.0), 'drift_m': math.hypot(n.od[-1][2] - x0, n.od[-1][3] - y0),
            'mc': n.mc, 'abort': ab or z}, ab or z


def step_turn(n, sign):
    x0, y0 = n.od[-1][2], n.od[-1][3]; t0 = time.monotonic(); n.log(k='step', kind='turn90', sign=sign)
    start = n.yaw(); goal = sign * math.pi / 2
    err = lambda: goal - math.remainder(n.yaw() - start, 2 * math.pi)
    mode, since, corr, done_t, peak, ab = 'turn', t0, 0, None, 0.0, None
    while time.monotonic() - t0 < 20.0:
        e = err(); peak = max(peak, sign * (goal - e))
        if mode == 'turn':
            if abs(e) < math.radians(5) or (done_t is None and e * sign < 0):
                mode, since = 'settle', time.monotonic(); done_t = done_t or since - t0
                ab = n.send(0.0, x0, y0)
            else:
                ab = n.send(math.copysign(min(0.5, max(0.1, abs(e))), e), x0, y0)
        else:
            if time.monotonic() - since > 2.0 and abs(e) > math.radians(5) and corr < 3:
                mode, corr = 'turn', corr + 1
            elif time.monotonic() - since > 2.0 and abs(e) <= math.radians(5):
                break
            ab = n.send(0.0, x0, y0)
        if ab: break
    if not ab: ab = n.hold(2.0, 0.0, x0, y0)
    t1 = time.monotonic()
    return {'kind': 'turn90', 'dir': 'CCW' if sign > 0 else 'CW', 'time_to_5deg_s': done_t,
            'overshoot_deg': math.degrees(peak - math.pi / 2), 'final_err_deg': math.degrees(err()), 'corrections': corr,
            'total_s': t1 - t0, 'u(mean,min,max)': n.umean(t0, t1), 'peak_rate': max((abs(n.rate(t, t + 0.3) or 0) for t in
            [t0 + 0.1 * k for k in range(int((t1 - t0) * 10))]), default=None),
            'drift_m': math.hypot(n.od[-1][2] - x0, n.od[-1][3] - y0), 'mc': n.mc, 'abort': ab}, ab


def main():
    if os.environ.get('CONFIRM') != '1':
        sys.exit('refusing: live motion needs CONFIRM=1 and explicit operator approval')
    rclpy.init(); raw = open(OUT + '_raw.jsonl', 'w'); n = T(raw)
    end = time.monotonic() + 3.0
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
    pre = {'adapter': n.ad, 'gc': {k: n.gc.get(k) for k in ('busy', 'latched', 'fault', 'trusted')}, 'mc': n.mc}
    print('preflight', json.dumps(pre)[:700])
    if not (n.ad.get('registration') == 'registered' and n.ad.get('state') == 'idle' and n.ad.get('ready') and not n.ad.get('latch')):
        sys.exit('preflight failed: adapter')
    if n.gc.get('busy') or n.mc is None or n.mc[1] != 'STAND_DEFAULT' or not n.od:
        sys.exit('preflight failed: gc busy or MC not STAND_DEFAULT or no odom')
    f = n.arm.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
    if not (f.result() and f.result().success): sys.exit(f'arm refused: {f.result()}')
    res, abort = [], None
    try:
        for fn, a in ((step_rate, 0.5), (step_rate, -0.5), (step_turn, 1), (step_turn, -1)):
            r, abort = fn(n, a); print(json.dumps(r, default=str)); res.append(r)
            if abort: break
            if fn is step_turn: n.hold(1.0, 0.0, n.od[-1][2], n.od[-1][3])
    finally:
        for _ in range(10):
            n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
        json.dump({'path': __doc__, 'preflight': pre, 'steps': res, 'abort': abort, 'adapter_end': n.ad, 'mc_end': n.mc},
                  open(OUT + '_summary.json', 'w'), indent=1, default=str)
        raw.close(); n.destroy_node(); rclpy.try_shutdown()
    print('SUMMARY', OUT + '_summary.json', 'abort=', abort)


if __name__ == '__main__':
    main()
