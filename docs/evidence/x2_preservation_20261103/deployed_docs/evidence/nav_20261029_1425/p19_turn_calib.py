#!/usr/bin/env python3
"""P19 in-place yaw calibration through the EXISTING adapter path (Twist -> /qazbot_d1_map/cmd_vel_authorized ->
x2_motion_adapter -> MC). Needs CONFIRM=1 and operator approval. Env: RATES (default 0.2,0.4,0.6,0.8,1.0), DUR 6, REST 2.
Aborts on: adapter latch/not registered, MC source change, MC action outside STAND/LOCOMOTION_DEFAULT, translation >0.3 m.
Writes <OUT>_raw.jsonl (every odom/MC/command sample) and <OUT>_summary.json."""
import json, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from aimdk_msgs.msg import McCommonState

NS = '/qazbot_d1_map'
RATES = [float(x) for x in os.environ.get('RATES', '0.2,0.4,0.6,0.8,1.0').split(',')]
DUR, REST = float(os.environ.get('DUR', '6.0')), float(os.environ.get('REST', '2.0'))
OUT = os.environ.get('OUT', os.path.join(os.path.dirname(os.path.abspath(__file__)), time.strftime('p19_turn_calib_%Y%m%d_%H%M%S')))
OK_ACTIONS = ('STAND_DEFAULT', 'LOCOMOTION_DEFAULT')
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Calib(Node):
    def __init__(self, raw):
        super().__init__('p19_turn_calib')
        self.raw, self.od, self.amcl, self.ad, self.gc, self.mc = raw, [], [], {}, {}, None
        self.pub = self.create_publisher(Twist, NS + '/cmd_vel_authorized', 10)
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/qazbot_native/odom', self._odom, be)
        self.create_subscription(PoseWithCovarianceStamped, NS + '/amcl_pose', self._amcl, 10)
        self.create_subscription(McCommonState, '/aima/mc/common/state', self._mc,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(String, NS + '/motion_adapter/status', lambda m: setattr(self, 'ad', json.loads(m.data)), 10)
        self.create_subscription(String, NS + '/goal_coordinator/status', lambda m: setattr(self, 'gc', json.loads(m.data)), 10)
        self.arm = self.create_client(Trigger, NS + '/motion_adapter/arm_motion')

    def log(self, **k):
        k['t'] = time.monotonic(); self.raw.write(json.dumps(k) + '\n')

    def _odom(self, m):
        p = m.pose.pose; s = (time.monotonic(), yawq(p.orientation), p.position.x, p.position.y)
        self.od.append(s); self.log(k='odom', yaw=s[1], x=s[2], y=s[3], wz=m.twist.twist.angular.z)

    def _amcl(self, m):
        p = m.pose.pose; self.amcl.append((time.monotonic(), yawq(p.orientation))); self.log(k='amcl', yaw=self.amcl[-1][1], x=p.position.x, y=p.position.y)

    def _mc(self, m):
        self.mc = (m.input_source.name, m.action_info.action_desc, m.fsm_state.current_state, m.body_status.value)
        self.log(k='mc', source=self.mc[0], action=self.mc[1], fsm=self.mc[2], body=self.mc[3])

    def rate(self, a, b, src=None):
        ys = [s for s in (src or self.od) if a <= s[0] <= b]
        if len(ys) < 3: return None
        tot = sum(math.remainder(ys[i + 1][1] - ys[i][1], 2 * math.pi) for i in range(len(ys) - 1))
        return tot / (ys[-1][0] - ys[0][0])

    def stall_frac(self, a, b, win=0.25):
        rs, t = [], a
        while t + win <= b:
            r = self.rate(t, t + win); t += win
            if r is not None: rs.append(abs(r) < 0.05)
        return round(sum(rs) / len(rs), 2) if rs else None

    def check(self, x0, y0, src_ref):
        a = self.ad
        if a.get('latch') or a.get('registration') != 'registered': return f'adapter {a.get("state")} latch={a.get("latch")}'
        if self.mc is None or self.mc[0] != src_ref: return f'mc source {self.mc}'
        if self.mc[1] not in OK_ACTIONS: return f'mc action {self.mc}'
        if self.od and math.hypot(self.od[-1][2] - x0, self.od[-1][3] - y0) > 0.3: return 'translated >0.3 m'
        return None

    def run_for(self, s, w, x0, y0, src_ref):
        end = time.monotonic() + s
        while time.monotonic() < end:
            t = Twist(); t.angular.z = w; self.pub.publish(t); self.log(k='cmd', w=w)
            rclpy.spin_once(self, timeout_sec=0.05)
            why = self.check(x0, y0, src_ref)
            if why: return why
        return None


def main():
    if os.environ.get('CONFIRM') != '1':
        sys.exit('refusing: live motion needs CONFIRM=1 and explicit operator approval')
    rclpy.init(); raw = open(OUT + '_raw.jsonl', 'w'); n = Calib(raw)
    end = time.monotonic() + 2.5
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
    pre = {'adapter': n.ad, 'gc': {k: n.gc.get(k) for k in ('busy', 'latched', 'fault', 'trusted')}, 'mc': n.mc}
    print('preflight', json.dumps(pre)[:600])
    if not (n.ad.get('registration') == 'registered' and n.ad.get('state') == 'idle' and n.ad.get('ready') and not n.ad.get('latch')):
        sys.exit('preflight failed: adapter')
    if n.gc.get('busy') or n.gc.get('latched') or n.mc is None or n.mc[1] != 'STAND_DEFAULT' or not n.od:
        sys.exit('preflight failed: gc busy/latched or MC not STAND_DEFAULT or no odom')
    src_ref = n.mc[0]
    f = n.arm.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
    if not (f.result() and f.result().success): sys.exit(f'arm refused: {f.result()}')
    x0, y0 = n.od[-1][2], n.od[-1][3]; steps, abort = [], None
    try:
        for w in [s * r for r in RATES for s in (-1.0, 1.0)]:
            mc0 = n.mc; t0 = time.monotonic(); n.log(k='step', w=w)
            abort = n.run_for(DUR, w, x0, y0, src_ref); t1 = time.monotonic()
            z = n.run_for(REST, 0.0, x0, y0, src_ref) if not abort else None
            rec = {'w_cmd': w, 'dir': 'CCW' if w > 0 else 'CW', 'dur': round(t1 - t0, 2),
                   'rate_0_1.5': n.rate(t0, t0 + 1.5), 'rate_1.5_6': n.rate(t0 + 1.5, t1),
                   'stall_frac_1.5_6': n.stall_frac(t0 + 1.5, t1), 'yaw_total': (n.rate(t0, t1) or 0) * (t1 - t0),
                   'coast_rad': (n.rate(t1, t1 + REST) or 0) * REST,
                   'amcl_rate': n.rate(t0, t1 + 1.0, n.amcl), 'mc_before': mc0, 'mc_after': n.mc,
                   'drift_m': math.hypot(n.od[-1][2] - x0, n.od[-1][3] - y0), 'abort': abort or z}
            print(json.dumps(rec)); steps.append(rec)
            if abort or z: abort = abort or z; break
    finally:
        for _ in range(10):
            n.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
        json.dump({'rates': RATES, 'dur': DUR, 'rest': REST, 'preflight': pre, 'steps': steps, 'abort': abort,
                   'adapter_end': n.ad, 'mc_end': n.mc}, open(OUT + '_summary.json', 'w'), indent=1)
        raw.close(); n.destroy_node(); rclpy.try_shutdown()
    print('SUMMARY', OUT + '_summary.json', 'abort=', abort)


if __name__ == '__main__':
    main()
