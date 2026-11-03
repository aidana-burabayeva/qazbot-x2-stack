#!/usr/bin/env python3
"""P19 calibration part 2 (operator-approved 2026-10-29). Path: Twist -> /qazbot_d1_map/cmd_vel_authorized ->
x2_motion_adapter (live, source sdk_qazbot_nav2_1029a, priority 30, McLocomotionVelocity on /aima/mc/locomotion/velocity
at 50 Hz) -> MC. Bypasses Nav2 (RPP/smoother) and the GC; adapter safety (trust, latch, takeover) stays active.
Steps: stand->CCW 0.8; stand->CCW 1.0; walk 0.3 m/s 1 s->CCW 0.6; walk->zero 0.8 s->CCW 0.6; walk->CW 0.6. Turn 6 s, rest 3 s.
Logs (>=20 Hz): chest yaw (native odom = leg_odometry pose of lidar_imu_chest_front), torso IMU yaw + waist_yaw_joint via a parallel ros2 bag (python cannot keep up with 500 Hz),
pelvis yaw = chest - waist_yaw (TF chain pelvis->waist_yaw->waist_pitch->torso->lidar_chest; feet NOT measured: python TF too slow), MC state/action/fsm/source/speed_mode, body_pose.
Abort: translation >0.3 m per turn step (walk phase >0.5 m), adapter latch, MC source change, MC action not STAND/LOCOMOTION_DEFAULT."""
import json, math, os, sys, time
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading
import rclpy.executors
from aimdk_msgs.msg import McCommonState, JointStateArray, McBodyPose

NS = '/qazbot_d1_map'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), time.strftime('p19_turn_calib2_%Y%m%d_%H%M%S'))
OK_ACTIONS = ('STAND_DEFAULT', 'LOCOMOTION_DEFAULT')
TURN, REST, WALK_V, WALK_S, KICK = 6.0, 3.0, 0.3, 1.0, 0.8
STEPS = [('stand', 0.8), ('stand', 1.0), ('walk', 0.6), ('walk_kick', 0.6), ('walk', -0.6)]
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
BE = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


class C(Node):
    def __init__(self, raw):
        super().__init__('p19_turn_calib2')
        self.raw, self.S, self.ad, self.gc, self.mc, self.last = raw, {k: [] for k in ('chest', 'torso', 'chesti', 'waist', 'pelvis', 'lfoot', 'rfoot')}, {}, {}, None, {}
        self.xy, self.tfpar = None, {}
        self.pub = self.create_publisher(Twist, NS + '/cmd_vel_authorized', 10)
        self.create_subscription(Odometry, '/qazbot_native/odom', self._odom, BE)
        self.create_subscription(McCommonState, '/aima/mc/common/state', self._mc, BE)
        self.create_subscription(McBodyPose, '/aima/mc/body_pose', lambda m: self.log(k='body_pose', src=m.source, twist_waist=m.twist_waist), BE)
        self.create_subscription(String, NS + '/motion_adapter/status', lambda m: setattr(self, 'ad', json.loads(m.data)), 10)
        self.create_subscription(String, NS + '/goal_coordinator/status', lambda m: setattr(self, 'gc', json.loads(m.data)), 10)
        self.arm = self.create_client(Trigger, NS + '/motion_adapter/arm_motion')

    def log(self, **k):
        k['t'] = time.monotonic(); self.raw.write(json.dumps(k) + '\n')

    def _s(self, key, y, hz=50.0):
        now = time.monotonic()
        if now - self.last.get(key, 0) < 1 / hz: return
        self.last[key] = now; self.S[key].append((now, y)); self.log(k=key, yaw=y)

    def _odom(self, m):
        p = m.pose.pose.position; self.xy = (p.x, p.y); self._s('chest', yawq(m.pose.pose.orientation), 100)

    def _waist(self, m):
        for j in m.joints:
            if j.name == 'waist_yaw_joint': self._s('waist', j.position)

    def _tf(self):   # pelvis_world = chest_world - yaw(pelvis->lidar_imu_chest_front); foot_world = pelvis + yaw(pelvis->ankle)
        if not self.S['chest']: return
        try:
            rel = lambda c: yawq(self.tfb.lookup_transform('pelvis', c, rclpy.time.Time()).transform.rotation)
            pel = self.S['chest'][-1][1] - rel('lidar_imu_chest_front')
            self._s('pelvis', pel); self._s('lfoot', pel + rel('left_ankle_roll_link')); self._s('rfoot', pel + rel('right_ankle_roll_link'))
            self.tfpar['ok'] = True
        except Exception as e:
            self.tfpar['err'] = str(e)[:120]

    def _mc(self, m):
        s = (m.input_source.name, m.action_info.action_desc, m.fsm_state.current_state, m.speed_status.speed_mode.value)
        if s != self.mc: self.log(k='mc', source=s[0], action=s[1], fsm=s[2], speed_mode=s[3])
        self.mc = s; self.mclog.append((time.monotonic(),) + s)
    mclog = []

    def spin(self, s, v=None, w=0.0, ref=None, lim=0.3):
        end = time.monotonic() + s
        while time.monotonic() < end:
            if v is not None:
                t = Twist(); t.linear.x, t.angular.z = v, w; self.pub.publish(t); self.log(k='cmd', v=v, w=w)
            time.sleep(0.02)
            a = self.ad
            if a.get('latch'): return 'adapter latch ' + a['latch']
            if self.mc and self.mc[0] != self.src: return f'mc source {self.mc}'
            if self.mc and self.mc[1] not in OK_ACTIONS: return f'mc action {self.mc}'
            if ref and self.xy and math.hypot(self.xy[0] - ref[0], self.xy[1] - ref[1]) > lim: return f'translated >{lim} m'
        return None

    def at(self, key, t):
        xs = self.S[key]; best = None
        for s in xs:
            if s[0] <= t: best = s
            else: break
        return best[1] if best else None

    def unwrap(self, key, a, b):
        xs = [s for s in self.S[key] if a <= s[0] <= b]
        if len(xs) < 2: return []
        out, acc = [(xs[0][0], 0.0)], 0.0
        for i in range(1, len(xs)):
            acc += math.remainder(xs[i][1] - xs[i - 1][1], 2 * math.pi); out.append((xs[i][0], acc))
        return out

    def rate(self, key, a, b):
        u = self.unwrap(key, a, b)
        return (u[-1][1] - u[0][1]) / (u[-1][0] - u[0][0]) if len(u) > 2 and u[-1][0] > u[0][0] else None


def summarize(n, kind, w, t0, t1, t2, abort):
    sgn = 1 if w > 0 else -1; r = lambda x: None if x is None else round(x, 3)
    out = {'pre': kind, 'w_cmd': w, 'dir': 'CCW' if w > 0 else 'CW', 'abort': abort}
    out['chest_rate_0_1.5'] = r(n.rate('chest', t0, t0 + 1.5)); out['chest_rate_1.5_6'] = r(n.rate('chest', t0 + 1.5, t1))
    u = n.unwrap('chest', t0 + 1.5, t1)
    if len(u) > 5:
        segs = [(u[i][1] - u[i - 5][1]) / (u[i][0] - u[i - 5][0]) for i in range(5, len(u), 5)]
        out['stall_frac_1.5_6'] = round(sum(abs(s) < 0.05 for s in segs) / len(segs), 2)
    for key in ('chest', 'pelvis', 'lfoot', 'rfoot', 'torso', 'chesti'):
        u = n.unwrap(key, t0, t2)
        if not u: continue
        cmdend = [y for t, y in u if t <= t1][-1]; peak = max(y * sgn for t, y in u) * sgn; settled = u[-1][1]
        out[key] = {'d_turn_deg': r(math.degrees(cmdend)), 'peak_deg': r(math.degrees(peak)), 'settled_deg': r(math.degrees(settled)),
                    'reversal_deg': r(math.degrees(abs(peak - settled)))}
    w0, w1, w2 = n.at('waist', t0), n.at('waist', t1), n.at('waist', t2)
    if None not in (w0, w1, w2):
        out['waist_yaw_deg'] = {'start': r(math.degrees(w0)), 'cmd_end': r(math.degrees(w1)), 'settled': r(math.degrees(w2)),
                                'max_abs': r(math.degrees(max(abs(y) for t, y in n.S['waist'] if t0 <= t <= t2)))}
        if 'chest' in out:   # cross-check: pelvis = chest - waist_yaw joint
            out['pelvis_from_waist'] = {'d_turn_deg': r(out['chest']['d_turn_deg'] - math.degrees(w1 - w0)),
                             'settled_deg': r(out['chest']['settled_deg'] - math.degrees(w2 - w0))}
    out['mc_timeline'] = [(round(m[0] - t0, 2),) + m[1:] for i, m in enumerate(n.mclog) if t0 - 1.5 <= m[0] <= t2 and (i == 0 or m[1:] != n.mclog[i - 1][1:])]
    out['tf'] = dict(n.tfpar)
    return out


def main():
    if os.environ.get('CONFIRM') != '1': sys.exit('needs CONFIRM=1 (operator approval)')
    rclpy.init(); raw = open(OUT + '_raw.jsonl', 'w'); n = C(raw); n.src = None
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    te = time.monotonic() + 25.0
    while time.monotonic() < te and not (n.ad and n.mc and len(n.S['chest']) > 20): time.sleep(0.1)
    n.spin(1.0)
    pre = {'adapter': n.ad, 'gc': {k: n.gc.get(k) for k in ('busy', 'latched', 'fault', 'trusted')}, 'mc': n.mc,
           'samples': {k: len(v) for k, v in n.S.items()}, 'tf': dict(n.tfpar)}
    print('preflight', json.dumps(pre)[:700])
    if not (n.ad.get('registration') == 'registered' and n.ad.get('state') == 'idle' and n.ad.get('ready') and not n.ad.get('latch')):
        sys.exit('preflight failed: adapter')
    if n.gc.get('busy') or n.gc.get('latched') or not n.mc or n.mc[1] != 'STAND_DEFAULT' or not n.xy: sys.exit('preflight failed')
    n.src = n.mc[0]
    f = n.arm.call_async(Trigger.Request()); te = time.monotonic() + 2.0
    while not f.done() and time.monotonic() < te: time.sleep(0.02)
    if not (f.result() and f.result().success): sys.exit(f'arm refused {f.result()}')
    res, abort = [], None
    try:
        for kind, w in STEPS:
            n.log(k='step', kind=kind, w=w)
            if kind.startswith('walk'):
                abort = n.spin(WALK_S, WALK_V, 0.0, n.xy, 0.5)
                if not abort and kind == 'walk_kick': abort = n.spin(KICK, 0.0, 0.0, n.xy, 0.3)
            t0 = time.monotonic(); ref = n.xy
            abort = abort or n.spin(TURN, 0.0, w, ref, 0.3); t1 = time.monotonic()
            z = n.spin(REST, 0.0, 0.0, ref, 0.4) if not abort else None
            t2 = time.monotonic(); s = summarize(n, kind, w, t0, t1, min(t2, t1 + 2.0) if not abort else t1, abort or z)
            s['drift_m'] = round(math.hypot(n.xy[0] - ref[0], n.xy[1] - ref[1]), 3)
            print(json.dumps(s)); res.append(s)
            if abort or z: abort = abort or z; break
    finally:
        for _ in range(10): n.pub.publish(Twist()); time.sleep(0.05)
        json.dump({'path': __doc__, 'steps': res, 'abort': abort, 'preflight': pre, 'adapter_end': n.ad, 'mc_end': n.mc},
                  open(OUT + '_summary.json', 'w'), indent=1)
        ex.shutdown(); raw.close(); n.destroy_node(); rclpy.try_shutdown()
    print('SUMMARY', OUT + '_summary.json', 'abort=', abort)


if __name__ == '__main__':
    main()
