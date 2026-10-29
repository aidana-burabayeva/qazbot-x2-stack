#!/usr/bin/env python3
"""P19 A/B: in-place yaw sent DIRECTLY to vendor MC, bypassing x2_motion_adapter, Nav2 and GC (operator-approved 2026-10-29).
AIMDK path: SetMcInputSource ADD (/aimdk_5Fmsgs/srv/SetMcInputSource, source sdk_qazbot_calib_direct, priority 30 = adapter's,
timeout 1000 ms) -> McLocomotionVelocity on /aima/mc/locomotion/velocity at 50 Hz (BEST_EFFORT depth 1, same as the adapter)
-> DELETE of the source at exit. Steps: CCW w=0.2..0.6 then CW 0.2..0.6, 10 s each, 3 s zero between.
Abort: per-step translation >0.3 m, MC action not STAND/LOCOMOTION_DEFAULT, MC source != ours after 0.6 s, adapter publishing, SIGINT/error.
Zeros always sent on exit. Needs CONFIRM=1."""
import json, math, os, signal, sys, threading, time
import rclpy, rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from aimdk_msgs.msg import McCommonState, McLocomotionVelocity, MessageHeader, McInputAction
from aimdk_msgs.srv import SetMcInputSource

SRC, PRIO = 'sdk_qazbot_calib_direct', 30
RATES = [float(x) for x in os.environ.get('RATES', '0.2,0.3,0.4,0.5,0.6').split(',')]
TURN, REST = float(os.environ.get('DUR', '10')), float(os.environ.get('REST', '3'))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), time.strftime('p19_mc_direct_%Y%m%d_%H%M%S'))
OK_ACTIONS = ('STAND_DEFAULT', 'LOCOMOTION_DEFAULT')
yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
STOP = [False]


class D(Node):
    def __init__(self, raw):
        super().__init__('p19_mc_direct')
        self.raw, self.od, self.mc, self.ad, self.gc, self.mclog = raw, [], None, {}, {}, []
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', be)
        self.cli = self.create_client(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
        self.create_subscription(Odometry, '/qazbot_native/odom', self._odom, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(McCommonState, '/aima/mc/common/state', self._mc,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        self.create_subscription(String, '/qazbot_d1_map/motion_adapter/status', lambda m: setattr(self, 'ad', json.loads(m.data)), 10)
        self.create_subscription(String, '/qazbot_d1_map/goal_coordinator/status', lambda m: setattr(self, 'gc', json.loads(m.data)), 10)

    def log(self, **k):
        k['t'] = time.time(); self.raw.write(json.dumps(k) + '\n')

    def _odom(self, m):
        p = m.pose.pose; self.od.append((time.time(), yawq(p.orientation), p.position.x, p.position.y))

    def _mc(self, m):
        s = (m.input_source.name, m.action_info.action_desc, m.fsm_state.current_state, m.speed_status.speed_mode.value)
        if s != self.mc: self.log(k='mc', s=s)
        self.mc = s; self.mclog.append((time.time(),) + s)

    def send(self, w):
        m = McLocomotionVelocity(); m.header = MessageHeader(); m.header.stamp = self.get_clock().now().to_msg()
        m.source = SRC; m.forward_velocity, m.lateral_velocity, m.angular_velocity = 0.0, 0.0, float(w); self.pub.publish(m)

    def call(self, action):
        r = SetMcInputSource.Request(); r.action.value = action
        r.input_source.name, r.input_source.priority, r.input_source.timeout = SRC, PRIO, 1000
        r.request.header.stamp = self.get_clock().now().to_msg()
        f = self.cli.call_async(r); te = time.time() + 3
        while not f.done() and time.time() < te: time.sleep(0.02)
        return f.result().response.header.code if f.done() and f.result() else 'timeout'

    def run(self, s, w, check_src):
        x0 = self.od[-1][2:]; t0 = time.time(); end = t0 + s
        while time.time() < end:
            if STOP[0]: return 'SIGINT'
            self.send(w); self.log(k='cmd', w=w); time.sleep(0.02)
            if self.mc[1] not in OK_ACTIONS: return f'mc action {self.mc}'
            if check_src and time.time() - t0 > 0.6 and self.mc[0] != SRC: return f'mc source {self.mc}'
            if math.hypot(self.od[-1][2] - x0[0], self.od[-1][3] - x0[1]) > 0.3: return 'translated >0.3 m'
            if self.ad.get('state') == 'active': return 'adapter active'
        return None

    def rate(self, a, b):
        ys = [s for s in self.od if a <= s[0] <= b]
        if len(ys) < 3: return None
        return sum(math.remainder(ys[i + 1][1] - ys[i][1], 2 * math.pi) for i in range(len(ys) - 1)) / (ys[-1][0] - ys[0][0])


def main():
    if os.environ.get('CONFIRM') != '1': sys.exit('needs CONFIRM=1')
    signal.signal(signal.SIGINT, lambda *a: STOP.__setitem__(0, True))
    rclpy.init(); raw = open(OUT + '_raw.jsonl', 'w'); n = D(raw)
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
    te = time.time() + 25
    while time.time() < te and not (n.ad and n.mc and len(n.od) > 20 and n.cli.service_is_ready()): time.sleep(0.1)
    c0 = n.ad.get('counters', {}).get('published'); time.sleep(2.0); c1 = n.ad.get('counters', {}).get('published')
    pre = {'adapter': n.ad, 'gc': {k: n.gc.get(k) for k in ('busy', 'latched', 'fault')}, 'mc': n.mc, 'adapter_published': [c0, c1]}
    print('preflight', json.dumps(pre)[:600])
    if not (n.ad.get('state') == 'idle' and c0 == c1 and c0 is not None and not n.gc.get('busy') and n.mc and n.mc[1] == 'STAND_DEFAULT'):
        sys.exit('preflight failed')
    res, abort, added = [], None, False
    try:
        code = n.call(McInputAction.INPUTACTION_ADD); n.log(k='add', code=code); print('ADD code', code)
        if code != 0: abort = f'ADD failed {code}'; return
        added = True
        for w in [s * r for s in (1.0, -1.0) for r in RATES]:
            t0 = time.time(); abort = n.run(TURN, w, True); t1 = time.time()
            z = n.run(REST, 0.0, False) if not abort else None; t2 = time.time()
            sg = 1 if w > 0 else -1
            ys = [s for s in n.od if t0 <= s[0] <= min(t1 + 2, t2)]; acc, u = 0.0, [0.0]
            for i in range(1, len(ys)): acc += math.remainder(ys[i][1] - ys[i - 1][1], 2 * math.pi); u.append(acc)
            ue = [u[i] for i in range(len(ys)) if ys[i][0] <= t1]
            rec = {'w_cmd': w, 'dir': 'CCW' if w > 0 else 'CW', 't0': t0, 't1': t1, 'rate_0_1.5': n.rate(t0, t0 + 1.5),
                   'rate_1.5_end': n.rate(t0 + 1.5, t1), 'turn_deg': math.degrees(ue[-1]) if ue else None,
                   'reversal_deg': math.degrees(abs(max(x * sg for x in u) * sg - u[-1])) if u else None,
                   'mc_timeline': [(round(m[0] - t0, 2),) + m[1:] for i, m in enumerate(n.mclog) if t0 - 0.5 <= m[0] <= t2 and (i == 0 or m[1:] != n.mclog[i - 1][1:])],
                   'abort': abort or z}
            print(json.dumps(rec)); res.append(rec)
            if abort or z: abort = abort or z; break
    finally:
        for _ in range(15): n.send(0.0); time.sleep(0.02)
        dc = n.call(McInputAction.INPUTACTION_DELETE) if added else None
        n.log(k='delete', code=dc); print('DELETE code', dc)
        time.sleep(1.5)
        json.dump({'path': __doc__, 'steps': res, 'abort': abort, 'preflight': pre, 'delete_code': dc, 'mc_end': n.mc, 'adapter_end': n.ad},
                  open(OUT + '_summary.json', 'w'), indent=1)
        print('SUMMARY', OUT + '_summary.json', 'abort=', abort, 'mc_end', n.mc)
        raw.close(); ex.shutdown(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
