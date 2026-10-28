#!/usr/bin/env python3
"""P2-T instrumented SHADOW copy of localization_trust (no source edits: imports ws/src modules and wraps methods).
Runs under namespace /qazbot_trace (override with --ns): subscribes to the SAME live inputs with the SAME QoS via
remaps; publishes only /qazbot_trace/localization_trust/{trusted,status} (unused by anything). The AMCL no-motion
refresh helper is disabled (go2_nomotion_on_initialpose:=false) and its client resolves to a non-existent
/qazbot_trace/request_nomotion_update. pose_store off. Never publishes TF/initialpose/cmd_vel.
Logs: T3 input-callback ENTER (before input_lock), T3L lock acquired, T4 stamp accept/reject (+ source stamp,
ros_now, previous), T5 tick enter / evaluate enter / evaluate exit / tick exit, T6 verdict reasons + input ages.
Usage: trust_trace_copy.py --duration 90 --out DIR --tag B
"""
import argparse, functools, json, math, os, sys, threading, time
ap = argparse.ArgumentParser()
ap.add_argument('--duration', type=float, default=90)
ap.add_argument('--out', required=True)
ap.add_argument('--tag', required=True)
ap.add_argument('--ns', default='/qazbot_trace')
a = ap.parse_args()
NAV = '/home/agi/control_backend/ws/src/navigation'
sys.path.insert(0, NAV)
import rclpy
import rclpy.node
import localization_trust_node as T
import loc_trust as L

LOG = []          # tuples, appended from 2 executor threads (list.append is atomic in CPython)
cur = threading.local()

# T3: wrap every subscription callback created on the input node (before input_lock is taken)
_orig_cs = rclpy.node.Node.create_subscription
def create_subscription(self, typ, topic, cb, qos, **kw):
    tname = topic
    def enter(msg):
        cur.topic = tname
        st = getattr(getattr(msg, 'header', None), 'stamp', None)
        LOG.append(('T3', tname, time.monotonic_ns(), time.time_ns(),
                    st.sec * 1_000_000_000 + st.nanosec if st is not None else -1, ''))
        return cb(msg)
    return _orig_cs(self, typ, topic, enter, qos, **kw)
rclpy.node.Node.create_subscription = create_subscription

def wrap_lock(name):
    f = getattr(T.TrustNode, name)
    @functools.wraps(f)
    def w(self, m, *aa):
        st = getattr(getattr(m, 'header', None), 'stamp', None)
        LOG.append(('T3L', name, time.monotonic_ns(), time.time_ns(),
                    st.sec * 1_000_000_000 + st.nanosec if st is not None else -1, ''))
        return f(self, m, *aa)
    setattr(T.TrustNode, name, w)
for n in ('_scan', '_odom', '_tf', '_amcl', '_cloud', '_map', '_initialpose'):
    wrap_lock(n)

_orig_stamp = T.TrustNode._stamp
@functools.wraps(_orig_stamp)
def _stamp(self, key, stamp, future=0.0):
    prev = self.stamps[key].last
    r = _orig_stamp(self, key, stamp, future)
    src = stamp.sec * 1_000_000_000 + stamp.nanosec
    ros_now = self.get_clock().now().nanoseconds
    why = 'ok' if math.isfinite(r) else ('dup_or_old' if prev is not None and src * 1e-9 <= prev else
                                         'future' if src * 1e-9 > ros_now * 1e-9 + future else 'invalid')
    LOG.append(('T4', key, time.monotonic_ns(), ros_now, src, f'{why};mono_eff={r:.6f}'))
    return r
T.TrustNode._stamp = _stamp

_orig_tick = T.TrustNode._tick
@functools.wraps(_orig_tick)
def _tick(self):
    LOG.append(('T5in', 'tick', time.monotonic_ns(), time.time_ns(), -1, ''))
    r = _orig_tick(self)
    LOG.append(('T5out', 'tick', time.monotonic_ns(), time.time_ns(), -1, ''))
    return r
T.TrustNode._tick = _tick

_orig_eval = L.TrustEvaluator.evaluate
def evaluate(self, inp):
    t0 = time.monotonic_ns()
    r = _orig_eval(self, inp)
    t1 = time.monotonic_ns()
    node = NODE[0]
    extra = dict(now=inp.now, scan_t=inp.scan_stamp, odom_t=inp.odom_stamp, odom_msg_t=inp.odom_msg_stamp,
                 scan_src=node.scan_src, odom_src=node.stamps['odom'].last, odom_msg_src=node.stamps['odom_msg'].last,
                 reasons=r.get('reasons'), eval_ns=t1 - t0)
    LOG.append(('T5eval', 'eval', t0, t1, -1, json.dumps(extra, default=float)))
    return r
L.TrustEvaluator.evaluate = evaluate

NODE = [None]
ns = a.ns
args = ['--ros-args', '-r', f'__ns:={ns}',
        '-r', f'{ns}/amcl_pose:=/qazbot_d1_map/amcl_pose', '-r', f'{ns}/particle_cloud:=/qazbot_d1_map/particle_cloud',
        '-r', f'{ns}/map:=/qazbot_d1_map/map', '-r', f'{ns}/initialpose:=/qazbot_d1_map/initialpose',
        '-p', 'map_yaml:=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml',
        '-p', 'expected_map_sha256:=f6032f75ffd386361993a4b4864cf0816713fc27f6d037c88d929ae5d48562fb',
        '-p', 'admission_policy:=go2', '-p', 'go2_nomotion_on_initialpose:=false', '-p', "pose_store:=''"]
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
rclpy.init(args=args)
node = T.TrustNode(); NODE[0] = node
assert node.get_namespace() == ns, node.get_namespace()
assert not node.seed_update_enabled
ex = MultiThreadedExecutor(num_threads=2); ex.add_node(node)          # same executor layout as live main()
inputs = SingleThreadedExecutor(); inputs.add_node(node.input_node)
stop = threading.Event()
def rx():
    while rclpy.ok() and not stop.is_set():
        inputs.spin_once(timeout_sec=.1)
th = threading.Thread(target=rx, name='localization_inputs'); th.start()
meta = dict(pid=os.getpid(), ns=node.get_namespace(), input_ns=node.input_node.get_namespace(),
            ros_minus_real_ns=node.get_clock().now().nanoseconds - time.time_ns(),
            subs=[(s.topic_name) for s in node.input_node.subscriptions])
print(json.dumps(meta), flush=True)
t_end = time.monotonic() + a.duration
while time.monotonic() < t_end:
    ex.spin_once(timeout_sec=.1)
stop.set(); th.join()
with open(f'{a.out}/copy_{a.tag}.tsv', 'w') as f:
    f.write('ev\tkey\tmono_ns\tros_or_real_ns\tsrc_ns\textra\n')
    for r in list(LOG):
        f.write('\t'.join(str(x) for x in r) + '\n')
json.dump(meta, open(f'{a.out}/copy_meta_{a.tag}.json', 'w'))
print('copy done', len(LOG), flush=True)
ex.shutdown(); inputs.shutdown(); node.input_node.destroy_node(); node.destroy_node(); rclpy.try_shutdown()
