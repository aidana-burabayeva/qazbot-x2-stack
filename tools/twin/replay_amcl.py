#!/usr/bin/env python3
"""Replay level 2: recorded scan + odom->base_footprint  ->  REAL map_server + REAL AMCL (production parameters).

Authoritative publisher per level (no double publishing):
  * replayer publishes ONLY the inputs of AMCL: /qazbot_lio/scan and the odom->base_footprint TF edge,
  * the recorded map->odom (the robot's AMCL output), amcl_pose and Trust topics are NEVER published;
    they are read from the bag only as the reference to compare the re-run against.
Header stamps are shifted by one constant (so AMCL runs on the normal wall clock, use_sim_time=false, exactly as in
production); order, spacing, frame_id and values are untouched. Pacing follows the bag's receive times (1x).
Every recorded /initialpose is replayed at its recorded receive time with its original header, pose and covariance.
A selected window without an operator seed is refused. AMCL output is never used to construct an input seed.
Randomness: AMCL's particle filter; this entrypoint runs one replay.

Usage: replay_amcl.py BAG_DIR RESULT_NAME [--max-seconds N]
Compares: step statistics (T5 semantics, loc_trust.TrustConfig thresholds) and pose disagreement against the recording.
"""
import argparse
import bisect
import hashlib
import json
import math
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

if os.environ.get('QAZBOT_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires the isolated SIM environment (source twin/env_sim.sh)')
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'digital_twin/ws/src/x2_twin_scenarios'))
from x2_twin_scenarios.analysis.replay_inputs import read_replay_inputs, shift_input_stamps
sys.path.insert(0, str(root / 'ws/src/navigation'))
from loc_trust import TrustConfig  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('name')
ap.add_argument('--max-seconds', type=float, default=0.0)
args = ap.parse_args()
bag = Path(args.bag)
out = root / 'tests/integration_results' / args.name
if out.exists():
    raise RuntimeError('result directory exists')
out.mkdir()
ns = 'qazbot_replay'
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):                                      # a (+) b, poses (x, y, yaw)
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


# ---------------------------------------------------------------- read the recording (read-only)
db = sqlite3.connect(next(bag.glob('*.db3')).resolve().as_uri() + '?mode=ro', uri=True)
tid = {name: i for i, name in db.execute('select id, name from topics')}
tf_topic, scan_topic = '/qazbot_d1_map/tf', '/qazbot_lio/scan'
status_topic = '/qazbot_d1_map/localization_trust/status'
last_status = json.loads(deserialize_message(bytes(db.execute(
    'select data from messages where topic_id=? order by timestamp desc limit 1', (tid[status_topic],)).fetchone()[0]), String).data)
map_dir = next(d for d in (root / 'ws/src/maps').iterdir()
               if (d / 'manifest.json').exists() and json.load(open(d / 'manifest.json'))['pgm_sha256'] == last_status['map_sha256'])
msgs, odom_base, rec_map_odom = read_replay_inputs(
    db, deserialize_message,
    {'tf': TFMessage, 'scan': LaserScan, 'initialpose': PoseWithCovarianceStamped}, args.max_seconds)
t0, t_end = msgs[0][0], msgs[-1][0]
ob_t = [t for t, _ in odom_base]


def odom_at(t):                                         # nearest recorded odom->base sample (<= 20 ms at the recorded rate)
    i = min(range(max(bisect.bisect_left(ob_t, t) - 1, 0), min(bisect.bisect_left(ob_t, t) + 1, len(ob_t))),
            key=lambda k: abs(ob_t[k] - t))
    return odom_base[i][1]


# ---------------------------------------------------------------- run
procs, logs = [], []
rclpy.init()
n = Node('bag_replayer', namespace=ns)
tf_pub = n.create_publisher(TFMessage, f'/{ns}/tf', QoSProfile(depth=100))
scan_pub = n.create_publisher(LaserScan, f'/{ns}/scan', qos_profile_sensor_data)
init_pub = n.create_publisher(PoseWithCovarianceStamped, f'/{ns}/initialpose', 1)
amcl_state = n.create_client(GetState, f'/{ns}/amcl/get_state')
replay_map_odom = []                                    # (receive time s since play start, pose)


def on_tf(m):
    for t in m.transforms:
        if (t.header.frame_id, t.child_frame_id) == ('map', 'odom'):
            tr = t.transform.translation
            p = (tr.x, tr.y, yaw_of(t.transform.rotation))
            if not replay_map_odom or replay_map_odom[-1][1] != p:
                replay_map_odom.append((time.monotonic(), p))


n.create_subscription(TFMessage, f'/{ns}/tf', on_tf, QoSProfile(depth=100))


def spin(s):
    end = time.monotonic() + s
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.01)


def start(name, cmd):
    f = (out / (name + '.log')).open('w')
    logs.append(f)
    procs.append((name, subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)))


result = {}
try:
    start('amcl', ['ros2', 'launch', str(root / 'tools/twin/amcl_sim.launch.py'), f'ns:={ns}', f'scan_topic:=/{ns}/scan',
                   f'map_yaml:={map_dir / "map.yaml"}'])
    end = time.monotonic() + 60
    while not amcl_state.service_is_ready():
        if time.monotonic() > end:
            raise RuntimeError('amcl service not discovered')
        spin(.1)
    while True:
        f = amcl_state.call_async(GetState.Request())
        while not f.done():
            spin(.02)
        if f.result().current_state.id == 3:
            break
        spin(.5)
    spin(1.0)
    # Align receive times, retaining source-stamp age (including delayed seeds/scans).
    now = n.get_clock().now().nanoseconds
    delta = now + 500_000_000 - round(t0 * 1e9)
    play0 = time.monotonic() + 0.5
    publishers = {'tf': tf_pub, 'scan': scan_pub, 'initialpose': init_pub}
    for ts, kind, m in msgs:
        while time.monotonic() < play0 + (ts - t0):
            rclpy.spin_once(n, timeout_sec=.002)
        shift_input_stamps(kind, m, delta, Time)
        publishers[kind].publish(m)
    spin(3.0)
    result['status'] = 'completed'
except BaseException as e:                              # noqa: BLE001 - recorded, then re-raised after cleanup
    result['status'] = 'error: ' + repr(e)
    raise
finally:
    for _, p in procs:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    codes = {}
    for name, p in procs:
        try:
            codes[name] = p.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            codes[name] = p.wait(timeout=5)
    for f in logs:
        f.close()
    cfg = TrustConfig()

    def steps(seq):
        out_ = []
        for (ta, a), (tb, b) in zip(seq, seq[1:]):
            out_.append((tb, math.hypot(b[0] - a[0], b[1] - a[1]), abs(wrap(b[2] - a[2]))))
        return out_

    def summary(seq):
        st = steps(seq)
        trips = [s for s in st if s[1] > cfg.jump_xy_max or s[2] > cfg.jump_yaw_max]
        xs = sorted(s[1] for s in st) or [0.0]
        return dict(updates=len(seq), max_step_xy=max([s[1] for s in st], default=0.0), max_step_yaw=max([s[2] for s in st], default=0.0),
                    p95_step_xy=xs[int(len(xs) * .95)], t5_trips=len(trips), first_trip_s=(round(trips[0][0], 2) if trips else None))

    rec = [(t - t0, p) for t, p in rec_map_odom if t <= t_end]
    rep = [(t - play0, p) for t, p in replay_map_odom if t >= play0]          # play-start-relative == bag-relative
    # pose disagreement with the recording: map pose = map->odom (+) odom->base at the same bag time
    diffs = []
    for t, p in rep:
        k = min(range(len(rec)), key=lambda k: abs(rec[k][0] - t)) if rec else None
        ref = compose(rec[k][1], odom_at(rec[k][0] + t0)) if k is not None else None
        # compare at the recorded sample's own bag time to avoid mixing motion between the two instants
        mine_at = compose(p, odom_at(rec[k][0] + t0)) if k is not None else None
        if ref is not None:
            diffs.append((math.hypot(mine_at[0] - ref[0], mine_at[1] - ref[1]), abs(wrap(mine_at[2] - ref[2]))))
    dd = sorted(d[0] for d in diffs) or [0.0]
    dy = sorted(d[1] for d in diffs) or [0.0]
    result.update(bag=str(bag.name), map=map_dir.name, initialpose_count=sum(kind == 'initialpose' for _, kind, _ in msgs), duration_s=round(t_end - t0, 1),
                  thresholds=dict(jump_xy_max=cfg.jump_xy_max, jump_yaw_max=cfg.jump_yaw_max),
                  recorded=summary(rec), replay=summary(rep),
                  pose_vs_recording=dict(n=len(diffs), xy_median=dd[len(dd) // 2], xy_p95=dd[int(len(dd) * .95)], xy_max=dd[-1],
                                         yaw_median=dy[len(dy) // 2], yaw_p95=dy[int(len(dy) * .95)], yaw_max=dy[-1]),
                  shutdown=codes, scope='recorded scan + odom->base_footprint -> REAL AMCL (production parameters); '
                                        'recorded AMCL output used only as reference; domain 87')
    (out / 'results.json').write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, indent=1, default=str))
    n.destroy_node()
    rclpy.try_shutdown()
