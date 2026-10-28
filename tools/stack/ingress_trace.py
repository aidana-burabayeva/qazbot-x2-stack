#!/usr/bin/env python3
"""P2-T passive ingress observer (subscribe only: no publishers, no services, no params changed).
Records per message (topic, header.stamp ns, CLOCK_MONOTONIC ns at callback enter, CLOCK_REALTIME ns) for
the live FAST-LIO -> level_bridge -> Trust path, plus every live Trust status (reasons, input_ages,
input_callbacks.received = Trust's own CLOCK_MONOTONIC at its input-callback enter), and /proc schedstat
deltas of the publisher/consumer processes. Raw (undeserialised) subscriptions are used where only the
stamp is needed, to keep observer overhead low. Writes <out>/ingress_<tag>.csv, status_<tag>.jsonl,
sched_<tag>.json, meta_<tag>.json.
Usage: ingress_trace.py --duration 90 --out DIR --tag A [--pids name=pid ...] [--copy-ns /qazbot_trace] [--map-check]
"""
import argparse, glob, json, os, struct, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument('--duration', type=float, default=90)
ap.add_argument('--out', required=True)
ap.add_argument('--tag', required=True)
ap.add_argument('--pids', nargs='*', default=[])
ap.add_argument('--copy-ns', default='')
ap.add_argument('--map-check', action='store_true')
ap.add_argument('--no-raw-lidar', action='store_true')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

def sched(pid):
    """sum over threads of (run_ns, wait_ns, timeslices) from /proc/<pid>/task/*/schedstat; plus per-thread max wait"""
    tot = [0, 0, 0]; per = {}
    for f in glob.glob(f'/proc/{pid}/task/*/schedstat'):
        try:
            r, w, n = map(int, open(f).read().split())
            comm = open(f.replace('schedstat', 'comm')).read().strip()
        except Exception:
            continue
        tot = [tot[0] + r, tot[1] + w, tot[2] + n]
        per[f.split('/')[4] + ':' + comm] = (r, w, n)
    return dict(total=tot, threads=per)

def psi():
    try:
        return open('/proc/pressure/cpu').read()
    except Exception:
        return None

pids = dict(p.split('=') for p in a.pids)
pids['observer'] = str(os.getpid())
s0 = {k: sched(v) for k, v in pids.items()}; psi0 = psi(); load0 = open('/proc/loadavg').read()

rclpy.init()
node = rclpy.create_node('qazbot_ingress_trace_ro')
rows = []          # (topic, stamp_ns, mono_ns, real_ns, extra)
status = []
mapinfo = {}

def stamp_raw(b):
    sec, nsec = struct.unpack_from('<iI', b, 4)
    return sec * 1_000_000_000 + nsec

def raw_cb(topic):
    def cb(b):
        m = time.monotonic_ns(); r = time.time_ns()
        rows.append((topic, stamp_raw(b), m, r, ''))
    return cb

def tf_cb(msg):
    m = time.monotonic_ns(); r = time.time_ns()
    for t in msg.transforms:
        rows.append(('tf:' + t.header.frame_id + '>' + t.child_frame_id,
                     t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec, m, r, ''))

def status_cb(src):
    def cb(msg):
        m = time.monotonic_ns(); r = time.time_ns()
        try:
            j = json.loads(msg.data)
        except Exception:
            return
        ic = j.get('input_callbacks', {}) or {}
        status.append(dict(src=src, mono_ns=m, real_ns=r, trusted=j.get('trusted'), ready=j.get('ready'),
                           motion_ok=j.get('motion_ok'), reasons=j.get('reasons'), input_ages=j.get('input_ages'),
                           stamp_rejections=j.get('stamp_rejections'), map_ok=j.get('map_ok'),
                           initialized=j.get('initialized'), map_sha256=j.get('map_sha256'),
                           cb={k: dict(count=v.get('count'), received=v.get('received'), dur=v.get('duration_s'),
                                       maxdur=v.get('max_duration_s')) for k, v in ic.items()}))
    return cb

def map_cb(msg):
    import hashlib
    o = msg.info.origin.position
    mapinfo.update(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, frame=msg.header.frame_id,
                   res=msg.info.resolution, w=msg.info.width, h=msg.info.height, origin=[o.x, o.y, o.z],
                   data_sha256=hashlib.sha256(bytes((x & 0xff) for x in msg.data)).hexdigest())
    try:
        sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
        import loc_trust as L
        g = L.OccGrid.from_pgm('/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml')
        mapinfo['identity_vs_v3_pgm'] = L.map_identity(g, msg.info.resolution, (o.x, o.y), msg.info.width,
                                                       msg.info.height, msg.data) or 'IDENTICAL'
    except Exception as e:
        mapinfo['identity_err'] = repr(e)

be1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)       # == trust scan/odom_level QoS
be5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
subs = []
if not a.no_raw_lidar:
    subs.append(node.create_subscription(PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
                                         raw_cb('raw_lidar'), be5, raw=True))
subs.append(node.create_subscription(PointCloud2, '/qazbot_lio/cloud_registered', raw_cb('lio_cloud'), be5, raw=True))
subs.append(node.create_subscription(Odometry, '/qazbot_lio/odometry', raw_cb('lio_odom'), be5, raw=True))
subs.append(node.create_subscription(LaserScan, '/qazbot_lio/scan', raw_cb('scan'), be1, raw=True))
subs.append(node.create_subscription(Odometry, '/qazbot_lio/odom_level', raw_cb('odom_level'), be1, raw=True))
subs.append(node.create_subscription(TFMessage, '/qazbot_d1_map/tf', tf_cb, QoSProfile(depth=50)))   # == trust TF QoS
rel10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
subs.append(node.create_subscription(String, '/qazbot_d1_map/localization_trust/status', status_cb('live'), rel10))
if a.copy_ns:
    subs.append(node.create_subscription(String, a.copy_ns + '/localization_trust/status', status_cb('copy'), rel10))
if a.map_check:
    subs.append(node.create_subscription(OccupancyGrid, '/qazbot_d1_map/map', map_cb,
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)))
meta = dict(tag=a.tag, start_real=time.time(), start_mono_ns=time.monotonic_ns(), duration=a.duration,
            ros_minus_real_ns=node.get_clock().now().nanoseconds - time.time_ns(),
            use_sim_time=node.get_parameter('use_sim_time').value, pids=pids, load0=load0, psi0=psi0)
t_end = time.monotonic() + a.duration
while time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.05)
s1 = {k: sched(v) for k, v in pids.items()}
meta.update(load1=open('/proc/loadavg').read(), psi1=psi(), end_real=time.time(), map=mapinfo)
dt = (meta['end_real'] - meta['start_real']) * 1e9
sch = {}
for k in pids:
    t0, t1 = s0[k]['total'], s1[k]['total']
    th = {}
    for name, v in s1[k]['threads'].items():
        u = s0[k]['threads'].get(name, (0, 0, 0))
        dr, dw, dn = v[0] - u[0], v[1] - u[1], v[2] - u[2]
        if dr > 0.002 * dt:
            th[name] = dict(cpu_pct=round(100 * dr / dt, 1), rq_wait_pct=round(100 * dw / dt, 1),
                            avg_wait_ms=round(dw / max(dn, 1) / 1e6, 3), slices=dn)
    sch[k] = dict(cpu_pct=round(100 * (t1[0] - t0[0]) / dt, 1), rq_wait_pct=round(100 * (t1[1] - t0[1]) / dt, 1),
                  avg_wait_ms_per_slice=round((t1[1] - t0[1]) / max(t1[2] - t0[2], 1) / 1e6, 3), threads=th)
json.dump(sch, open(f'{a.out}/sched_{a.tag}.json', 'w'), indent=1)
json.dump(meta, open(f'{a.out}/meta_{a.tag}.json', 'w'), indent=1)
with open(f'{a.out}/ingress_{a.tag}.csv', 'w') as f:
    f.write('topic,stamp_ns,mono_ns,real_ns\n')
    for r in rows:
        f.write(f'{r[0]},{r[1]},{r[2]},{r[3]}\n')
with open(f'{a.out}/status_{a.tag}.jsonl', 'w') as f:
    for s in status:
        f.write(json.dumps(s) + '\n')
print(json.dumps(dict(rows=len(rows), status=len(status), map=mapinfo,
                      sched={k: {x: v[x] for x in ('cpu_pct', 'rq_wait_pct', 'avg_wait_ms_per_slice')} for k, v in sch.items()})))
node.destroy_node(); rclpy.try_shutdown()
