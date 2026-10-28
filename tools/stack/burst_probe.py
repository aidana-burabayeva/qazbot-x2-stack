#!/usr/bin/env python3
"""P2T-b burst localiser (passive: subscribe only, /proc reads only; no publishers/services/params).

Mode `observe`: subscribes the upstream chain and records, per message, header.stamp, the DDS
SOURCE timestamp (DataWriter::write time inside the publishing process, CLOCK_REALTIME), the DDS
RECEIVED timestamp (observer's DDS receive thread) and CLOCK_MONOTONIC/REALTIME at executor take.
Source timestamps make per-hop publish latency independent of the observer's own scheduling:
  HAL       raw_lidar.src - raw_lidar.stamp           (stamp = scan start; nominal ~0.100 s)
  FAST-LIO  lio_odom.src  - raw_lidar.src  (lio stamp = raw stamp + 0.0952 s)
  bridge    odom_level.src - lio_odom.src  (same stamp); scan.src - lio_cloud.src
Also records live trust status (reasons, input_ages, input_callbacks.received = trust CLOCK_MONOTONIC).
Mode `sched`: samples every --period s /proc/<pid>/task/*/{schedstat,status} (run ns, runqueue-wait ns,
timeslices, voluntary/nonvoluntary ctxt switches) for the named pids plus /proc/stat (per-cpu idle,
procs_running). Run it as a SEPARATE process so a stalled observer does not stall the sampler.
Mode `join`: controlled DDS-discovery trigger for the hypothesis "a new participant joining domain 0 stalls
the chain". Creates ONE short-lived participant (rclpy node, rosout disabled, no parameter services) that publishes
nothing on robot topics; variant `bare` = no subscriptions, `sub` = BEST_EFFORT raw subs to /qazbot_lio/odom_level,
/qazbot_lio/scan, /qazbot_lio/odometry (endpoint matching with bridge/FAST-LIO writers). Logs its own timeline.
Usage:
  burst_probe.py join --variant bare --duration 8 --out DIR --tag X1j1
  burst_probe.py observe --duration 290 --out DIR --tag P1
  burst_probe.py sched   --duration 300 --out DIR --tag P1 --period 0.1 --pids fastlio=86463 bridge=86488 ...
"""
import argparse, glob, json, os, struct, sys, time

ap = argparse.ArgumentParser()
ap.add_argument('mode', choices=['observe', 'sched', 'join'])
ap.add_argument('--variant', default='bare', choices=['bare', 'sub'])
ap.add_argument('--duration', type=float, default=290)
ap.add_argument('--out', required=True)
ap.add_argument('--tag', required=True)
ap.add_argument('--period', type=float, default=0.1)
ap.add_argument('--pids', nargs='*', default=[])
ap.add_argument('--no-imu', action='store_true')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)


def run_sched():
    pids = dict(p.split('=') for p in a.pids)
    pids['sampler'] = str(os.getpid())
    names = {}
    fds = {}
    rescan = 0
    fstat = os.open('/proc/stat', os.O_RDONLY)
    f = open(f'{a.out}/bsched_{a.tag}.jsonl', 'w')
    t_end = time.monotonic() + a.duration
    nxt = time.monotonic()
    while time.monotonic() < t_end:
        m = time.monotonic_ns()
        rec = dict(m=m, r=time.time_ns())
        try:
            st = os.pread(fstat, 65536, 0).decode().splitlines()
            rec['cpu'] = [list(map(int, ln.split()[1:8])) for ln in st if ln.startswith('cpu') and ln[3] != ' ']
            rec['procs_running'] = int([ln for ln in st if ln.startswith('procs_running')][0].split()[1])
        except Exception:
            pass
        th = {}
        if m >= rescan:                       # refresh thread list every 5 s; keep fds open (pread is cheap)
            rescan = m + 5_000_000_000
            for k, pid in pids.items():
                for d in glob.glob(f'/proc/{pid}/task/*'):
                    tid = d.rsplit('/', 1)[1]
                    if tid in fds:
                        continue
                    try:
                        fds[tid] = (os.open(d + '/schedstat', os.O_RDONLY), os.open(d + '/status', os.O_RDONLY))
                        names[tid] = (k, open(d + '/comm').read().strip())
                    except Exception:
                        pass
        for tid, (fs, fst) in list(fds.items()):
            try:
                r, w, n = map(int, os.pread(fs, 128, 0).split())
                b = os.pread(fst, 4096, 0)
                i = b.find(b'voluntary_ctxt_switches:')
                j = b.find(b'nonvoluntary_ctxt_switches:')
                vol = int(b[i + 24:b.find(b'\n', i)])
                inv = int(b[j + 27:b.find(b'\n', j)])
                th[tid] = (r, w, n, vol, inv)
            except Exception:
                os.close(fs); os.close(fst); del fds[tid]
        rec['t'] = th
        f.write(json.dumps(rec, separators=(',', ':')) + '\n')
        nxt += a.period
        dt = nxt - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            nxt = time.monotonic()
    f.close()
    json.dump(dict(pids=pids, names=names, period=a.period), open(f'{a.out}/bsched_meta_{a.tag}.json', 'w'), indent=1)


def run_observe():
    import rclpy
    import rclpy.executors as E
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, PointCloud2, Imu
    from std_msgs.msg import String

    def _take(self, sub):          # keep rmw message_info (source/received timestamps)
        with sub.handle:
            mi = sub.handle.take_message(sub.msg_type, sub.raw)
            if mi is not None:
                return (mi[0], mi[1], time.monotonic_ns(), time.time_ns())
        return None
    E.Executor._take_subscription = _take

    rclpy.init()
    node = rclpy.create_node('qazbot_burst_probe_ro')
    rows, status = [], []

    def raw_cb(topic):
        def cb(t):
            b, info, m, r = t
            sec, nsec = struct.unpack_from('<iI', b, 4)
            rows.append((topic, sec * 1_000_000_000 + nsec, info['source_timestamp'], info['received_timestamp'], m, r))
        return cb

    def status_cb(t):
        msg, info, m, r = t
        try:
            j = json.loads(msg.data)
        except Exception:
            return
        ic = j.get('input_callbacks', {}) or {}
        status.append(dict(m=m, r=r, src=info['source_timestamp'], rcv=info['received_timestamp'],
                           reasons=j.get('reasons'), input_ages=j.get('input_ages'),
                           cb={k: dict(count=v.get('count'), received=v.get('received')) for k, v in ic.items()}))

    def be(d):
        return QoSProfile(depth=d, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = [
        node.create_subscription(PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', raw_cb('raw_lidar'), be(20), raw=True),
        node.create_subscription(Odometry, '/qazbot_lio/odometry', raw_cb('lio_odom'), be(20), raw=True),
        node.create_subscription(PointCloud2, '/qazbot_lio/cloud_registered', raw_cb('lio_cloud'), be(20), raw=True),
        node.create_subscription(LaserScan, '/qazbot_lio/scan', raw_cb('scan'), be(10), raw=True),
        node.create_subscription(Odometry, '/qazbot_lio/odom_level', raw_cb('odom_level'), be(10), raw=True),
        node.create_subscription(String, '/qazbot_d1_map/localization_trust/status', status_cb,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)),
    ]
    if not a.no_imu:
        subs.append(node.create_subscription(Imu, '/aima/hal/sensor/lidar_chest_front/imu', raw_cb('imu'), be(400), raw=True))
    ex = E.SingleThreadedExecutor()
    ex.add_node(node)
    meta = dict(tag=a.tag, pid=os.getpid(), start_real=time.time(), start_mono_ns=time.monotonic_ns(),
                real_minus_mono_ns=time.time_ns() - time.monotonic_ns(), duration=a.duration,
                load0=open('/proc/loadavg').read())
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        ex.spin_once(timeout_sec=0.05)
    meta.update(end_real=time.time(), load1=open('/proc/loadavg').read(),
                real_minus_mono_ns_end=time.time_ns() - time.monotonic_ns())
    json.dump(meta, open(f'{a.out}/bmeta_{a.tag}.json', 'w'), indent=1)
    with open(f'{a.out}/bingress_{a.tag}.csv', 'w') as f:
        f.write('topic,stamp_ns,src_ns,rcv_ns,take_mono_ns,take_real_ns\n')
        for r in rows:
            f.write(','.join(map(str, r)) + '\n')
    with open(f'{a.out}/bstatus_{a.tag}.jsonl', 'w') as f:
        for s in status:
            f.write(json.dumps(s) + '\n')
    print(json.dumps(dict(rows=len(rows), status=len(status))))
    node.destroy_node()
    rclpy.try_shutdown()


def run_join():
    t_a = time.time_ns()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    t_b = time.time_ns()
    rclpy.init()
    node = rclpy.create_node('qazbot_join_probe_ro', enable_rosout=False, start_parameter_services=False)
    t_c = time.time_ns()
    n = [0]
    if a.variant == 'sub':
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        cb = lambda m: n.__setitem__(0, n[0] + 1)
        node.create_subscription(Odometry, '/qazbot_lio/odom_level', cb, be, raw=True)
        node.create_subscription(LaserScan, '/qazbot_lio/scan', cb, be, raw=True)
        node.create_subscription(Odometry, '/qazbot_lio/odometry', cb, be, raw=True)
    t_d = time.time_ns()
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    t_e = time.time_ns()
    node.destroy_node(); rclpy.try_shutdown()
    t_f = time.time_ns()
    json.dump(dict(variant=a.variant, pid=os.getpid(), start_imports=t_a, imports_done=t_b, participant_node_created=t_c,
                   subs_created=t_d, spin_end=t_e, destroyed=t_f, msgs=n[0]), open(f'{a.out}/join_{a.tag}.json', 'w'))


{'sched': run_sched, 'observe': run_observe, 'join': run_join}[a.mode]()
