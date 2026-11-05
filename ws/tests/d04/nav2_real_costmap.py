#!/usr/bin/env python3
"""D-04 check on the REAL Humble nav2_costmap_2d (standalone costmap node), isolated ROS domain 77.

    source /opt/ros/humble/setup.bash
    nice -n 19 python3 <repo>/tests/d04/nav2_real_costmap.py [--out results/nav2_real.json]

Flow
1. Offline: first ~N s of bag d04_static_bag_204153 -> frames (fitted levelling, same code as replay_d04),
   synthetic objects injected exactly like replay_d04 scenarios; baseline A (ws/src/lio/level_bridge.py)
   and candidate C produce scan_obstacles / scan_clearing PointCloud2 (real on_cloud code).
2. Per (scenario, variant) a fresh /opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d is started
   (use_sim_time, live local_costmap params from the live params dump, node /costmap/costmap),
   configured + activated through its lifecycle services. This script replays /clock, TF odom->base_footprint
   (the bag's /qazbot_d1_map/tf: the same native-odom TF the live costmap uses, interpolated by tf2) and the two clouds with the bag stamps, and records costmap_raw.
3. Metrics are computed on lethal (254) cells exactly like the emulator (edge_lethal, region) and compared
   with the emulator on the same frames.
Never uses domain 0 (asserted). Local-host only DDS.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '77'
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
assert os.environ['ROS_DOMAIN_ID'] != '0'

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_d04 as R  # noqa: E402

NAV2_BIN = '/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d'
LIVE_PARAMS = '/home/agi/.ros/log/qazbot_ctl_t9htg14z.yaml'
LIVE_SECTION = '/qazbot_d1_map/nav2/local_costmap/local_costmap'
BAG = 'static_204153'
NODE = 'costmap/costmap'     # standalone binary: Costmap2DROS('costmap') -> /costmap/costmap
RAW_TOPIC = '/costmap/costmap_raw'
LETHAL = 254
SCEN = ['recorded_only', 'S9_D04_right_-52deg_0.5m_transient', 'S10_D04_left_+48deg_0.55m_transient',
        'S6_side_chair_+49deg_0.8m_static']


def load_params():
    import yaml
    if os.path.exists(LIVE_PARAMS):
        sec = yaml.safe_load(open(LIVE_PARAMS))[LIVE_SECTION]['ros__parameters']
    else:                                                       # copy of the section kept next to the script
        sec = yaml.safe_load(open(os.path.join(HERE, 'local_costmap_live_params.yaml')))['ros__parameters']
    sec = dict(sec)
    sec['use_sim_time'] = True
    return sec


def _ns(st):
    return st.sec * 10**9 + st.nanosec


def tf_pose_at(tfs, ns):
    """(x, y, yaw) of the latest odom->base_footprint sample with stamp <= ns (the pose the costmap sees)."""
    c = [t for t in tfs if _ns(t.header.stamp) <= ns] or tfs[:1]
    t = c[-1].transform
    q = t.rotation
    return (t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def limited_reader(n_cloud_max, tf_store):
    """Drop-in for R.read_bag that stops after n_cloud_max clouds and stashes /qazbot_lio/tf."""
    orig = R.read_bag

    def rb(db, topics):
        cl = R.BAGS[BAG]['cloud']
        n = 0
        for t, ts, m in orig(db, list(topics) + ['/qazbot_d1_map/tf']):
            if t == '/qazbot_d1_map/tf':
                for tr in m.transforms:
                    if tr.child_frame_id == 'base_footprint' and tr.header.frame_id == 'odom':
                        tf_store.append(tr)
                continue
            yield t, ts, m
            if t == cl:
                n += 1
                if n >= n_cloud_max:
                    return
    return rb


def build_inputs(nf, n_cloud_max):
    tf_store = []
    R.read_bag = limited_reader(n_cloud_max, tf_store)
    frames, lsrc, A = R.prepare(BAG, 'fit')
    frames = frames[:nf]
    assert len(frames) == nf, f'only {len(frames)} frames'
    fz = R.floor_z(frames)
    sc = R.scenarios(nf)
    C = R.load_bridge(R.CANDIDATE, 'lb_candidate_nav2')
    A = R.load_bridge(R.RUNNING, 'lb_running_nav2')
    tf_store.sort(key=lambda t: _ns(t.header.stamp))
    return frames, fz, sc, A, C, tf_store


def offline_clouds(frames, fz, spec, mod):
    """[(obstacles_msg, clearing_msg)] per frame, real on_cloud of `mod`."""
    evs = spec[0]
    sv = R.SplitVariant(mod, frames[0]['R_lv'])
    out = []
    for k, fr in enumerate(frames):
        boxes = [b for (f0, f1, b) in evs if f0 <= k < f1]
        msg = fr['msg']
        if boxes:
            Qi, _ = R.inject(R.level_points(fr), fr['pose'][2], boxes, fz)
            msg = R.to_msg(fr, Qi)
        sv.observations(fr, msg)
        out.append((sv.s.pub_obstacles.msg, sv.s.pub_clearing.msg))
    return out


def costmap_cells(m):
    """nav2_msgs/Costmap -> (world xy of lethal cells, (res, size))."""
    md = m.metadata
    g = np.asarray(m.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
    iy, ix = np.nonzero(g == LETHAL)
    res = md.resolution
    xy = np.column_stack((md.origin.position.x + (ix + 0.5) * res, md.origin.position.y + (iy + 0.5) * res))
    return xy, g


def metrics_from_xy(xy, pose, evs):
    b, r = R.bearing_range(xy, pose) if len(xy) else (np.empty(0), np.empty(0))
    edge = int((((b > R.OLD_FOV[1]) | (b < R.OLD_FOV[0])) & (np.abs(b) <= 90) & (r < 1.5)).sum())
    reg = []
    for (_, _, bx) in evs:
        cx, cy, hx, hy, _, _ = bx
        x, y, th = pose
        c, s = math.cos(-th), math.sin(-th)
        bxx = c * (xy[:, 0] - x) - s * (xy[:, 1] - y)
        byy = s * (xy[:, 0] - x) + c * (xy[:, 1] - y)
        reg.append(int(((np.abs(bxx - cx) <= hx + 0.05) & (np.abs(byy - cy) <= hy + 0.05)).sum()) if len(xy) else 0)
    return int(len(xy)), edge, reg


def run_nav2(params, frames, clouds, tfs, evs, tag, frame_wall_s=0.7, log_dir=None):
    """One fresh standalone costmap; returns per-published-costmap records."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import PointCloud2
    from nav2_msgs.msg import Costmap
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from builtin_interfaces.msg import Time

    pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    import yaml
    yaml.safe_dump({'/**': {'ros__parameters': params}}, pf)
    pf.close()
    logf = open(os.path.join(log_dir or tempfile.gettempdir(), f'nav2_{tag}.log'), 'w')
    proc = subprocess.Popen(
        ['nice', '-n', '19', NAV2_BIN, '--ros-args', '--params-file', pf.name],
        stdout=logf, stderr=subprocess.STDOUT, env=dict(os.environ), preexec_fn=os.setsid)
    rclpy.init()
    node = Node('d04_replayer', parameter_overrides=[])
    recs = []
    try:
        qos_rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        p_clock = node.create_publisher(Clock, '/clock', 10)
        p_tf = node.create_publisher(TFMessage, '/tf', qos_rel)
        p_ob = node.create_publisher(PointCloud2, '/qazbot_lio/scan_obstacles', qos_rel)
        p_cl = node.create_publisher(PointCloud2, '/qazbot_lio/scan_clearing', qos_rel)
        state = {'k': -1, 'phase': 'pre'}

        def on_raw(m):
            recs.append(dict(k=state['k'], phase=state['phase'], msg=m))
        node.create_subscription(Costmap, RAW_TOPIC, on_raw, 10)

        def spin(sec):
            end = time.time() + sec
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        def stamp(k):
            return frames[k]['msg'].header.stamp

        def clock(t):
            c = Clock()
            c.clock = t
            p_clock.publish(c)

        sent = {'i': 0}
        stamps = [_ns(t.header.stamp) for t in tfs]

        def tf_resend(k, ahead_ns=60_000_000):
            """warm-up only (before the costmap's TF listener has discovered us): resend all samples"""
            lim = _ns(stamp(k)) + ahead_ns
            for i, t in enumerate(tfs):
                if stamps[i] <= lim:
                    p_tf.publish(TFMessage(transforms=[t]))

        def tf(k, ahead_ns=60_000_000):
            """publish every bag TF sample with stamp <= frame stamp + 60 ms (brackets the cloud stamp)"""
            lim = _ns(stamp(k)) + ahead_ns
            while sent['i'] < len(tfs) and stamps[sent['i']] <= lim:
                p_tf.publish(TFMessage(transforms=[tfs[sent['i']]]))
                sent['i'] += 1

        # wait for the node, then give it clock+TF *before* configuring
        cli = node.create_client(ChangeState, f'/{NODE}/change_state')
        assert cli.wait_for_service(timeout_sec=40), 'costmap lifecycle service not up'
        for _ in range(10):
            clock(stamp(0)); tf_resend(0); spin(0.1)

        def change(tid):
            f = cli.call_async(ChangeState.Request(transition=Transition(id=tid)))
            end = time.time() + 60
            while not f.done() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                clock(stamp(0)); tf_resend(0)
            return f.done() and f.result().success

        assert change(Transition.TRANSITION_CONFIGURE), 'configure failed'
        assert change(Transition.TRANSITION_ACTIVATE), 'activate failed'
        # discovery of subscribers
        end = time.time() + 20
        while time.time() < end and (p_ob.get_subscription_count() < 1 or p_cl.get_subscription_count() < 1):
            clock(stamp(0)); tf_resend(0); spin(0.1)
        assert p_ob.get_subscription_count() >= 1 and p_cl.get_subscription_count() >= 1, 'costmap not subscribed'
        spin(1.0)
        state['phase'] = 'run'
        for k in range(len(frames)):
            state['k'] = k
            clock(stamp(k)); tf(k); spin(0.05)
            ob, cl = clouds[k]
            p_cl.publish(cl); p_ob.publish(ob)           # clearing first, then marks
            spin(frame_wall_s)
        # settle: advance sim clock so the publish timer fires for the final state
        state['phase'] = 'settle'
        last = stamp(len(frames) - 1)
        for i in range(1, 4):
            t = Time(sec=last.sec + i, nanosec=last.nanosec)
            clock(t); spin(0.8)
        out = []
        for r in recs:
            xy, g = costmap_cells(r['msg'])
            out.append(dict(k=r['k'], phase=r['phase'], xy=xy, n_cells=int(g.size),
                            md=(r['msg'].metadata.size_x, r['msg'].metadata.size_y,
                                round(r['msg'].metadata.resolution, 4))))
        return out
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=15)
        except Exception:                                  # noqa: BLE001
            os.killpg(proc.pid, signal.SIGKILL)
        os.unlink(pf.name)


def summarize_run(recs, frames, evs, tfs):
    """Per costmap_raw publication: lethal total, edge lethal, region lethal at the pose of frame k."""
    rows = []
    for r in recs:
        k = max(r['k'], 0)
        pose = tf_pose_at(tfs, _ns(frames[k]['msg'].header.stamp))
        n, edge, reg = metrics_from_xy(r['xy'], pose, evs)
        rows.append(dict(k=r['k'], phase=r['phase'], lethal=n, edge_lethal=edge, region=reg, md=r['md']))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'results', 'nav2_real.json'))
    ap.add_argument('--nf', type=int, default=40)
    ap.add_argument('--clouds', type=int, default=170, help='bag clouds to read (stops early)')
    ap.add_argument('--scenarios', nargs='*', default=SCEN)
    ap.add_argument('--frame-wall-s', type=float, default=0.7)
    ap.add_argument('--variants', nargs='*', default=['A', 'C'])
    a = ap.parse_args()
    why = R.bag_missing_reason(BAG)
    if why:
        sys.exit('ERROR: ' + why)
    params = load_params()
    frames, fz, sc, A, C, tf_store = build_inputs(a.nf, a.clouds)
    mods = {'A': A, 'C': C}
    assert len(tf_store) > 20, 'no odom->base_footprint TF in the bag window'
    xy0 = np.array([[t.transform.translation.x, t.transform.translation.y] for t in tf_store])
    dpos = float(np.ptp(xy0, axis=0).max())
    res = dict(meta=dict(bag=BAG, nf=a.nf, params=params, domain=DOMAIN, floor_z=fz,
                         bag_tf_samples=len(tf_store), bag_tf_xy_extent_m=dpos,
                         running_sha256=__import__('hashlib').sha256(open(R.RUNNING, 'rb').read()).hexdigest(),
                         candidate_sha256=__import__('hashlib').sha256(open(R.CANDIDATE, 'rb').read()).hexdigest()),
               scenarios={})
    logd = tempfile.gettempdir()       # costmap node logs (noisy DDS warnings) stay out of the repo
    for name in a.scenarios:
        spec = sc[name]
        evs = spec[0]
        emu = R.run_scenario(frames, fz, name, spec, A, C, variants=tuple(a.variants), collide=False)
        res['scenarios'][name] = dict(events=[(f0, f1) for f0, f1, _ in evs], emulator={}, nav2={})
        off = max((f1 for _, f1, _ in evs), default=0)
        for v in a.variants:
            o = emu['out'][v]
            res['scenarios'][name]['emulator'][v] = dict(
                edge_lethal_final=o['edge_lethal'][-1], edge_lethal_max=max(o['edge_lethal']),
                lethal_final=o['lethal'][-1], region_final=o['region'][-1] if evs else [],
                region_peak=[max(r[i] for r in o['region']) for i in range(len(evs))] if evs else [])
            clouds = offline_clouds(frames, fz, spec, mods[v])
            t0 = time.time()
            recs = run_nav2(params, frames, clouds, tf_store, evs, f'{name}_{v}', a.frame_wall_s, logd)
            rows = summarize_run(recs, frames, evs, tf_store)
            fin = [r for r in rows if r['phase'] == 'settle']
            run_rows = [r for r in rows if r['phase'] == 'run']
            during = [r for r in run_rows if evs and evs[0][0] + 2 <= r['k'] < off]
            res['scenarios'][name]['nav2'][v] = dict(
                n_publications=len(rows), wall_s=round(time.time() - t0, 1),
                edge_lethal_final=fin[-1]['edge_lethal'] if fin else None,
                edge_lethal_max=max((r['edge_lethal'] for r in rows), default=None),
                lethal_final=fin[-1]['lethal'] if fin else None,
                region_final=fin[-1]['region'] if fin else None,
                region_peak=[max((r['region'][i] for r in rows), default=0) for i in range(len(evs))],
                region_during_presence_max=[max((r['region'][i] for r in during), default=0)
                                            for i in range(len(evs))],
                series=[(r['k'], r['phase'], r['lethal'], r['edge_lethal'], r['region']) for r in rows])
            print(name, v, json.dumps({k: res['scenarios'][name]['nav2'][v][k] for k in
                                       ('edge_lethal_final', 'edge_lethal_max', 'lethal_final', 'region_final',
                                        'region_peak', 'n_publications')}), flush=True)
    json.dump(res, open(a.out, 'w'), indent=1)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
