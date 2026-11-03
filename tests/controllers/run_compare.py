#!/usr/bin/env python3
"""Offline Nav2 controller comparison runner (RPP / DWB / MPPI), task 20261103-controller-compare.

Run via run.sh (sets the isolated environment: ROS_DOMAIN_ID=78, localhost, rmw_fastrtps_dynamic_cpp, overlays).
  run.sh matrix  [--controllers rpp,dwb,mppi] [--scenarios ...] [--models ideal,mc_hyp] [--seed 1]
  run.sh single  --controller rpp --scenario nominal --model ideal --seed 1 --out results/x.json

One single run = fresh launch of controller_server + velocity_smoother + lifecycle manager (launch/controller_harness
.launch.xml) + this process acting as robot sim (SimNode: odom, TF, synthetic LiDAR clouds) and FollowPath client.
All application logic (dynamics, ray casting, geometry) lives in sim_model.py.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_model as sm  # noqa: E402

MAP_YAML = os.path.join(HERE, '..', '..', 'ws', 'src', 'maps', 'd1_qazbot_20261029', 'map.yaml')
PLAN_JSON = os.path.join(HERE, 'data', 'live2_plan_183029.json')
SCENARIOS = ('nominal', 'static_block', 'transient', 'stale_cell')
RUN_TIMEOUT = {'nominal': 60.0, 'static_block': 50.0, 'transient': 60.0, 'stale_cell': 45.0}

# Scenario geometry (map frame, along the LIVE-2 path). Box 0.4 x 0.4 m ~ person/box, tall (visible to the LiDAR).
BLOCK_S = 3.0           # static box centred on the path at arc length 3.0 m
TRANSIENT_S = 2.4       # box centred on the path at s=2.4 m appears when the robot reaches s=1.0 m ...
TRANSIENT_TRIGGER = 1.0
TRANSIENT_HOLD = 6.0    # ... and disappears 6 s later
BOX_HALF = 0.2
# D-04b stale LETHAL cluster, body frame of the START pose: x 0.02..0.26, y 0.31..0.50 (left side, outside FOV/min range)
STALE_X = (0.02, 0.26)
STALE_Y = (0.31, 0.50)


# ----------------------------------------------------------------------------------------------- single run (ROS)
def single_run(a):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from nav2_msgs.action import FollowPath
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage

    plan = json.load(open(PLAN_JSON))
    pxy = [(p[0], p[1]) for p in plan['poses']]
    path = sm.PathRef(pxy)
    world = sm.World.from_map_yaml(MAP_YAML)
    rng = np.random.default_rng(a.seed)
    start_yaw = 2.0 * math.atan2(plan['poses'][0][2], plan['poses'][0][3])
    robot = sm.RobotModel(a.model, pxy[0][0], pxy[0][1], start_yaw)

    if a.scenario == 'static_block':
        (bx, by), _ = path.point_at(BLOCK_S)
        world.boxes['block'] = (bx, by, BOX_HALF, BOX_HALF)
    (tx, ty), _ = path.point_at(TRANSIENT_S)

    def cloud(header, pts):
        msg = PointCloud2(header=header, height=1, width=len(pts), is_bigendian=False, point_step=12,
                          row_step=12 * len(pts), is_dense=True,
                          fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                  for i, n in enumerate('xyz')])
        msg.data = np.asarray(pts, dtype=np.float32).reshape(-1, 3).tobytes()
        return msg

    class SimNode(Node):
        def __init__(self):
            super().__init__('cmp_sim')
            self.lock = threading.Lock()
            self.cmd = None
            self.cmd_stamp = -1e9
            self.t0 = time.monotonic()
            self.log_cmd, self.log_nav, self.log_pose = [], [], []
            self.events = []
            self.transient_state = 'pending'
            self.transient_t = None
            self.stale_burst = 0
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 50)
            static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)
            self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
            self.mark_pub = self.create_publisher(PointCloud2, '/scan_obstacles', 5)
            self.clear_pub = self.create_publisher(PointCloud2, '/scan_clearing', 5)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
            self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav, 20)
            self.ac = ActionClient(self, FollowPath, 'follow_path')
            self.publish_static()
            self.create_timer(0.02, self.on_sim)
            self.create_timer(0.1, self.on_lidar)

        def now(self):
            return time.monotonic() - self.t0

        def publish_static(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = 'map', 'odom'
            t.transform.rotation.w = 1.0
            self.tf_static_pub.publish(TFMessage(transforms=[t]))

        def on_cmd(self, m):
            with self.lock:
                self.cmd = (m.linear.x, m.angular.z)
                self.cmd_stamp = self.now()
                self.log_cmd.append((self.cmd_stamp, m.linear.x, m.angular.z))

        def on_nav(self, m):
            self.log_nav.append((self.now(), m.linear.x, m.angular.z))

        def on_sim(self):
            t = self.now()
            with self.lock:
                robot.step(t, 0.02, self.cmd, self.cmd_stamp)
            stamp = self.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_footprint'
            tf.transform.translation.x, tf.transform.translation.y = robot.x, robot.y
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(robot.yaw / 2), math.cos(robot.yaw / 2)
            self.tf_pub.publish(TFMessage(transforms=[tf]))
            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_footprint'
            od.pose.pose.position.x, od.pose.pose.position.y = robot.x, robot.y
            od.pose.pose.orientation = tf.transform.rotation
            od.twist.twist.linear.x, od.twist.twist.angular.z = robot.v, robot.w
            self.odom_pub.publish(od)
            if int(t / 0.02) % 5 == 0:
                self.log_pose.append((t, robot.x, robot.y, robot.yaw, robot.v, robot.w))
            # transient obstacle state machine
            if a.scenario == 'transient' and self.goal_sent_t is not None:
                _, s = path.project(robot.x, robot.y)
                if self.transient_state == 'pending' and s >= TRANSIENT_TRIGGER:
                    world.boxes['transient'] = (tx, ty, BOX_HALF, BOX_HALF)
                    self.transient_state, self.transient_t = 'present', t
                    self.events.append((t, 'transient_appear'))
                elif self.transient_state == 'present' and t - self.transient_t >= TRANSIENT_HOLD:
                    world.boxes.pop('transient', None)
                    self.transient_state, self.transient_t = 'gone', t
                    self.events.append((t, 'transient_disappear'))

        goal_sent_t = None

        def on_lidar(self):
            hdr = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_footprint')
            b, r, hit = world.raycast(robot.x, robot.y, robot.yaw, rng=rng, noise=0.01)
            z = 0.0   # level_bridge-like 2D points (per-source max_obstacle_height default 0.0 in Humble)
            mk = hit & (r >= sm.LIDAR_MIN_MARK) & (r <= sm.LIDAR_MAX_MARK)
            mpts = np.stack([r[mk] * np.cos(b[mk]), r[mk] * np.sin(b[mk]), np.full(mk.sum(), z)], 1)
            if self.stale_burst > 0:          # D-04b-like stale cluster: marked ONCE (5 msgs) from the start pose
                self.stale_burst -= 1
                gx, gy = np.meshgrid(np.arange(STALE_X[0], STALE_X[1] + 1e-9, 0.04),
                                     np.arange(STALE_Y[0], STALE_Y[1] + 1e-9, 0.04))
                c, s_ = math.cos(robot.yaw - self.stale_yaw), math.sin(robot.yaw - self.stale_yaw)
                # keep the cluster fixed in odom: express start-pose body points in the current body frame
                wx = self.stale_pose[0] + math.cos(self.stale_yaw) * gx - math.sin(self.stale_yaw) * gy
                wy = self.stale_pose[1] + math.sin(self.stale_yaw) * gx + math.cos(self.stale_yaw) * gy
                dx, dy = wx - robot.x, wy - robot.y
                bx = math.cos(robot.yaw) * dx + math.sin(robot.yaw) * dy
                by = -math.sin(robot.yaw) * dx + math.cos(robot.yaw) * dy
                spts = np.stack([bx.ravel(), by.ravel(), np.full(bx.size, z)], 1)
                mpts = np.concatenate([mpts, spts])
                del c, s_
            self.mark_pub.publish(cloud(hdr, mpts))
            # level_bridge.clearing_endpoints semantics: clearing FOV -46..+44.5 deg, ray ends at the return; a
            # return inside the blind range (< 0.45 m; raw sensor min ~0.69 m) gives NO ray at all (blocked, no data)
            ck = (b >= math.radians(sm.CLEAR_FOV_DEG[0])) & (b <= math.radians(sm.CLEAR_FOV_DEG[1])) & \
                ~(hit & (r < sm.LIDAR_MIN_MARK))
            cpts = np.stack([r[ck] * np.cos(b[ck]), r[ck] * np.sin(b[ck]), np.full(int(ck.sum()), z)], 1)
            self.clear_pub.publish(cloud(hdr, cpts))

    rclpy.init()
    node = SimNode()
    ex_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ex_thread.start()

    log_path = a.out.replace('.json', '.log')
    env = dict(os.environ)
    launch = subprocess.Popen(
        ['ros2', 'launch', os.path.join(HERE, 'launch', 'controller_harness.launch.xml'),
         'common_yaml:=' + os.path.join(HERE, 'config', 'common.yaml'),
         'controller_yaml:=' + os.path.join(HERE, 'config', a.controller + '.yaml'),
         'cs_log_level:=' + os.environ.get('CS_LOG_LEVEL', 'info')],
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT, env=env, start_new_session=True)
    res = {'controller': a.controller, 'scenario': a.scenario, 'model': a.model, 'seed': a.seed,
           'clear_fov_deg': list(sm.CLEAR_FOV_DEG)}
    try:
        if not node.ac.wait_for_server(timeout_sec=60.0):
            res['error'] = 'follow_path server not available'
            return finish(res, a, launch, node, rclpy)
        time.sleep(3.0)    # costmap: a few sensor updates at 5 Hz after activation
        cs_pid = find_child(launch.pid, 'controller_server')
        res['declared_params'] = dump_declared(a.controller)
        if a.scenario == 'stale_cell':
            node.stale_pose, node.stale_yaw = (robot.x, robot.y), robot.yaw
            node.stale_burst = 5
            time.sleep(1.5)
        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.frame_id = plan['frame']
        goal.path.header.stamp = node.get_clock().now().to_msg()
        for (x, y, qz, qw) in plan['poses']:
            ps = PoseStamped()
            ps.header = goal.path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            goal.path.poses.append(ps)
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        cpu0 = proc_cpu(cs_pid)
        n_nav0 = len(node.log_nav)
        t_send = node.now()
        node.goal_sent_t = t_send
        fut = node.ac.send_goal_async(goal)
        while not fut.done() and node.now() - t_send < 10.0:
            time.sleep(0.02)
        gh = fut.result() if fut.done() else None
        status, rss = None, []
        if gh is None or not gh.accepted:
            res['error'] = 'goal rejected'
        else:
            rf = gh.get_result_async()
            while not rf.done() and node.now() - t_send < RUN_TIMEOUT[a.scenario]:
                time.sleep(0.25)
                rss.append(proc_rss_kb(cs_pid))
            if rf.done():
                status = rf.result().status
            else:
                status = 'TIMEOUT'
                cf = gh.cancel_goal_async()
                t_c = time.monotonic()
                while not cf.done() and time.monotonic() - t_c < 5.0:
                    time.sleep(0.05)
        t_end = node.now()
        cpu1 = proc_cpu(cs_pid)
        res.update(metrics(node, path, world, a, t_send, t_end, status, n_nav0))
        if cpu0 is not None and cpu1 is not None:
            n_cycles = max(1, len(node.log_nav) - n_nav0)
            res['cs_cpu_pct'] = round(100.0 * (cpu1 - cpu0) / (t_end - t_send), 1)
            res['cs_cpu_ms_per_cycle'] = round(1000.0 * (cpu1 - cpu0) / n_cycles, 2)
        rss = [r for r in rss if r]
        res['cs_rss_mb_max'] = round(max(rss) / 1024.0, 1) if rss else None
        res['cs_vmhwm_mb'] = round((proc_status(cs_pid, 'VmHWM') or 0) / 1024.0, 1)
    except Exception as e:  # noqa: BLE001 - record and clean up
        res['error'] = repr(e)
    return finish(res, a, launch, node, rclpy)


def finish(res, a, launch, node, rclpy):
    try:
        os.killpg(launch.pid, signal.SIGINT)
        launch.wait(timeout=20)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(launch.pid, signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass
    log_path = a.out.replace('.json', '.log')
    res.update(log_counts(log_path))
    node.destroy_node()
    rclpy.try_shutdown()
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != 'declared_params' and not k.startswith('raw_')}))
    return 0


def find_child(ppid, name):
    for _ in range(50):
        for p in os.listdir('/proc'):
            if not p.isdigit():
                continue
            try:
                cmd = open(f'/proc/{p}/cmdline', 'rb').read().split(b'\0')
                stat = open(f'/proc/{p}/stat').read()
            except OSError:
                continue
            if cmd and cmd[0].endswith(name.encode()) and int(stat.rsplit(')', 1)[1].split()[1]) == ppid:
                return int(p)
        time.sleep(0.1)
    return None


def proc_cpu(pid):
    if pid is None:
        return None
    try:
        f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')
    except OSError:
        return None


def proc_status(pid, key):
    if pid is None:
        return None
    try:
        for line in open(f'/proc/{pid}/status'):
            if line.startswith(key + ':'):
                return int(line.split()[1])
    except OSError:
        return None
    return None


def proc_rss_kb(pid):
    return proc_status(pid, 'VmRSS')


def dump_declared(ctrl):
    """Names of declared FollowPath.* params (verifies the YAML against what the 1.1.18 plugin actually declares)."""
    try:
        out = subprocess.run(['ros2', 'param', 'list', '/controller_server', '--no-daemon'],
                             capture_output=True, text=True, timeout=30).stdout
        return sorted(line.strip() for line in out.splitlines() if line.strip().startswith('FollowPath.'))
    except Exception as e:  # noqa: BLE001
        return ['ERROR ' + repr(e)]


def log_counts(path):
    pats = {
        'log_collision_ahead': 'collision ahead',
        'log_failed_progress': 'Failed to make progress',
        'log_mppi_fail': 'Optimizer fail',
        'log_dwb_no_valid': 'No valid trajector',
        'log_missed_rate': 'missed its desired rate',
        'log_controller_exception': 'Exception',
        'log_aborting': 'Aborting',
        'log_error_lines': '[ERROR]',
    }
    try:
        txt = open(path, errors='replace').read()
    except OSError:
        return {}
    return {k: txt.count(p) for k, p in pats.items()}


def metrics(node, path, world, a, t_send, t_end, status, n_nav0):
    st_names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    m = {'status': st_names.get(status, status), 'success': status == 4, 'time_s': round(t_end - t_send, 2)}
    poses = [p for p in node.log_pose if t_send <= p[0] <= t_end]
    dev, sarc, clear_all, clear_box = [], [], [], []
    for (t, x, y, yaw, v, w) in poses:
        d, s = path.project(x, y)
        dev.append(d)
        sarc.append(s)
        fp = sm.transform_footprint(x, y, yaw)
        clear_all.append(sm.polygon_clearance(fp, world.obstacle_points(x, y, 2.0)))
    if poses:
        last = poses[-1]
        gx, gy = path.p[-1]
        m['final_goal_dist'] = round(math.hypot(last[1] - gx, last[2] - gy), 3)
        m['progress_m'] = round(max(sarc), 2)
        m['dev_mean'] = round(float(np.mean(dev)), 3)
        m['dev_max'] = round(float(np.max(dev)), 3)
        m['min_clearance'] = round(float(np.min(clear_all)), 3)
        m['contact'] = bool(np.min(clear_all) <= 0.0)
        m['dist_travelled'] = round(float(sum(math.hypot(b[1] - a_[1], b[2] - a_[2])
                                              for a_, b in zip(poses, poses[1:]))), 2)
    # obstacle-box clearance (separately, box only) for block / transient
    for name, key in (('block', 'static_block'), ('transient', 'transient')):
        if a.scenario != key:
            continue
        if key == 'static_block':
            cx, cy, hx, hy = world.boxes['block']
        else:
            (cx, cy), _ = path.point_at(TRANSIENT_S)
            hx = hy = BOX_HALF
        outline = sm.box_outline(cx, cy, hx, hy)
        win = poses
        if key == 'transient':
            ev = dict((e, t) for t, e in node.events)
            ta, td = ev.get('transient_appear'), ev.get('transient_disappear', t_end)
            win = [p for p in poses if ta is not None and ta <= p[0] <= td]
            m['transient_appear_t'] = None if ta is None else round(ta - t_send, 2)
            m['transient_gone_t'] = None if 'transient_disappear' not in ev else round(td - t_send, 2)
            # stop: robot speed < 0.02 for >= 0.5 s while box present; resume: first v > 0.05 after disappear
            stopped = [p for p in win if abs(p[4]) < 0.02]
            m['stopped_while_present'] = len(stopped) >= 5
            after = [p for p in poses if p[0] > td and abs(p[4]) > 0.05]
            m['resume_delay_s'] = round(after[0][0] - td, 2) if (after and 'transient_disappear' in ev) else None
        if win:
            cb = [sm.polygon_clearance(sm.transform_footprint(p[1], p[2], p[3]), outline) for p in win]
            m['min_clear_box'] = round(float(np.min(cb)), 3)
            m['box_contact'] = bool(np.min(cb) <= 0.0)
            if key == 'static_block':
                m['stopped_before_box'] = bool(np.min(cb) > 0.0 and abs(poses[-1][4]) < 0.02)
                m['passed_box'] = bool(max(sarc) > BLOCK_S + 0.5) if sarc else False
    # controller output (cmd_vel_nav) and smoothed output (cmd_vel = what the adapter sees)
    nav = [c for c in node.log_nav[n_nav0:] if t_send <= c[0] <= t_end]
    cmd = [c for c in node.log_cmd if t_send <= c[0] <= t_end]
    if len(nav) > 3:
        tn = np.array([c[0] for c in nav])
        vx = np.array([c[1] for c in nav])
        wz = np.array([c[2] for c in nav])
        per = np.diff(tn) * 1000.0
        m['nav_period_ms_p50'] = round(float(np.percentile(per, 50)), 1)
        m['nav_period_ms_p95'] = round(float(np.percentile(per, 95)), 1)
        m['nav_period_ms_p99'] = round(float(np.percentile(per, 99)), 1)
        m['nav_period_ms_max'] = round(float(per.max()), 1)
        fps, flips = sm.sign_flips_per_s(list(tn), list(wz))
        m['wz_flips_per_s'] = round(fps, 3)
        m['wz_flips'] = flips
        # per-cycle differences at the nominal 20 Hz period (arrival-time jitter would blow up d/dt)
        dt = np.full(len(tn) - 1, 0.05)
        acc = np.diff(vx) / dt
        jerk = np.diff(acc) / dt[1:] if len(acc) > 1 else np.zeros(1)
        m['vx_jerk_rms'] = round(float(np.sqrt(np.mean(jerk ** 2))), 2)
        wacc = np.diff(wz) / dt
        m['wz_acc_rms'] = round(float(np.sqrt(np.mean(wacc ** 2))), 2)
        moving = (np.abs(vx) > 0.02)
        m['vx_start_stop_transitions'] = int(np.sum(moving[1:] != moving[:-1]))
        m['nav_vx_mean_moving'] = round(float(vx[moving].mean()), 3) if moving.any() else 0.0
    m['raw_nav'] = [[round(c[0] - t_send, 3), round(c[1], 4), round(c[2], 4)] for c in nav]
    m['raw_cmd'] = [[round(c[0] - t_send, 3), round(c[1], 4), round(c[2], 4)] for c in cmd]
    m['raw_pose'] = [[round(p[0] - t_send, 3)] + [round(v, 4) for v in p[1:]] for p in poses]
    if len(cmd) > 3:
        tc = np.array([c[0] for c in cmd])
        vx = np.array([c[1] for c in cmd])
        wz = np.array([c[2] for c in cmd])
        pure = (np.abs(vx) < 0.02) & (np.abs(wz) > 0.15)
        subrot = (np.abs(vx) < 0.02) & (np.abs(wz) > 0.05) & (np.abs(wz) <= 0.15)
        lowv = (np.abs(vx) >= 0.02) & (np.abs(vx) < 0.15)
        n = len(cmd)
        m['p23_pure_rot_frac'] = round(float(pure.mean()), 3)
        m['p23_subthresh_rot_frac'] = round(float(subrot.mean()), 3)
        m['p23_low_vx_frac'] = round(float(lowv.mean()), 3)
        m['p23_mode_switches'] = int(np.sum(pure[1:] != pure[:-1]))
        m['cmd_gap_ms_max'] = round(float(np.diff(tc).max() * 1000.0), 1)
        m['cmd_gaps_over_150ms'] = int(np.sum(np.diff(tc) > 0.15))
        m['n_cmd'] = n
    return m


# ----------------------------------------------------------------------------------------------- matrix
def matrix(a):
    os.makedirs(a.results, exist_ok=True)
    rows = []
    for model in a.models.split(','):
        for sc in a.scenarios.split(','):
            for ctrl in a.controllers.split(','):
                out = os.path.join(a.results, f'{ctrl}_{sc}_{model}_s{a.seed}.json')
                if a.skip_existing and os.path.exists(out):
                    rows.append(json.load(open(out)))
                    continue
                print(f'=== {ctrl} {sc} {model}', flush=True)
                subprocess.run([sys.executable, os.path.abspath(__file__), 'single', '--controller', ctrl,
                                '--scenario', sc, '--model', model, '--seed', str(a.seed), '--out', out],
                               timeout=240)
                time.sleep(2.0)   # let DDS participants of the previous run vanish
                if os.path.exists(out):
                    rows.append(json.load(open(out)))
    with open(os.path.join(a.results, 'summary.json'), 'w') as f:
        json.dump(rows, f, indent=1)
    print(table(rows))


def table(rows):
    cols = ['controller', 'scenario', 'model', 'status', 'time_s', 'progress_m', 'final_goal_dist', 'dev_mean',
            'dev_max', 'min_clearance', 'min_clear_box', 'box_contact', 'stopped_while_present', 'resume_delay_s',
            'passed_box', 'wz_flips_per_s', 'vx_jerk_rms', 'p23_pure_rot_frac', 'p23_low_vx_frac',
            'p23_mode_switches', 'cmd_gaps_over_150ms', 'nav_period_ms_p50', 'nav_period_ms_p95',
            'nav_period_ms_p99', 'cs_cpu_pct', 'cs_cpu_ms_per_cycle', 'cs_rss_mb_max', 'log_collision_ahead',
            'log_failed_progress', 'log_mppi_fail', 'log_dwb_no_valid', 'log_missed_rate', 'error']
    lines = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
    for r in rows:
        lines.append('| ' + ' | '.join('' if r.get(c) is None else str(r.get(c)) for c in cols) + ' |')
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('single')
    s.add_argument('--controller', required=True, choices=['rpp', 'dwb', 'mppi'])
    s.add_argument('--scenario', required=True, choices=SCENARIOS)
    s.add_argument('--model', default='ideal', choices=['ideal', 'mc_hyp', 'mc_lin'])
    s.add_argument('--seed', type=int, default=1)
    s.add_argument('--out', required=True)
    mx = sub.add_parser('matrix')
    mx.add_argument('--controllers', default='rpp,dwb,mppi')
    mx.add_argument('--scenarios', default=','.join(SCENARIOS))
    mx.add_argument('--models', default='ideal,mc_hyp')
    mx.add_argument('--seed', type=int, default=1)
    mx.add_argument('--results', default=os.path.join(HERE, 'results'))
    mx.add_argument('--skip-existing', action='store_true')
    tb = sub.add_parser('table')
    tb.add_argument('--results', default=os.path.join(HERE, 'results'))
    a = p.parse_args()
    if a.cmd == 'single':
        return single_run(a)
    if a.cmd == 'table':
        print(table(json.load(open(os.path.join(a.results, 'summary.json')))))
        return 0
    return matrix(a)


if __name__ == '__main__':
    sys.exit(main())
