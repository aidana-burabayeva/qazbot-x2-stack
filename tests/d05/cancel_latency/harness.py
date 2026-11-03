#!/usr/bin/env python3
"""R5 dry harness (isolated ROS_DOMAIN_ID=84, localhost only). REAL: controller_server + velocity_smoother (Nav2 1.1.18,
live params), goal_coordinator.py, x2_motion_adapter.py (live:=false). STUBS (this node): bt_navigator get_state,
private NavigateToPose (FollowPath forwarder; no BT recovery needed: controller failure_tolerance keeps the goal),
localization_trust topics/arm_check, MC common state, map->odom. B1: cancel mid-cruise. B2: bag replay of
scan_obstacles/scan_clearing/odom->base_footprint (restamped, original log-stamp relation kept), obstacle events.
usage: harness.py PARAMS OUT WIN_PKL REPS"""
import copy, json, math, os, pickle, signal, subprocess, sys, time
assert os.environ.get('ROS_DOMAIN_ID') == '84' and os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'domain 84 only'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.msg import Costmap
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from action_msgs.srv import CancelGoal
from aimdk_msgs.msg import McCommonState
from rclpy.qos import QoSProfile, ReliabilityPolicy

PARAMS, OUT, WIN, REPS = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
NS, NAV = '/qazbot_d1_map', '/qazbot_d1_map/nav2'
SRC = '/home/agi/control_backend/ws/src/navigation'
EVENTS = [50.51, 79.31, 229.91, 371.11, 503.71]   # bag-relative stamp-log of first scan_obstacles frame with pts in corridor
procs = []
def start(cmd, log, cwd=None):
    p = subprocess.Popen(cmd, stdout=open(os.path.join(OUT, log), 'w'), stderr=subprocess.STDOUT, start_new_session=True, cwd=cwd)
    procs.append(p); return p
remap = ['-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static']
start(['ros2', 'run', 'nav2_controller', 'controller_server', '--ros-args', '-r', f'__ns:={NAV}', '-r', '__node:=controller_server',
       *remap, '-r', 'cmd_vel:=cmd_vel_nav', '--params-file', PARAMS], 'ctl.log')
start(['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args', '-r', f'__ns:={NAV}', '-r', '__node:=velocity_smoother',
       *remap, '-r', 'cmd_vel:=cmd_vel_nav', '-r', f'cmd_vel_smoothed:={NS}/cmd_vel', '--params-file', PARAMS], 'sm.log')
start(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-r', f'__ns:={NAV}', '-r', '__node:=lifecycle_manager_control',
       '-p', 'autostart:=true', '-p', 'node_names:=[controller_server,velocity_smoother]'], 'lm.log')
start(['python3', f'{SRC}/goal_coordinator.py', '--ros-args', '-r', f'__ns:={NS}', '-p', 'allow_dry_run_driver:=true'], 'gc.log', cwd=SRC)
start(['python3', f'{SRC}/x2_motion_adapter.py', '--ros-args', '-r', f'__ns:={NS}', '-p', 'live:=false', '-p', 'priority:=30'], 'ad.log', cwd=SRC)

rclpy.init()
n = Node('r5_harness')
LOG = open(os.path.join(OUT, 'events.jsonl'), 'w')
def ev(kind, **kw):
    kw.update(t=time.time_ns(), k=kind); LOG.write(json.dumps(kw) + '\n')
def tw(name):
    return lambda m: ev(name, vx=m.linear.x, wz=m.angular.z)
rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
for topic, name in [(f'{NAV}/cmd_vel_nav', 'nav'), (f'{NS}/cmd_vel', 'sm'), (f'{NS}/cmd_vel_authorized', 'auth'),
                    (f'{NS}/motion_adapter/dry_run_cmd', 'ad')]:
    n.create_subscription(Twist, topic, tw(name), rel)
gcst, adst = {}, {}
def on_gc(m): gcst.clear(); gcst.update(json.loads(m.data)); ev('gcs', **{k: gcst.get(k) for k in ('busy', 'latched', 'gate', 'fault', 'source_zero', 'ready')})
def on_ad(m): adst.clear(); adst.update(json.loads(m.data)); ev('ads', **{k: adst.get(k) for k in ('state', 'reason', 'latch', 'ready', 'motion_armed')})
n.create_subscription(String, f'{NS}/goal_coordinator/status', on_gc, 10)
n.create_subscription(String, f'{NS}/motion_adapter/status', on_ad, 10)
def on_cm(m):
    # lethal cells in the forward corridor x in [0.17, 0.65] (robot frame approx: costmap is odom; robot pose from last TF)
    x0, y0, th = pose['x'], pose['y'], pose['th']; r = m.metadata.resolution; ox, oy = m.metadata.origin.position.x, m.metadata.origin.position.y
    W = m.metadata.size_x; c = 0
    for i, v in enumerate(m.data):
        if v != 254: continue
        gx, gy = ox + (i % W + 0.5) * r - x0, oy + (i // W + 0.5) * r - y0
        x, y = math.cos(th) * gx + math.sin(th) * gy, -math.sin(th) * gx + math.cos(th) * gy
        if 0.0 < x < 0.70 and abs(y) < 0.42: c += 1
    ev('cm', stamp=m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, lethal_corr=c)
n.create_subscription(Costmap, f'{NAV}/local_costmap/costmap_raw', on_cm, 10)

# ---- stubs
pose = dict(x=0.0, y=0.0, th=0.0)
tf_pub = n.create_publisher(TFMessage, f'{NS}/tf', rel)
ob_pub = n.create_publisher(PointCloud2, '/qazbot_lio/scan_obstacles', rel)
cl_pub = n.create_publisher(PointCloud2, '/qazbot_lio/scan_clearing', rel)
tr_pub = n.create_publisher(Bool, f'{NS}/localization_trust/trusted', 10)
ts_pub = n.create_publisher(String, f'{NS}/localization_trust/status', 10)
mc_pub = n.create_publisher(McCommonState, '/aima/mc/common/state', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
od_pub = n.create_publisher(Odometry, '/qazbot_native/odom', 10)
st = dict(static_base=True)
def tfs(parent, child, x, y, th):
    t = TransformStamped(); t.header.stamp = n.get_clock().now().to_msg(); t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y = x, y; t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
    return t
def tick20():
    m = TFMessage(transforms=[tfs('map', 'odom', 0.0, 0.0, 0.0)])
    if st['static_base']: m.transforms.append(tfs('odom', 'base_footprint', pose['x'], pose['y'], pose['th']))
    tf_pub.publish(m)
    o = Odometry(); o.header.stamp = n.get_clock().now().to_msg(); o.header.frame_id, o.child_frame_id = 'odom', 'base_footprint'
    o.pose.pose.position.x, o.pose.pose.position.y = pose['x'], pose['y']; o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose['th'] / 2), math.cos(pose['th'] / 2)
    od_pub.publish(o)
def tick10():
    tr_pub.publish(Bool(data=True)); ts_pub.publish(String(data=json.dumps({'trusted': True, 't1_transient': False, 'reasons': []})))
    s = McCommonState(); s.input_source.name = 'rc'; s.body_status.value = 1; s.fsm_state.current_state = 4
    mc_pub.publish(s)
n.create_timer(0.05, tick20); n.create_timer(0.1, tick10)
n.create_service(GetState, f'{NAV}/bt_navigator/get_state', lambda q, r: (setattr(r.current_state, 'id', 3), r)[1])
n.create_service(Trigger, f'{NS}/localization_trust/arm_check', lambda q, r: (setattr(r, 'success', True), r)[1])
fp = ActionClient(n, FollowPath, f'{NAV}/follow_path')
def mkpath():
    g = FollowPath.Goal(); g.path = Path(); g.path.header.frame_id = 'odom'; g.path.header.stamp = n.get_clock().now().to_msg()
    for i in range(121):
        p = PoseStamped(); p.header = g.path.header
        p.pose.position.x = pose['x'] + i * 0.05 * math.cos(pose['th']); p.pose.position.y = pose['y'] + i * 0.05 * math.sin(pose['th'])
        p.pose.orientation.z, p.pose.orientation.w = math.sin(pose['th'] / 2), math.cos(pose['th'] / 2); g.path.poses.append(p)
    g.controller_id, g.goal_checker_id = 'FollowPath', 'goal_checker'; return g
stub = dict(fh=None)
async def ntp_exec(h):
    ev('stub_ntp_exec')
    fh = await fp.send_goal_async(mkpath()); stub['fh'] = fh; ev('stub_fp_accepted', ok=fh.accepted)
    res = await fh.get_result_async(); ev('stub_fp_result', status=res.status)
    if h.is_cancel_requested: h.canceled(); ev('stub_ntp_canceled')
    else: h.abort(); ev('stub_ntp_abort')
    return NavigateToPose.Result()
def ntp_cancel(h):
    ev('stub_ntp_cancel_req')
    if stub['fh'] is not None: stub['fh'].cancel_goal_async()
    return CancelResponse.ACCEPT
ActionServer(n, NavigateToPose, f'{NAV}/navigate_to_pose', ntp_exec, goal_callback=lambda r: GoalResponse.ACCEPT, cancel_callback=ntp_cancel)
pub = ActionClient(n, NavigateToPose, f'{NS}/navigate_to_pose')
clear = n.create_client(ClearEntireCostmap, f'{NAV}/local_costmap/clear_entirely_local_costmap')
gc_resume = n.create_client(Trigger, f'{NS}/goal_coordinator/resume')
gc_stop = n.create_client(Trigger, f'{NS}/goal_coordinator/stop')
cancel_all = n.create_client(CancelGoal, f'{NS}/navigate_to_pose/_action/cancel_goal')

def spin(sec, until=None):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.005)
        if until and until(): return True
    return bool(until and until())
def call(cli, req, tmo=3.0):
    if not cli.wait_for_service(timeout_sec=tmo): return None
    f = cli.call_async(req); spin(tmo, f.done); return f.result()
def send_goal():
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.pose.position.x = 6.0; g.pose.pose.orientation.w = 1.0
    f = pub.send_goal_async(g); spin(5.0, f.done); h = f.result(); ev('goal_resp', accepted=bool(h and h.accepted)); return h if h and h.accepted else None
def reset_gc():
    spin(6.0, lambda: gcst.get('busy') is False and gcst.get('source_zero') is True)
    r = call(gc_resume, Trigger.Request()); ev('gc_resume', ok=bool(r and r.success), msg=r.message if r else None)
    spin(1.5, lambda: adst.get('state') == 'idle' and gcst.get('latched') is False)

D = pickle.load(open(WIN, 'rb')); T0 = D['T0']
def schedule(a, b):
    """messages with bag log time in [a, b] s, as (log_ns, kind, payload)."""
    q = []
    for w in D['wins']:
        if not (w['a'] <= a and b <= w['b']): continue
        for t, m in w['tf']:
            tr = [x for x in m.transforms if (x.header.frame_id, x.child_frame_id) == ('odom', 'base_footprint')]
            if tr and T0 + a * 1e9 <= t <= T0 + b * 1e9: q.append((t, 'tf', tr))
        for key in ('ob', 'cl'):
            for t, s, f, P in w[key]:
                if T0 + a * 1e9 <= t <= T0 + b * 1e9: q.append((t, key, (s, f, P)))
    return sorted(q, key=lambda x: x[0])
def replay(q, on_start=None, hooks=()):
    """publish q at the original pace; stamps shifted by the same offset as the publish times."""
    off = time.time_ns() + 50_000_000 - q[0][0]; i = 0; hooks = sorted(hooks); hi = 0
    while i < len(q):
        now = time.time_ns()
        while i < len(q) and q[i][0] + off <= now:
            t, kind, pl = q[i]; i += 1
            if kind == 'tf':
                out = []
                for x in pl:
                    y = copy.deepcopy(x); s = x.header.stamp.sec * 10**9 + x.header.stamp.nanosec + off
                    y.header.stamp = Time(sec=s // 10**9, nanosec=s % 10**9); out.append(y)
                    pose['x'], pose['y'] = x.transform.translation.x, x.transform.translation.y
                    pose['th'] = 2 * math.atan2(x.transform.rotation.z, x.transform.rotation.w)
                tf_pub.publish(TFMessage(transforms=out))
            else:
                s, f, P = pl; s += off
                msg = pc2.create_cloud_xyz32(Header(stamp=Time(sec=s // 10**9, nanosec=s % 10**9), frame_id=f), P.tolist())
                (ob_pub if kind == 'ob' else cl_pub).publish(msg)
                if kind == 'ob': ev('ob_pub', bag_log=t, n=len(P), stamp=s)
        while hi < len(hooks) and hooks[hi][0] + off <= now:
            hooks[hi][1](); hi += 1
        rclpy.spin_once(n, timeout_sec=0.002)
    return off
try:
    assert fp.wait_for_server(timeout_sec=30.0) and pub.wait_for_server(timeout_sec=30.0), 'servers'
    spin(4.0, lambda: adst.get('ready') is True and gcst.get('ready') is True)
    ev('ready', gc=dict(gcst), ad=dict(adst)); print('ready', gcst.get('ready'), adst.get('ready'), flush=True)
    # ---------------- B1: cancel / stop mid-cruise (static robot, empty costmap)
    for mode in ['cancel_handle'] * REPS + ['cancel_all_srv', 'gc_stop']:
        st['static_base'] = True; reset_gc(); call(clear, ClearEntireCostmap.Request())
        ev('trial', b='B1', mode=mode); h = send_goal()
        if h is None: print('B1 goal rejected', mode, gcst, adst, flush=True); continue
        spin(4.0)                                         # cruise: smoother at 0.30 after 0.75 s ramp
        ev('stop_req', mode=mode)
        if mode == 'cancel_handle': f = h.cancel_goal_async(); spin(2.0, f.done)
        elif mode == 'cancel_all_srv': call(cancel_all, CancelGoal.Request())
        else: call(gc_stop, Trigger.Request())
        ev('stop_done', mode=mode); spin(3.0)
        ev('after', gc=dict(gcst), ad=dict(adst)); print('B1', mode, 'gc', gcst.get('latched'), gcst.get('fault'), 'ad', adst.get('state'), adst.get('reason'), flush=True)
    # ---------------- B2: obstacle appears during an active goal (bag replay)
    for rep in range(REPS):
        for e in EVENTS:
            reset_gc(); call(clear, ClearEntireCostmap.Request())
            q = schedule(e - 6.0, e + 2.5); st['static_base'] = False
            ev('trial', b='B2', event=e, rep=rep)
            hold = {}
            def go(): hold['h'] = 'pending'
            off = replay(q, hooks=[(T0 + int((e - 5.4) * 1e9), go)]) if False else None
            # send the goal 0.6 s into the replay (non-blocking), then let the replay run
            def mk():
                g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.pose.position.x = 6.0; g.pose.pose.orientation.w = 1.0
                hold['f'] = pub.send_goal_async(g); ev('goal_sent')
            off = replay(q, hooks=[(T0 + int((e - 5.4) * 1e9), mk)])
            ev('event_ref', event=e, rep=rep, off=off, ev_wall=T0 + int(e * 1e9) + off)
            spin(0.5); f = hold.get('f'); h = f.result() if f is not None and f.done() else None
            ev('goal_resp', accepted=bool(h and h.accepted))
            if h is not None and h.accepted:
                cf = h.cancel_goal_async(); spin(2.0, cf.done)
            st['static_base'] = True
            print('B2', e, rep, 'accepted', bool(h and h.accepted), flush=True)
    reset_gc()
finally:
    LOG.flush()
    for p in procs:
        if p.poll() is None:
            try: os.killpg(p.pid, signal.SIGINT)
            except (ProcessLookupError, PermissionError): pass
    time.sleep(2.0)
    for p in procs:
        if p.poll() is None: os.killpg(p.pid, signal.SIGKILL)
    n.destroy_node(); rclpy.try_shutdown()
