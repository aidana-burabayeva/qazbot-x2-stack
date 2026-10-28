#!/usr/bin/env python3
"""Full NavigateToPose chain in dry run with the REAL motion adapter and the REAL localization trust node.

goal -> goal_coordinator -> bt_navigator -> planner -> controller(RPP) -> velocity_smoother -> <ns>/cmd_vel
     -> coordinator gate -> <ns>/cmd_vel_authorized -> x2_motion_adapter (live=false, shadow state machine)
     -> <ns>/motion_adapter/dry_run_cmd -> kinematic model (loc_model_fixture.py).
Localization: emulated AMCL outputs + D1 ray-cast scan -> real localization_trust_node.
Isolated domain 87, localhost only; fake McCommonState only inside domain 87. No MC velocity publisher,
no registration, no /cmd_vel. Usage: run_model.sh NEW_RESULT_DIR
"""
import json, math, os, re, signal, subprocess, sys, time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires isolated domain 87, localhost only')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
sys.path.insert(0, str(Path(__file__).resolve().parent))
from stop_criterion import stop_check
from e2e_map import START, STRAIGHT, REPEAT, FAR, NEAR, WALL_X, OCCUPIED, OUT_OF_MAP, EXPECTED_PGM_SHA256

root = Path(__file__).resolve().parents[2]; nav = root / 'ws/src/navigation'; ns = 'qazbot_e2e'
out = root / 'tests/integration_results' / sys.argv[1]
if out.exists(): raise RuntimeError('result directory exists')
out.mkdir()
XY_TOL = YAW_TOL = 0.10            # nav_full.launch.py goal checker defaults xy_tol / yaw_tol
PERIOD = 1.0 / 20.0                # controller_frequency 20 Hz
procs, logs, results, known_defects, smac_libraries = [], [], [], {}, []


def start(name, args):
    f = (out / (name + '.log')).open('w'); logs.append(f)
    p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append((name, p)); return p


rclpy.init(); n = Node('integration_harness', namespace=ns)
S = dict(pose=None, vel=None, coord={}, adapter={}, trust={})
cmds, auth, odoms, internal_status = [], [], [], {}


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    S['pose'] = (p.x, p.y, 2 * math.atan2(q.z, q.w)); S['vel'] = (m.twist.twist.linear.x, m.twist.twist.angular.z)
    odoms.append((time.monotonic(),) + S['pose'] + S['vel'])
    if len(odoms) > 20000: del odoms[:10000]


def on_internal(m):
    for s in m.status_list: internal_status[bytes(s.goal_info.goal_id.uuid)] = s.status


sub = n.create_subscription
sub(Odometry, 'model_truth', on_odom, 50)          # model ground truth; <ns>/odom can be cut by a test
sub(Twist, 'motion_adapter/dry_run_cmd', lambda m: cmds.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), 100)
sub(Twist, 'cmd_vel_authorized', lambda m: auth.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z)), 100)
sub(String, 'goal_coordinator/status', lambda m: S.__setitem__('coord', json.loads(m.data)), 10)
sub(String, 'motion_adapter/status', lambda m: S.__setitem__('adapter', json.loads(m.data)), 10)
trust_log = []                                     # diagnostic: every change of the trust reasons


def on_trust(m):
    d = json.loads(m.data); S['trust'] = d
    key = (d.get('trusted'), tuple(d.get('reasons', [])))
    if not trust_log or trust_log[-1][1:] != key:
        trust_log.append((round(time.monotonic(), 3),) + key + (d.get('metrics', {}).get('scan_score'),))


sub(String, 'localization_trust/status', on_trust, 10)
sub(GoalStatusArray, 'nav2/navigate_to_pose/_action/status', on_internal, 10)
pub = n.create_publisher
reset = pub(Float64MultiArray, 'sim_reset', 1)
toggles = {k: pub(Bool, f'fixture_{k}_enabled', 1) for k in ('scan', 'odom', 'amcl')}
offset = pub(Float64MultiArray, 'fixture_loc_offset', 1)
mc = pub(String, 'fixture_mc', 1)
client = ActionClient(n, NavigateToPose, 'navigate_to_pose'); internal = ActionClient(n, NavigateToPose, 'nav2/navigate_to_pose')
srv = {k: n.create_client(Trigger, k) for k in ('goal_coordinator/stop', 'goal_coordinator/resume', 'motion_adapter/resume',
       'localization_trust/baseline_start', 'localization_trust/baseline_finish', 'localization_trust/confirm_anchor')}


def wait(pred, seconds=30):     # observation deadline of the TEST, not a runtime timeout
    end = time.monotonic() + seconds
    while not pred():
        dead = [name for name, p in procs if p.poll() is not None]
        if dead: raise RuntimeError('child exited: ' + str(dead))
        if time.monotonic() > end: raise RuntimeError('observation deadline: ' + json.dumps(dict(c=S['coord'], a=S['adapter'], t=S['trust'].get('reasons'))))
        rclpy.spin_once(n, timeout_sec=.02)


def spin(s):
    end = time.monotonic() + s
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=.02)


def call(name):
    c = srv[name]; wait(c.service_is_ready); f = c.call_async(Trigger.Request()); wait(f.done, 60); return f.result()


def check(name, ok, detail=None):
    results.append(dict(name=name, ok=bool(ok), detail=detail)); print(json.dumps(results[-1], default=str), flush=True)
    if not ok: raise AssertionError(name)


def goal_msg(x, y, yaw):
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
    g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2); return g


def new_goal(x, y, yaw):
    f = client.send_goal_async(goal_msg(x, y, yaw)); wait(f.done, 60); return f.result()


def stopped(): return S['vel'] is not None and max(abs(v) for v in S['vel']) < .01
def moving(): return S['vel'] is not None and S['vel'][0] > .15
def trusted(): return S['trust'].get('trusted') is True


def finish(h):
    f = h.get_result_async(); wait(f.done, 90); return f.result().status, time.monotonic()


def stop_ok(label, t_term, period=PERIOD):
    """Terminal observed at t_term -> (stop criterion) model at rest + stationary 1 s window."""
    wait(stopped, 10); w0 = time.monotonic(); spin(1.0)
    ok, ev = stop_check(cmds, odoms, t_term, period, w0)
    check(label + '_stop_criterion', ok, ev)


def pose_ok(label, target):
    p = S['pose']; err = math.hypot(p[0] - target[0], p[1] - target[1])
    yaw = abs(math.atan2(math.sin(p[2] - target[2]), math.cos(p[2] - target[2])))
    check(label + '_model_pose', err <= XY_TOL and yaw <= YAW_TOL, dict(pose=p, xy_error=err, yaw_error=yaw, tol=(XY_TOL, YAW_TOL)))


def no_remotion(label, seconds=1.5):
    t = time.monotonic(); p0 = S['pose']; spin(seconds)
    late = [c for c in cmds if c[0] >= t and any(abs(v) > 1e-9 for v in c[1:])]
    check(label + '_no_remotion', not late and math.dist(p0[:2], S['pose'][:2]) < .005, dict(late=late[:3], p0=p0, p1=S['pose']))


def resume_all():
    a = call('motion_adapter/resume') if S['adapter'].get('state') == 'latched' else None
    c = call('goal_coordinator/resume') if S['coord'].get('latched') else None
    wait(lambda: S['coord'].get('ready', False), 30)
    return a, c


def reset_pose(x=START[0], y=START[1], yaw=START[2], wall=-1., half=3.):
    reset.publish(Float64MultiArray(data=[x, y, yaw, wall, half])); spin(.5)
    wait(stopped); wait(lambda: not S['coord'].get('busy', True) and S['coord'].get('source_zero', False))
    # The reset TELEPORTS the model (not physical). The scan around the jump can be scored at a pose inconsistent
    # with it (odom interpolated across the discontinuity): T4/T6 then are DIAGNOSTICS only (READY/MOTION_OK split
    # 2026-10-22, no stable_s hysteresis; map->odom is unchanged, so no T5). Precondition of every case: localization trusted
    # AND the coordinator's composite admission state (status.ready, as resume_all uses). Trust alone is not enough:
    # the coordinator learns driver readiness from the adapter status (2 Hz), which lags a trust recovery by up to one
    # report period (nav_e2e_verify_04 takeover_goal_A_accepted). Same single 40 s observation deadline as before.
    wait(lambda: trusted() and S['coord'].get('ready', False), 40)


def reconfirm(jump):
    """Operator relocalization after READY was lost (trust READY/MOTION_OK split, 2026-10-22): wait until the
    restoring map->odom change has been seen and has left the T5 window (or TF is fresh again), then confirm."""
    if jump: wait(lambda: S['trust'].get('metrics', {}).get('jump_xy', 0) > .10, 10)
    wait(lambda: S['trust'].get('metrics', {}).get('jump_xy', 1) <= .10 and 'T1_tf_stale' not in S['trust'].get('reasons', []), 10)
    check('reconfirm_still_not_ready', S['trust'].get('ready') is False, S['trust'].get('reasons'))
    r = call('localization_trust/confirm_anchor'); check('reconfirm_anchor', r.success, r.message); wait(trusted, 20)


def drive_case(label, target):
    reset_pose(); cmds.clear(); h = new_goal(*target); check(label + '_accepted', h.accepted)
    st, t = finish(h); check(label + '_succeeded', st == 4, st)
    stop_ok(label, t); pose_ok(label, target)
    check(label + '_limits', all(abs(v) <= .300001 and abs(y) < 1e-9 and abs(w) <= .500001 for _, v, y, w in cmds))
    return h


def fault_case(label, inject, restore, expect_latch=None):
    """Drive, inject a fault, expect abort + stop; restore; expect no re-motion and latched until resume."""
    reset_pose(); h = new_goal(*FAR); check(label + '_goal_accepted', h.accepted, dict(trust=S['trust'].get('reasons'), adapter=S['adapter'].get('reason'), coord=S['coord']))
    wait(moving); inject(); st, t = finish(h)
    check(label + '_aborts', st == 6, dict(status=st, fault=S['coord'].get('fault')))
    stop_ok(label, t)
    if expect_latch: wait(lambda: S['adapter'].get('state') == 'latched', 5); check(label + '_adapter_latched', S['adapter'].get('latch') in expect_latch, S['adapter'].get('latch'))
    else: check(label + '_detected_by', True, dict(adapter=S['adapter'].get('state'), latch=S['adapter'].get('latch'), coordinator_fault=S['coord'].get('fault')))
    restore(); wait(trusted, 40); wait(lambda: S['coord'].get('trusted', False), 5)
    if expect_latch: check(label + '_recovery_stays_latched', S['coord'].get('latched') and not S['coord'].get('gate'), S['coord'])
    no_remotion(label + '_after_recovery')
    check(label + '_old_goal_not_revived', not S['coord'].get('busy') and S['coord'].get('child_uuid') is None, S['coord'])
    return h


try:
    # ---------------- bring-up: model, trust, adapter (dry run), Nav2 + coordinator (dry-run driver path)
    start('model', ['python3', str(root / 'tests/nav_e2e/loc_model_fixture.py'), ns, '--ros-args',
                    '-r', f'/{ns}/cmd_vel:=/{ns}/motion_adapter/dry_run_cmd'])
    wait(lambda: S['pose'] is not None)
    start('trust', ['python3', str(nav / 'localization_trust_node.py'), '--ros-args', '-r', f'__ns:=/{ns}',
                    '-r', f'/qazbot_d1_map/tf:=/{ns}/tf', '-r', f'/qazbot_lio/odom_level:=/{ns}/odom', '-r', f'/qazbot_lio/scan:=/{ns}/scan',
                    '-p', f'expected_map_sha256:={EXPECTED_PGM_SHA256}'])
    adapter = start('adapter', ['python3', str(nav / 'x2_motion_adapter.py'), '--ros-args', '-r', f'__ns:=/{ns}', '-p', 'live:=false',
                                '-p', f'cmd_vel_topic:=/{ns}/cmd_vel_authorized', '-p', f'tf_topic:=/{ns}/tf',
                                '-p', f'trust_topic:=/{ns}/localization_trust/trusted'])
    wait(lambda: S['adapter'].get('mode') == 'dry_run' and S['trust'])
    check('adapter_dry_run_mode', S['adapter'].get('live') is False and S['adapter'].get('registration') == 'dry_run', S['adapter'])
    check('no_MC_velocity_publisher', not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
    clients = n.get_client_names_and_types_by_node('motion_adapter', '/' + ns)
    check('no_registration_client', not any('SetMcInputSource' in c for c, _ in clients), clients)
    fds = [os.readlink(f'/proc/{adapter.pid}/fd/{x}') for x in os.listdir(f'/proc/{adapter.pid}/fd')]
    check('no_owner_lock_in_dry_run', not any('x2_motion_owner.lock' in x for x in fds))
    spin(5.0)
    check('trust_sees_served_map_identical', S['trust'].get('map_ok') == '', S['trust'].get('map_ok'))
    check('trust_map_is_nav2_v2', S['trust'].get('map_sha256') == EXPECTED_PGM_SHA256, S['trust'].get('map_sha256'))
    check('ambiguity_budget_default_conservative', S['trust']['experimental']['degenerate_budget_m'] == 0.0, S['trust']['experimental'])
    check('trust_baseline_start', call('localization_trust/baseline_start').success); spin(5.0)
    r = call('localization_trust/baseline_finish'); check('trust_baseline_finish', r.success, r.message)
    r = call('localization_trust/confirm_anchor'); check('trust_anchor_confirmed', r.success, r.message)
    wait(trusted, 20); check('real_trust_node_trusted', True, S['trust'].get('metrics'))
    start('nav2', ['ros2', 'launch', str(nav / 'nav_full.launch.py'), f'ns:={ns}', 'robot_radius:=0.454',
                   f'scan_topic:=/{ns}/scan', f'odom_topic:=/{ns}/odom', 'allow_dry_run_driver:=true'])
    wait(internal.server_is_ready, 60)
    wait(lambda: client.server_is_ready() and S['coord'].get('ready', False), 60)
    check('coordinator_accepts_dry_run_driver', S['coord'].get('driver_mode') == 'dry_run' and S['coord'].get('driver_ready'), S['coord'])
    for row in subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines():
        if '/lib/nav2_planner/planner_server ' in row and '__ns:=/' + ns + '/nav2' in row:
            pid = int(row.split()[0])
            smac_libraries = sorted({l.split()[-1] for l in Path(f'/proc/{pid}/maps').read_text().splitlines() if 'libnav2_smac_planner_2d.so' in l})
    check('expected_smac_overlay_loaded', bool(smac_libraries) and all(os.environ['QAZBOT_SMAC_PREFIX'] in x for x in smac_libraries), smac_libraries)

    # ---------------- nominal: straight 1.5 m, rotate in place, already at goal
    drive_case('straight', STRAIGHT)
    check('straight_adapter_idle_after', S['adapter'].get('state') == 'idle', S['adapter'])
    drive_case('rotate_in_place', (START[0], START[1], math.pi / 2))
    reset_pose(); cmds.clear(); h = new_goal(*START); check('already_there_accepted', h.accepted)
    st, t = finish(h); check('already_there_succeeded', st == 4, st); stop_ok('already_there', t); pose_ok('already_there', START)
    check('already_there_no_motion', all(abs(v) < 1e-9 and abs(w) < 1e-9 for _, v, _, w in cmds), cmds[:5])

    # ---------------- repeat goal: re-send while active is refused; same goal again after success -> no motion
    reset_pose(); target = REPEAT; h = new_goal(*target); check('repeat_first_accepted', h.accepted)
    wait(moving); check('repeat_while_active_rejected', not new_goal(*target).accepted)
    st, t = finish(h); check('repeat_first_succeeded', st == 4, st); stop_ok('repeat_first', t); pose_ok('repeat_first', target)
    cmds.clear(); h = new_goal(*target); check('repeat_again_accepted', h.accepted)
    st, t = finish(h); check('repeat_again_succeeded', st == 4, st); stop_ok('repeat_again', t); pose_ok('repeat_again', target)
    check('repeat_again_no_motion', all(abs(v) < 1e-9 and abs(w) < 1e-9 for _, v, _, w in cmds), cmds[:5])

    # ---------------- cancel
    reset_pose(); cmds.clear(); h = new_goal(*FAR); check('cancel_goal_accepted', h.accepted)
    wait(moving); f = h.get_result_async(); c = h.cancel_goal_async(); wait(c.done); wait(f.done); t = time.monotonic()
    check('cancel_terminal', f.result().status == 5, f.result().status); stop_ok('cancel', t)
    check('cancel_adapter_not_latched', S['adapter'].get('state') == 'idle', S['adapter'])
    check('resume_after_cancel', call('goal_coordinator/resume').success)

    # ---------------- obstacle (unmapped box in front, local costmap only) / unreachable / out of map / occupied
    reset_pose(wall=WALL_X, half=.3); spin(1.0); cmds.clear()
    h = new_goal(*FAR); check('obstacle_goal_accepted', h.accepted)
    st, t = finish(h); check('obstacle_goal_not_succeeded', st == 6, dict(status=st, fault=S['coord'].get('fault')))
    stop_ok('obstacle', t)
    check('obstacle_safe_termination_adapter_idle', S['adapter'].get('state') == 'idle' and not S['coord'].get('busy'), dict(a=S['adapter'].get('state'), fault=S['coord'].get('fault')))
    check('obstacle_body_clearance', WALL_X - S['pose'][0] > .464, dict(pose=S['pose'], clearance=WALL_X - S['pose'][0]))
    reset_pose(); wait(trusted, 40); resume_all(); cmds.clear(); h = new_goal(99., 99., 0.); check('unreachable_accepted_for_evaluation', h.accepted)
    st, t = finish(h); check('unreachable_aborted', st == 6, st); spin(.5)
    check('unreachable_no_motion', all(abs(v) < 1e-9 and abs(w) < 1e-9 for _, v, _, w in cmds))
    for label, x, y in OUT_OF_MAP + [('occupied',) + OCCUPIED]:
        cmds.clear(); h = new_goal(x, y, 0.); check(label + '_accepted_for_planning', h.accepted)
        st, t = finish(h); spin(.2); check(label + '_fails_without_crash', st == 6, st)
        check(label + '_no_motion', all(abs(v) < 1e-9 and abs(w) < 1e-9 for _, v, _, w in cmds))

    # ---------------- RViz goal_pose topic
    reset_pose(); pose_pub = n.create_publisher(PoseStamped, 'goal_pose', 1); wait(lambda: pose_pub.get_subscription_count() >= 1)
    gp = PoseStamped(); gp.header.frame_id = 'map'; gp.header.stamp = n.get_clock().now().to_msg()
    gp.pose.position.x, gp.pose.position.y = NEAR[:2]; gp.pose.orientation.w = 1.
    pose_pub.publish(gp); wait(lambda: S['coord'].get('busy', False)); wait(lambda: not S['coord'].get('busy', True), 60)
    t = time.monotonic(); stop_ok('rviz_goal_pose', t); pose_ok('rviz_goal_pose', NEAR)

    # ---------------- PS5 takeover through the REAL adapter (fake McCommonState source switch, domain 87)
    reset_pose(); h = new_goal(*FAR); check('takeover_goal_A_accepted', h.accepted)
    wait(moving); child = bytes(S['coord']['child_uuid']); mc.publish(String(data=json.dumps(dict(source='rc'))))
    st, t = finish(h); check('takeover_goal_A_terminal', st in (5, 6), st); stop_ok('takeover', t)
    wait(lambda: S['adapter'].get('state') == 'latched', 5)
    check('takeover_adapter_latched', S['adapter'].get('latch') == 'operator_takeover', S['adapter'])
    wait(lambda: internal_status.get(child) in (4, 5, 6), 10)
    check('takeover_internal_nav2_goal_terminal', internal_status.get(child) in (5, 6), internal_status.get(child))
    mc.publish(String(data=json.dumps(dict(source='sdk_qazbot_nav2'))))      # operator releases the stick
    spin(1.0); check('takeover_release_stays_latched', S['adapter'].get('state') == 'latched' and S['coord'].get('latched'), (S['adapter'], S['coord']))
    no_remotion('takeover_release')
    check('takeover_new_goal_refused_while_latched', not new_goal(*FAR).accepted)
    a, c = resume_all(); check('takeover_explicit_resume', a.success and c.success, (a.message, c.message))
    no_remotion('takeover_resume_alone')
    check('takeover_goal_A_not_revived', h.status in (5, 6) and not S['coord'].get('busy') and internal_status.get(child) in (5, 6),
          dict(public=h.status, internal=internal_status.get(child)))
    reset_pose(); target = NEAR; h = new_goal(*target); check('takeover_goal_B_accepted', h.accepted)
    st, t = finish(h); check('takeover_goal_B_succeeded', st == 4, st); stop_ok('takeover_goal_B', t); pose_ok('takeover_goal_B', target)

    # ---------------- source losses through the real trust node (+ adapter latch)
    on, off = Bool(data=True), Bool(data=False)
    for label, key, latch in [('scan_loss', 'scan', ('localization_untrusted',)), ('odom_loss', 'odom', ('localization_untrusted',)),
                              # AMCL loss: Nav2 itself loses map->odom first (its TF tolerance is shorter than the
                              # 1 s trust/adapter TF age), the controller aborts -> normal end, no latch needed.
                              ('amcl_loss', 'amcl', None)]:
        fault_case(label, lambda k=key: toggles[k].publish(off), lambda k=key: toggles[k].publish(on), latch)
        a, c = resume_all(); check(label + '_resume_ready', S['coord'].get('ready'), None if c is None else c.message)
    reset_pose(); toggles['amcl'].publish(off); wait(lambda: not trusted(), 5)
    check('amcl_loss_standing_untrusted', 'T1_tf_stale' in S['trust'].get('reasons', []), S['trust'].get('reasons'))
    check('amcl_loss_standing_goal_rejected', not new_goal(*FAR).accepted)
    # loss still present at a goal request = prolonged: READY lost, operator relocalization (confirm) required
    wait(lambda: S['trust'].get('ready') is False, 5)
    check('amcl_loss_at_goal_request_clears_ready', 'not_ready:input_loss_at_goal_request' in ' '.join(S['trust'].get('reasons', [])), S['trust'].get('reasons'))
    toggles['amcl'].publish(on); reconfirm(False); wait(lambda: S['coord'].get('ready', False), 10)

    # ---------------- false localization: wrong initial pose, pose jump while driving, recovery without resume
    reset_pose(); cmds.clear(); offset.publish(Float64MultiArray(data=[.6, 0., 0.])); wait(lambda: not trusted(), 5); spin(.5)
    check('wrong_initial_pose_untrusted', S['trust'].get('ready') is False and 'not_ready:T5_map_odom_jump' in S['trust'].get('reasons', []), S['trust'].get('reasons'))
    check('wrong_initial_pose_goal_rejected', not new_goal(*FAR).accepted)
    check('wrong_initial_pose_no_motion', all(abs(v) < 1e-9 and abs(w) < 1e-9 for _, v, _, w in cmds))
    offset.publish(Float64MultiArray(data=[0., 0., 0.])); reconfirm(True); wait(lambda: S['coord'].get('ready', False), 10)
    jump = Float64MultiArray(data=[.5, 0., 0.]); zero = Float64MultiArray(data=[0., 0., 0.])
    hA = fault_case('pose_jump', lambda: offset.publish(jump), lambda: (offset.publish(zero), reconfirm(True)), ('localization_untrusted', 'localization_lost'))
    check('pose_jump_old_goal_rejected_before_resume', not new_goal(*FAR).accepted)
    a, c = resume_all(); check('pose_jump_explicit_resume', c.success, c.message)
    no_remotion('pose_jump_resume_alone')
    check('pose_jump_goal_A_not_revived', hA.status == 6 and not S['coord'].get('busy'), hA.status)
    reset_pose(); target = NEAR; h = new_goal(*target); check('pose_jump_goal_B_accepted', h.accepted)
    st, t = finish(h); check('pose_jump_goal_B_succeeded', st == 4, st); stop_ok('pose_jump_goal_B', t); pose_ok('pose_jump_goal_B', target)

    # ---------------- operator stop
    reset_pose(); h = new_goal(*FAR); check('operator_goal_accepted', h.accepted)
    wait(moving); check('operator_stop_ack', call('goal_coordinator/stop').success)
    st, t = finish(h); check('operator_stop_aborts_old_goal', st == 6, st); stop_ok('operator_stop', t)
    check('operator_release_no_restart', S['coord'].get('latched') and not S['coord'].get('gate')); no_remotion('operator_stop')
    check('operator_resume', call('goal_coordinator/resume').success)

    # ---------------- driver (adapter) not ready / lost through MC state
    reset_pose(); mc.publish(String(data=json.dumps(dict(fsm=6)))); wait(lambda: not S['coord'].get('driver_ready', True), 5)
    check('unready_driver_rejects_goal', not new_goal(*NEAR).accepted, S['adapter'].get('reason'))
    mc.publish(String(data=json.dumps(dict(fsm=4)))); wait(lambda: S['coord'].get('ready', False), 10)
    hA = fault_case('driver_loss', lambda: mc.publish(String(data=json.dumps(dict(enabled=False)))),
                    lambda: mc.publish(String(data=json.dumps(dict(enabled=True)))), ('mc_state_stale',))
    a, c = resume_all(); check('driver_recovery_explicit_resume', a.success and c.success)
    reset_pose(); target = NEAR; h = new_goal(*target); check('driver_new_goal_after_recovery', h.accepted)
    st, t = finish(h); check('driver_new_goal_succeeds', st == 4, st); stop_ok('driver_new_goal', t); pose_ok('driver_new_goal', target)

    # ---------------- Nav2 restart (controller dies mid-goal; whole Nav2 group stops; known lifecycle shutdown defect)
    reset_pose(); h = new_goal(*FAR); check('restart_goal_A_accepted', h.accepted); wait(moving)
    ctl = [int(r.split()[0]) for r in subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines()
           if '/lib/nav2_controller/controller_server ' in r and '__ns:=/' + ns + '/nav2' in r]
    check('restart_single_controller', len(ctl) == 1, ctl)
    parent = dict(procs)['nav2']; procs.remove(('nav2', parent)); os.kill(ctl[0], signal.SIGINT)
    wait(lambda: parent.poll() is not None, 60); t = time.monotonic()
    known_defects['nav2_group_exit_after_controller_kill'] = parent.returncode
    stop_ok('restart_group_exit', t)
    check('restart_adapter_not_active', S['adapter'].get('state') in ('idle', 'latched'), S['adapter'])
    start('nav2_restarted', ['ros2', 'launch', str(nav / 'nav_full.launch.py'), f'ns:={ns}', 'robot_radius:=0.454',
                             f'scan_topic:=/{ns}/scan', f'odom_topic:=/{ns}/odom', 'allow_dry_run_driver:=true'])
    S['coord'] = {}; wait(lambda: S['coord'].get('ready', False), 90)
    check('restart_has_no_cached_goal', not S['coord'].get('busy') and not S['coord'].get('gate'), S['coord'])
    no_remotion('restart')
    target = NEAR; h2 = new_goal(*target); check('restart_new_goal_B_accepted', h2.accepted)
    check('restart_new_goal_UUID', bytes(h.goal_id.uuid) != bytes(h2.goal_id.uuid))
    st, t = finish(h2); check('restart_new_goal_B_succeeded', st == 4, st); stop_ok('restart_goal_B', t); pose_ok('restart_goal_B', target)
    check('never_a_global_cmd_vel_publisher', not n.get_publishers_info_by_topic('/cmd_vel'))
    check('never_a_MC_velocity_publisher', not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity'))
finally:
    for _, p in reversed(procs):
        if p.poll() is None: os.kill(p.pid, signal.SIGINT)
    codes = {}
    for name, p in reversed(procs):
        try: codes[name] = p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM); codes[name] = p.wait(timeout=5); codes[name + '_needed_sigterm'] = True
    for f in logs: f.close()
    final_log = out / ('nav2_restarted.log' if (out / 'nav2_restarted.log').exists() else 'nav2.log')
    codes['abnormal_children'] = len([x for x in re.findall(r'process has died.*exit code (-?\d+)', final_log.read_text()) if int(x) != 0])
    if (out / 'nav2_restarted.log').exists():
        first = re.findall(r'process has died.*exit code (-?\d+)', (out / 'nav2.log').read_text())
        known_defects['first_group_abnormal_children'] = len([x for x in first if int(x) != 0])
        known_defects['note'] = 'lifecycle manager shutdown defect (candidates/nav2_lifecycle_shutdown not accepted); recorded, not masked'
    (out / 'results.json').write_text(json.dumps(dict(checks=results, failed=[r['name'] for r in results if not r['ok']],
        shutdown=codes, known_defects=known_defects, smac_libraries=smac_libraries, trust_log=trust_log[-400:],
        scope='D1 grid + real Nav2 + REAL x2_motion_adapter (dry run shadow) + REAL localization_trust (emulated AMCL outputs, D1 ray-cast scan) + kinematic model on adapter dry_run_cmd; domain 87'), indent=2, default=str))
    n.destroy_node(); rclpy.try_shutdown()
    if any(v not in (0, False) for k, v in codes.items()): raise RuntimeError('unclean shutdown: ' + str(codes))
