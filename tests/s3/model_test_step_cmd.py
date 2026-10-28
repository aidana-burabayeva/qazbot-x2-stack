#!/usr/bin/env python3
"""Model test of tests/s3/step_cmd.py in the isolated domain 87 (localhost only). No hardware DDS, no MC velocity.

Chain: step_cmd.py -> /qazbot_e2e/cmd_vel_authorized -> REAL x2_motion_adapter (live=false, shadow)
       -> /qazbot_e2e/motion_adapter/dry_run_cmd -> kinematic model (loc_model_fixture.py, fake McCommonState)
plus REAL localization_trust_node and REAL goal_coordinator (allow_dry_run_driver). Nav2 is not started: the tool
only uses the coordinator's stop service/status (CPU: a live stack may run on domain 0 on the same Jetson; all
children run under nice 19 and off the FAST-LIO cores 5,6).
Usage: run_model_s3.sh NEW_RESULT_DIR
"""
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('requires isolated domain 87, localhost only')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
root = Path(__file__).resolve().parents[2]; nav = root / 'ws/src/navigation'; ns = 'qazbot_e2e'
sys.path.insert(0, str(root / 'tests/nav_e2e'))
from e2e_map import EXPECTED_PGM_SHA256
out = root / 'tests/integration_results' / sys.argv[1]
if out.exists(): raise RuntimeError('result directory exists')
out.mkdir()
PRE = ['nice', '-n', '19', 'taskset', '-c', '0-4,7']
procs, logs, results = [], [], []
TOOL = str(root / 'tests/s3/step_cmd.py')
COMMON = ['--expect-mode', 'dry_run', '--ns', '/' + ns, '--odom-topic', f'/{ns}/odom']


def start(name, args):
    f = (out / (name + '.log')).open('w'); logs.append(f)
    p = subprocess.Popen(PRE + args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    procs.append((name, p)); time.sleep(1.0); return p


rclpy.init(); n = Node('s3_model_harness', namespace=ns)
S = dict(adapter={}, coord={}, trust=None, pose=None)
dry, auth = [], []
n.create_subscription(Twist, 'motion_adapter/dry_run_cmd', lambda m: dry.append((time.time(), m.linear.x, m.angular.z)), 100)
n.create_subscription(Twist, 'cmd_vel_authorized', lambda m: auth.append((time.time(), m.linear.x, m.angular.z)), 100)
n.create_subscription(String, 'motion_adapter/status', lambda m: S.__setitem__('adapter', json.loads(m.data)), 10)
n.create_subscription(String, 'goal_coordinator/status', lambda m: S.__setitem__('coord', json.loads(m.data)), 10)
n.create_subscription(Bool, 'localization_trust/trusted', lambda m: S.__setitem__('trust', m.data), 10)


def on_truth(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    S['pose'] = (p.x, p.y, 2 * math.atan2(q.z, q.w))


n.create_subscription(Odometry, 'model_truth', on_truth, 10)
mc = n.create_publisher(String, 'fixture_mc', 1)
scan_on = n.create_publisher(Bool, 'fixture_scan_enabled', 1)
srv = {k: n.create_client(Trigger, k) for k in ('motion_adapter/resume', 'localization_trust/baseline_start', 'localization_trust/baseline_finish',
                                                'localization_trust/confirm_anchor', 'goal_coordinator/resume')}


def wait(pred, seconds=30):
    end = time.monotonic() + seconds
    while not pred():
        dead = [k for k, p in procs if p.poll() is not None]
        if dead: raise RuntimeError('child exited: ' + str(dead))
        if time.monotonic() > end: raise RuntimeError('observation deadline: ' + json.dumps(dict(a=S['adapter'], c=S['coord'], t=S['trust'])))
        rclpy.spin_once(n, timeout_sec=.02)


def spin(s):
    end = time.monotonic() + s
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=.02)


def call(name):
    c = srv[name]; wait(c.service_is_ready); f = c.call_async(Trigger.Request()); wait(f.done, 60); return f.result()


def check(name, ok, detail=None):
    results.append(dict(name=name, ok=bool(ok), detail=detail)); print(json.dumps(results[-1], default=str), flush=True)


def run_tool(label, args, during=None, env_domain='87'):
    """Run the tool; `during(proc)` is polled while it runs. Returns (exit code, csv rows, summary, t_start, t_end)."""
    csvp = out / (label + '.csv')
    env = dict(os.environ, ROS_DOMAIN_ID=env_domain)
    t0 = time.time()
    with (out / (label + '.tool.log')).open('w') as f:
        p = subprocess.Popen(PRE + ['python3', TOOL] + args + ['--csv', str(csvp)], stdout=f, stderr=subprocess.STDOUT, env=env)
        while p.poll() is None:
            rclpy.spin_once(n, timeout_sec=.02)
            if during: during(p)
    spin(1.0)
    rows, summ = [], {}
    if csvp.exists():
        import csv
        rows = list(csv.DictReader(csvp.open()))
        summ = json.loads(Path(str(csvp) + '.summary.json').read_text())
    return p.returncode, rows, summ, t0, time.time()


def nonzero(v): return any(abs(x) > 1e-9 for x in v)


def segments(rows):
    """[(phase, step, n_rows, t_first, t_last)] for consecutive rows of the same (phase, step)."""
    seg = []
    for r in rows:
        k = (r['phase'], r['step'])
        if seg and seg[-1][0] == k: seg[-1][1].append(float(r['t_unix']))
        else: seg.append((k, [float(r['t_unix'])]))
    return [(k[0], k[1], len(ts), ts[0], ts[-1]) for k, ts in seg]


def no_mc_velocity_graph():
    return (not n.get_publishers_info_by_topic('/aima/mc/locomotion/velocity')
            and not n.get_publishers_info_by_topic('/cmd_vel'))


def adapter_idle_unarmed(): return S['adapter'].get('state') == 'idle' and not S['adapter'].get('motion_armed')


def zero_burst_then_silence(t_off, t_end):
    """Adapter output after the last nonzero tool command: zeros only, and nothing nonzero later."""
    after = [d for d in dry if d[0] >= t_off and d[0] <= t_end]
    return after and not any(nonzero(d[1:]) for d in after)


try:
    start('model', ['python3', str(root / 'tests/nav_e2e/loc_model_fixture.py'), ns, '--ros-args',
                    '-r', f'/{ns}/cmd_vel:=/{ns}/motion_adapter/dry_run_cmd'])
    wait(lambda: S['pose'] is not None)
    start('trust', ['python3', str(nav / 'localization_trust_node.py'), '--ros-args', '-r', f'__ns:=/{ns}',
                    '-r', f'/qazbot_d1_map/tf:=/{ns}/tf', '-r', f'/qazbot_lio/odom_level:=/{ns}/odom', '-r', f'/qazbot_lio/scan:=/{ns}/scan',
                    '-p', f'expected_map_sha256:={EXPECTED_PGM_SHA256}'])
    start('adapter', ['python3', str(nav / 'x2_motion_adapter.py'), '--ros-args', '-r', f'__ns:=/{ns}', '-p', 'live:=false',
                      '-p', f'cmd_vel_topic:=/{ns}/cmd_vel_authorized', '-p', f'tf_topic:=/{ns}/tf',
                      '-p', f'trust_topic:=/{ns}/localization_trust/trusted'])
    start('coordinator', ['python3', str(nav / 'goal_coordinator.py'), '--ros-args', '-r', f'__ns:=/{ns}', '-p', 'allow_dry_run_driver:=true'])
    wait(lambda: S['adapter'].get('mode') == 'dry_run' and S['coord'], 60)
    spin(5.0)
    r = call('localization_trust/baseline_start'); spin(8.0)
    r = call('localization_trust/baseline_finish'); check('trust_baseline_finish', r.success, r.message)
    r = call('localization_trust/confirm_anchor'); check('trust_anchor_confirmed', r.success, r.message)
    wait(lambda: S['trust'] is True and S['adapter'].get('ready'), 30)
    check('graph_no_mc_velocity_or_cmd_vel_publisher_before', no_mc_velocity_graph())

    # C0: wrong domain -> refused before any ROS node exists
    code, rows, _, _, _ = run_tool('c0_wrong_domain', ['--expect-domain', '0', '--rot', '0.1'] + COMMON)
    check('c0_wrong_domain_refused', code == 4 and not rows, code)

    # C1: rotation CCW, two steps; zeros between and after; path through the real adapter
    yaw0 = S['pose'][2]; dry.clear(); auth.clear()
    code, rows, summ, t0, t1 = run_tool('c1_rot', ['--expect-domain', '87', '--rot', '0.1,0.2', '--rot-dur', '1.5',
                                                    '--stop-after-yaw-deg', '360', '--stop-after-dist', '0.3'] + COMMON)
    seg = segments(rows)
    check('c1_exit_done', code == 0, (code, summ.get('abort_reason'), summ.get('limit_hit')))
    steps = [s for s in seg if s[0] == 'step']
    check('c1_two_steps_at_20hz', len(steps) == 2 and all(abs(s[2] / 1.5 - 20) <= 2 for s in steps), [(s[1], s[2]) for s in steps])
    pauses = [s for s in seg if s[0] == 'pause']
    check('c1_zero_pause_after_each_step_ge_1s', len(pauses) == 2 and all(s[4] - s[3] >= 0.95 - 0.05 and s[2] >= 19 for s in pauses),
          [(s[1], s[2], round(s[4] - s[3], 3)) for s in pauses])
    check('c1_pause_rows_all_zero', all(float(r['vx_cmd']) == 0 and float(r['wz_cmd']) == 0 for r in rows if r['phase'] != 'step'))
    check('c1_step_values_exact', {(r['step'], float(r['wz_cmd']), float(r['vx_cmd'])) for r in rows if r['phase'] == 'step'} == {('0', .1, 0.), ('1', .2, 0.)})
    out_vals = sorted({round(d[2], 6) for d in dry if nonzero(d[1:])})
    check('c1_adapter_output_equals_tool_steps', out_vals == [0.1, 0.2], out_vals)
    last_step_end = max([float(r['t_unix']) for r in rows if r['phase'] == 'step'], default=0.0)
    check('c1_adapter_zero_burst_then_silence_at_end', zero_burst_then_silence(last_step_end + 0.01, time.time()),
          sum(1 for d in dry if d[0] > last_step_end))
    first_pause_t = pauses[0][3] if pauses else 0.0; step1_t = steps[1][3] if len(steps) > 1 else 0.0
    check('c1_adapter_no_motion_between_steps', not any(nonzero(d[1:]) for d in dry if first_pause_t + 0.01 < d[0] < step1_t - 0.01))
    dyaw = math.degrees(math.atan2(math.sin(S['pose'][2] - yaw0), math.cos(S['pose'][2] - yaw0)))
    check('c1_model_rotated_ccw', dyaw > 10, round(dyaw, 1))
    wait(adapter_idle_unarmed, 5); check('c1_adapter_idle_unarmed_after', True, S['adapter'].get('state'))
    wait(lambda: S['coord'].get('latched'), 3); check('c1_coordinator_latched_nav2_locked_out', S['coord'].get('fault') == 'operator_stop', S['coord'].get('fault'))
    check('c1_summary_per_step', len(summ.get('per_step', [])) == 2 and all(s['onset_lag_s'] is not None for s in summ['per_step']), summ.get('per_step'))

    # C2: straight with distance limit -> exit 3, end zeros, disarmed
    dry.clear()
    code, rows, summ, _, _ = run_tool('c2_lin_limit', ['--expect-domain', '87', '--lin', '0.15,0.2', '--lin-dur', '3',
                                                        '--stop-after-dist', '0.25'] + COMMON)
    seg = segments(rows)
    check('c2_exit_limit', code == 3 and summ.get('limit_hit', '').startswith('dist'), (code, summ.get('limit_hit')))
    end = [s for s in seg if s[0] == 'end_zeros']
    check('c2_end_zeros_ge_1s', len(end) == 1 and end[0][4] - end[0][3] >= 0.9 and end[0][2] >= 19, end)
    t_last = max([float(r['t_unix']) for r in rows if r['phase'] == 'step'], default=0.0)
    check('c2_adapter_silent_after_limit', zero_burst_then_silence(t_last + 0.01, time.time()))
    wait(adapter_idle_unarmed, 5); check('c2_adapter_idle_unarmed_after', True)

    # C3: operator takeover (fake MC source switch to rc) -> adapter latch -> tool aborts, zeros
    dry.clear(); flag = dict(sent=False)

    def takeover(p):
        if not flag['sent'] and any(nonzero(d[1:]) for d in dry):
            mc.publish(String(data=json.dumps(dict(source='rc')))); flag['sent'] = time.time()
    code, rows, summ, _, _ = run_tool('c3_takeover', ['--expect-domain', '87', '--lin', '0.15', '--lin-dur', '8',
                                                       '--stop-after-dist', '3'] + COMMON, during=takeover)
    check('c3_exit_abort_operator_takeover', code == 2 and 'operator_takeover' in summ.get('abort_reason', ''), (code, summ.get('abort_reason')))
    end = [s for s in segments(rows) if s[0] == 'end_zeros']
    check('c3_end_zeros_ge_1s', len(end) == 1 and end[0][4] - end[0][3] >= 0.9, end)
    check('c3_no_adapter_motion_after_latch', not any(nonzero(d[1:]) for d in dry if d[0] > flag['sent'] + 0.25),
          [d for d in dry if d[0] > flag['sent']][:8])
    spin(1.0); check('c3_adapter_left_latched', S['adapter'].get('state') == 'latched' and S['adapter'].get('latch') == 'operator_takeover', S['adapter'].get('latch'))
    mc.publish(String(data=json.dumps(dict(source='sdk_qazbot_nav2')))); spin(1.0)
    check('c3_release_no_remotion', not any(nonzero(d[1:]) for d in dry if d[0] > flag['sent'] + 0.25))
    # C6: refuse to start while the adapter is latched
    code, rows, _, _, _ = run_tool('c6_refuse_latched', ['--expect-domain', '87', '--rot', '0.1'] + COMMON)
    check('c6_refused_while_latched', code == 4 and not any(r['phase'] == 'step' for r in rows), code)
    r = call('motion_adapter/resume'); check('c3_operator_resume', r.success, r.message)
    wait(lambda: S['adapter'].get('ready') and S['adapter'].get('state') == 'idle', 10)

    # C4: localization loss (scan off) -> untrusted -> abort
    dry.clear(); flag = dict(sent=False)

    def scan_loss(p):
        if not flag['sent'] and any(nonzero(d[1:]) for d in dry):
            scan_on.publish(Bool(data=False)); flag['sent'] = time.time()
    code, rows, summ, _, _ = run_tool('c4_untrusted', ['--expect-domain', '87', '--rot', '0.2', '--rot-dur', '10',
                                                        '--stop-after-dist', '0.3'] + COMMON, during=scan_loss)
    check('c4_exit_abort_untrusted', code == 2 and 'untrusted' in summ.get('abort_reason', ''), (code, summ.get('abort_reason')))
    check('c4_no_adapter_motion_after_loss', not any(nonzero(d[1:]) for d in dry if d[0] > flag['sent'] + 1.0))
    scan_on.publish(Bool(data=True)); wait(lambda: S['trust'] is True, 40)
    if S['adapter'].get('state') == 'latched':
        r = call('motion_adapter/resume'); check('c4_operator_resume', r.success, r.message)
    wait(lambda: S['adapter'].get('ready') and S['adapter'].get('state') == 'idle', 20)

    # C7: foreign publisher on cmd_vel_authorized -> refused
    intruder = n.create_publisher(Twist, 'cmd_vel_authorized', 1); spin(1.0)
    code, rows, _, _, _ = run_tool('c7_foreign_cmd_publisher', ['--expect-domain', '87', '--rot', '0.1'] + COMMON)
    check('c7_refused_foreign_command_publisher', code == 4 and not any(r['phase'] == 'step' for r in rows), code)
    n.destroy_publisher(intruder); spin(1.0)

    # C5: SIGINT mid-step -> zeros >= 1 s, adapter stop (left latched), exit 130
    dry.clear(); flag = dict(sent=False)

    def interrupt(p):
        if not flag['sent'] and any(nonzero(d[1:]) for d in dry):
            p.send_signal(signal.SIGINT); flag['sent'] = time.time()
    code, rows, summ, _, _ = run_tool('c5_sigint', ['--expect-domain', '87', '--rot', '0.2', '--rot-dur', '10',
                                                     '--stop-after-dist', '0.3'] + COMMON, during=interrupt)
    check('c5_exit_130', code == 130, code)
    end = [s for s in segments(rows) if s[0] == 'end_zeros']
    check('c5_sigint_zeros_ge_1s', len(end) == 1 and end[0][4] - end[0][3] >= 0.9 and end[0][2] >= 19, end)
    check('c5_last_row_zero', rows and float(rows[-1]['wz_cmd']) == 0 and float(rows[-1]['vx_cmd']) == 0)
    check('c5_adapter_zero_then_silence', zero_burst_then_silence(flag['sent'] + 0.2, time.time()))
    spin(1.0); check('c5_adapter_left_latched_stop', S['adapter'].get('state') == 'latched', S['adapter'].get('latch'))
    r = call('motion_adapter/resume'); check('c5_operator_resume', r.success, r.message)
    check('graph_no_mc_velocity_or_cmd_vel_publisher_after', no_mc_velocity_graph())
finally:
    for name, p in reversed(procs):
        if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
    for name, p in reversed(procs):
        try: p.wait(timeout=15)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL); p.wait()
    for f in logs: f.close()
    ok = sum(r['ok'] for r in results)
    (out / 'result.json').write_text(json.dumps(dict(passed=ok, total=len(results), checks=results,
        scope='domain 87 localhost; real adapter dry_run + real trust + real coordinator; no Nav2; no MC velocity'), indent=2, default=str))
    print(f'RESULT {ok}/{len(results)}', flush=True)
    n.destroy_node(); rclpy.try_shutdown()
