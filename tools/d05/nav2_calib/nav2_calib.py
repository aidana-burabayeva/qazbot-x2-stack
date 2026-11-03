#!/usr/bin/env python3
"""D-05 calibration runner through REAL Nav2 goals (protocol P-D05, option B). Operator tooling, not a stack node.

Does only: (a) read live controller/smoother params (read-only services) and publish one per-run nav2_msgs/SpeedLimit
(percentage <= 100 %, so it can only LOWER the RPP desired speed), (b) start the bag recorder, (c) wait for the
operator's RViz goal and follow it via goal_coordinator status, (d) write run metadata (JSON). It never sends goals,
never publishes velocity, never calls adapter/GC/trust services. All admission, GC gate, trust, adapter and MC checks
stay in the path. After the goal ends the limit is restored (NO_SPEED_LIMIT); if the tool stops while a goal is
still busy it LEAVES the lower limit in place (safe side) and prints the restore command.

Exit codes: 0 done, 2 invalid request/envelope, 3 refused (preflight), 4 aborted (signal/timeout), 5 internal error.
"""
import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_core as core  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CTL_NAMES = ['FollowPath.desired_linear_vel', 'FollowPath.approach_velocity_scaling_dist',
             'FollowPath.min_approach_linear_velocity', 'goal_checker.xy_goal_tolerance', 'speed_limit_topic',
             'FollowPath.use_regulated_linear_velocity_scaling', 'FollowPath.regulated_linear_scaling_min_radius',
             'FollowPath.regulated_linear_scaling_min_speed', 'FollowPath.use_cost_regulated_linear_velocity_scaling',
             'FollowPath.use_rotate_to_heading', 'FollowPath.rotate_to_heading_min_angle',
             'FollowPath.use_collision_detection', 'FollowPath.max_allowed_time_to_collision_up_to_carrot',
             'controller_frequency', 'odom_topic']
SM_NAMES = ['max_velocity', 'min_velocity', 'max_accel', 'max_decel', 'feedback', 'smoothing_frequency',
            'velocity_timeout']


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def args_parse(argv):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--run-id', required=True, help='e.g. r01_030 (only [A-Za-z0-9_.-])')
    a.add_argument('--target-vx', type=float, help='m/s, per-run RPP speed (<= live envelope)')
    a.add_argument('--ns', default='/qazbot_d1_map')
    a.add_argument('--path-len-m', type=float, default=4.0, help='planned straight goal distance (metadata/estimate)')
    a.add_argument('--gain', type=float, default=1.61, help='assumed real/cmd ratio for the estimate only (D1)')
    a.add_argument('--goal-wait-s', type=float, default=300.0)
    a.add_argument('--max-goal-s', type=float, default=120.0)
    a.add_argument('--post-roll-s', type=float, default=4.0, help='keep recording after the goal ends (stop/coast)')
    a.add_argument('--record', action=argparse.BooleanOptionalAction, default=True)
    a.add_argument('--out-root', default='/home/agi/control_backend/docs/evidence/d05_nav2calib')
    a.add_argument('--restore-only', action='store_true', help='publish NO_SPEED_LIMIT (only if GC not busy) and exit')
    ns = a.parse_args(argv)
    if not ns.run_id or any(not (c.isalnum() or c in '_.-') for c in ns.run_id):
        a.error('--run-id: only [A-Za-z0-9_.-]')
    if not ns.restore_only and ns.target_vx is None:
        a.error('--target-vx required')
    for k in ('goal_wait_s', 'max_goal_s', 'post_roll_s'):
        if not 0 < getattr(ns, k) <= 600:
            a.error(f'--{k.replace("_", "-")} must be in (0, 600]')
    return ns


def main(argv=None):
    opt = args_parse(argv if argv is not None else sys.argv[1:])
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import Twist
    from nav2_msgs.msg import SpeedLimit
    from std_msgs.msg import String

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = {'sig': None}
    signal.signal(signal.SIGINT, lambda s, f: stop.update(sig='SIGINT'))
    signal.signal(signal.SIGTERM, lambda s, f: stop.update(sig='SIGTERM'))
    node = Node('d05_nav2_calib')
    log = node.get_logger()
    ns = opt.ns.rstrip('/')
    limit_topic = f'{ns}/nav2/speed_limit'
    st = {'gc': None, 'gc_t': None, 'ad': None, 'ad_t': None}
    meta = {'run_id': opt.run_id, 'tool': 'tools/d05/nav2_calib/nav2_calib.py', 'started_utc': utc(),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'), 'ns': ns, 'target_vx': opt.target_vx,
            'path_len_m_planned': opt.path_len_m, 'events': [], 'operator': {
                'tape_start_m': None, 'tape_stop_m': None, 'tape_at_cmd_end_m': None, 'notes': ''}}

    def event(name, **kw):
        meta['events'].append(dict(event=name, utc=utc(), ros_ns=node.get_clock().now().nanoseconds, **kw))

    def on_gc(m):
        try:
            st['gc'], st['gc_t'] = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    def on_ad(m):
        try:
            st['ad'], st['ad_t'] = json.loads(m.data), time.monotonic()
        except ValueError:
            pass

    node.create_subscription(String, f'{ns}/goal_coordinator/status', on_gc, 10)
    node.create_subscription(String, f'{ns}/motion_adapter/status', on_ad, 10)
    limit_pub = node.create_publisher(SpeedLimit, limit_topic, QoSProfile(
        depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))

    def spin_for(sec, until=None):
        end = time.monotonic() + sec
        while time.monotonic() < end and stop['sig'] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def age(key):
        return None if st[key + '_t'] is None else time.monotonic() - st[key + '_t']

    def gc_busy_or_unknown():
        return st['gc'] is None or age('gc') > core.STATUS_MAX_AGE_S or bool(st['gc'].get('busy'))

    def send_limit(value, n=3):
        msg = SpeedLimit()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.percentage, msg.speed_limit = True, float(value)
        for _ in range(n):
            limit_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

    def restore(reason):
        if gc_busy_or_unknown():
            log.warn(f'speed limit LEFT ACTIVE ({reason}): GC busy/unknown. Restore after the goal ends with: '
                     f'nav2_calib.py --run-id restore --restore-only --ns {ns}')
            event('limit_left_active', reason=reason)
            return False
        send_limit(core.NO_SPEED_LIMIT)
        event('limit_restored', reason=reason)
        log.info('speed limit restored (NO_SPEED_LIMIT)')
        return True

    def limit_counts():
        pubs = node.get_publishers_info_by_topic(limit_topic)
        foreign = [p for p in pubs if p.node_name != node.get_name()]
        return node.count_subscribers(limit_topic), len(foreign)

    rec = {'proc_pid': None, 'bag': None}

    def stop_recorder():
        if rec['proc_pid']:
            try:
                os.kill(rec['proc_pid'], signal.SIGINT)
                event('record_stop', pid=rec['proc_pid'])
            except ProcessLookupError:
                event('record_already_ended', pid=rec['proc_pid'])
            rec['proc_pid'] = None

    out_dir = opt.out_root
    os.makedirs(out_dir, exist_ok=True)
    meta_path = os.path.join(out_dir, f'calib_{opt.run_id}_{time.strftime("%H%M%S")}.meta.json')

    def finish(code, verdict):
        meta['exit_code'], meta['verdict'], meta['ended_utc'] = code, verdict, utc()
        if not opt.restore_only:
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=1, default=str)
            log.info(f'metadata: {meta_path}')
        node.destroy_node()
        rclpy.try_shutdown()
        return code

    try:
        spin_for(5.0, lambda: st['gc'] is not None and st['ad'] is not None and limit_counts()[0] >= 1)
        if opt.restore_only:
            return finish(0 if restore('restore_only') else 4, 'restore_only')

        # ---- (a) live params, read-only
        def get_params(srv, names):
            cli = node.create_client(GetParameters, srv)
            if not cli.wait_for_service(timeout_sec=3.0):
                raise core.CalibError(f'{srv} not available')
            fut = cli.call_async(GetParameters.Request(names=names))
            spin_for(3.0, fut.done)
            if not fut.done() or fut.result() is None:
                raise core.CalibError(f'{srv} no answer')
            out = {}
            for n, v in zip(names, fut.result().values):
                out[n] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value,
                          8: list(v.double_array_value)}.get(v.type)
            return out
        try:
            ctl = get_params(f'{ns}/nav2/controller_server/get_parameters', CTL_NAMES)
            sm = get_params(f'{ns}/nav2/velocity_smoother/get_parameters', SM_NAMES)
            meta['live_params'] = {'controller_server': ctl, 'velocity_smoother': sm}
            live = core.LiveParams(
                desired_linear_vel=ctl['FollowPath.desired_linear_vel'], smoother_max_vx=sm['max_velocity'][0],
                smoother_accel_x=sm['max_accel'][0], smoother_decel_x=sm['max_decel'][0],
                approach_dist=ctl['FollowPath.approach_velocity_scaling_dist'],
                min_approach_vel=ctl['FollowPath.min_approach_linear_velocity'],
                xy_goal_tolerance=ctl['goal_checker.xy_goal_tolerance'])
            if ctl.get('speed_limit_topic') not in ('speed_limit', limit_topic):
                raise core.CalibError(f"controller speed_limit_topic={ctl.get('speed_limit_topic')!r} unexpected")
            plan = core.plan_speed_limit(opt.target_vx, live)
            meta['speed_limit'] = {'topic': limit_topic, 'percentage': True, 'value': plan.percentage,
                                   'expected_cmd_vx': plan.expected_cmd_vx}
            meta['estimate'] = core.segment_estimate(opt.target_vx, opt.path_len_m, live, opt.gain)
            meta['estimate']['approach_scaling_active'] = core.approach_affects(opt.target_vx, live)
            meta['estimate']['min_path_for_6s_m'] = core.min_path_for_steady(opt.target_vx, 6.0, live, opt.gain)
        except (core.CalibError, KeyError, TypeError, IndexError) as e:
            log.error(f'INVALID: {e}')
            return finish(2, f'invalid: {e}')

        # ---- preflight (read-only)
        subs, foreign = limit_counts()
        why = core.preflight(st['gc'], age('gc'), st['ad'], age('ad'), subs, foreign)
        meta['preflight'] = {'refusals': why, 'gc': st['gc'], 'adapter': st['ad']}
        if why:
            log.error('REFUSED: ' + '; '.join(why))
            return finish(3, 'refused')

        # ---- (b) recorder
        if opt.record:
            max_s = int(opt.goal_wait_s + opt.max_goal_s + opt.post_roll_s + 30)
            env = dict(os.environ, OUT_ROOT=out_dir, NS=ns)
            r = subprocess.run(['bash', os.path.join(HERE, 'record_calib.sh'), opt.run_id, str(max_s)],
                               env=env, capture_output=True, text=True, timeout=30)
            parts = r.stdout.split()
            if r.returncode != 0 or 'pid' not in parts:
                log.error(f'recorder failed: {r.stdout} {r.stderr}')
                return finish(5, 'recorder_failed')
            rec['bag'], rec['proc_pid'] = parts[0], int(parts[parts.index('pid') + 1])
            meta['bag'] = rec['bag']
            event('record_start', bag=rec['bag'], pid=rec['proc_pid'])
            spin_for(2.0)   # let the recorder discover topics before the limit / goal

        # ---- (a) publish the per-run limit
        summ = core.RunSummary(opt.target_vx, plan)
        t_ref = time.monotonic()
        collecting = {'on': False}

        def on_nav(m):
            if collecting['on']:
                summ.nav_samples.append((time.monotonic() - t_ref, m.linear.x))

        def on_sm(m):
            if collecting['on']:
                summ.smooth_samples.append((time.monotonic() - t_ref, m.linear.x))
        node.create_subscription(Twist, f'{ns}/nav2/cmd_vel_nav', on_nav, 10)
        node.create_subscription(Twist, f'{ns}/cmd_vel', on_sm, 10)
        send_limit(plan.percentage)
        event('limit_set', percentage=plan.percentage, expected_cmd_vx=plan.expected_cmd_vx)
        log.info(f'speed limit {plan.percentage:.2f} % -> RPP {plan.expected_cmd_vx:.3f} m/s. '
                 f'READY: operator sends the goal in RViz (straight, ~{opt.path_len_m} m).')

        # ---- (c) wait for the operator goal, follow it
        tracker = core.GoalTracker(post_roll_s=opt.post_roll_s)
        t_wait, last_refresh, last_stale_log = time.monotonic(), 0.0, 0.0
        verdict, code = None, 0
        while verdict is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if stop['sig']:
                verdict, code = f'aborted_{stop["sig"]}', 4
                break
            if now - last_refresh > 1.0:   # idempotent refresh while the run is open
                if tracker.state in ('WAIT', 'ACTIVE'):
                    send_limit(plan.percentage, n=1)
                last_refresh = now
            gc_fresh = st['gc'] is not None and age('gc') <= core.STATUS_MAX_AGE_S
            if not gc_fresh and now - last_stale_log > 5.0:
                log.warn('goal_coordinator status stale')
                last_stale_log = now
            prev = tracker.state
            if gc_fresh:
                tracker.update(now, st['gc'])
            if tracker.state != prev:
                event(f'goal_{tracker.state.lower()}', gc=st['gc'])
                log.info(f'goal state {prev} -> {tracker.state}')
                collecting['on'] = tracker.state in ('ACTIVE', 'POST')
            if tracker.state == 'WAIT' and now - t_wait > opt.goal_wait_s:
                verdict, code = 'timeout_no_goal', 4
            elif tracker.state == 'ACTIVE' and now - tracker.t_start > opt.max_goal_s:
                verdict, code = 'timeout_goal_still_busy', 4
            elif tracker.state == 'DONE':
                verdict = 'goal_ended_fault' if (tracker.fault_end or tracker.latched_end) else 'goal_ended'
        collecting['on'] = False
        meta['goal'] = {'t_start_rel_s': None if tracker.t_start is None else tracker.t_start - t_ref,
                        't_end_rel_s': None if tracker.t_end is None else tracker.t_end - t_ref,
                        'gc_fault_at_end': tracker.fault_end, 'gc_latched_at_end': tracker.latched_end}
        meta['commanded'] = summ.to_dict(live)
        if meta['commanded']['nav_cmd_samples'] and not meta['commanded']['limit_honored']:
            log.error(f"speed limit NOT honoured: controller max {meta['commanded']['nav_cmd_max_vx']:.3f} m/s")
        restore(verdict)
        stop_recorder()
        log.info(f"run {opt.run_id}: {verdict}; steady cmd {meta['commanded']['steady_cmd_s']:.1f} s")
        return finish(code, verdict)
    except Exception as e:  # noqa: BLE001 - tooling: always restore/stop recorder and leave metadata
        log.error(f'internal error: {e!r}')
        try:
            restore('internal_error')
        finally:
            stop_recorder()
        return finish(5, f'internal_error: {e!r}')


if __name__ == '__main__':
    sys.exit(main())
