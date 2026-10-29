"""Scenario 'live chain': an operator session against a running stack — localise, register the driver, drive START -> GOAL -> START.

Client of the stack started by ``ros2 launch x2_twin_bringup twin.launch.py``. Procedure follows
docs/robot_docs/X2_LOCALIZATION_TRUST.md: RViz-like initial pose, no-motion AMCL updates until sigma is within the trust gate,
baseline, confirm_anchor, explicit driver registration, then goals. Goal outcomes are MEASUREMENTS (soft checks): the official MC
overshoots and stalls in-place turns, which is the point of the twin. Everything else (isolation, limits, registration) is hard.

  ros2 run x2_twin_scenarios live_chain --plant official --name run01 [--max-angular 0.5]
"""
import argparse
import json
import math
import os
import pathlib
import sys

import rclpy
from aimdk_msgs.msg import McCommonState, McLocomotionVelocity
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from x2_twin_scenarios.harness import Harness, SUCCEEDED

RVIZ_COV_XY, RVIZ_COV_YAW = 0.25, 0.06853891945200942       # RViz2 2D Pose Estimate defaults


def _trust_gate():
    """Thresholds of the trust node itself (not duplicated here)."""
    sys.path.insert(0, os.path.join(os.environ.get('TWIN_STACK_DIR', '/opt/x2twin/stack'), 'navigation'))
    from loc_trust import TrustConfig
    return TrustConfig()


def run(args) -> int:
    rclpy.init()
    h = Harness()
    mc_cmds, amcl_err, mc, cmd_trace = [], [], {}, []
    be = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_mc_state(m):
        mc.update(source=m.input_source.name, body=m.body_status.value, fsm=m.fsm_state.current_state)
    h.create_subscription(McCommonState, '/aima/mc/common/state', on_mc_state,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    h.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity',
                          lambda m: (mc_cmds.append((m.source, m.forward_velocity, m.lateral_velocity, m.angular_velocity)),
                                     cmd_trace.append((round(h.now(), 3), m.forward_velocity, m.angular_velocity))), be)

    def on_amcl(m):
        if h.pose:
            q = m.pose.pose.orientation
            yaw = 2 * math.atan2(q.z, q.w)
            amcl_err.append((math.hypot(m.pose.pose.position.x - h.pose[0], m.pose.pose.position.y - h.pose[1]),
                             abs(math.atan2(math.sin(yaw - h.pose[2]), math.cos(yaw - h.pose[2])))))
    h.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_amcl,
                          QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    start = tuple(float(v) for v in args.start.split(','))
    goal = (start[0] + args.distance, start[1], start[2])
    rc = 1
    try:
        # --- 1. the body and the MC
        h.wait(lambda: h.pose is not None and mc, 120, 'plant pose and MC state')
        if args.plant == 'official':
            h.check('official_mc_standing', mc.get('body') == 1 and mc.get('fsm') == 4, dict(mc))   # tools: sim supervisor brings it up
        h.wait(lambda: h.state['adapter'].get('live') is True, 120, 'live adapter status')
        h.check('adapter_is_live', h.state['adapter'].get('mode') == 'live', h.state['adapter'].get('mode'))
        h.check('registration_not_automatic', h.state['adapter'].get('registration') == 'not_requested')

        # --- 2. localisation (operator procedure)
        if args.localization == 'truth':       # twin-only: ground-truth map->odom + trust verdict (launch localization:=truth)
            h.wait(lambda: h.state['trust'].get('source') == 'truth_localizer' and h.state['trust'].get('trusted') is True, 120,
                   'truth localizer trust verdict')
            h.check('localization_is_truth', True, dict(trust=h.state['trust']))
        else:
            gate = _trust_gate()
            amcl_state = h.client(GetState, 'amcl/get_state')
            h.wait(amcl_state.service_is_ready, 120, 'amcl lifecycle service')
            h.wait(lambda: _amcl_active(h, amcl_state), 120, 'amcl active')
            ip = PoseWithCovarianceStamped()
            ip.header.frame_id, ip.header.stamp = 'map', h.get_clock().now().to_msg()
            ip.pose.pose.position.x, ip.pose.pose.position.y = start[0], start[1]
            ip.pose.pose.orientation.z, ip.pose.pose.orientation.w = math.sin(start[2] / 2), math.cos(start[2] / 2)
            ip.pose.covariance[0] = ip.pose.covariance[7] = RVIZ_COV_XY
            ip.pose.covariance[35] = RVIZ_COV_YAW
            h.initialpose.publish(ip)
            h.wait(lambda: len(amcl_err) > 0, 60, 'first amcl_pose')
            h.wait(lambda: 'sigma_xy' in h.state['trust'].get('metrics', {}), 60, 'trust sigma metric')

            def metrics():
                return h.state['trust']['metrics']

            def converged():
                return metrics()['sigma_xy'] <= gate.sigma_xy_max and metrics()['sigma_yaw'] <= gate.sigma_yaw_max
            h.check('amcl_starts_unconverged_like_rviz_pose', not converged(), dict(sigma_xy=metrics()['sigma_xy']))
            updates = 0
            while not converged():
                if updates >= 200:
                    raise RuntimeError('AMCL did not converge in 200 no-motion updates')
                h.empty('request_nomotion_update')
                updates += 1
                h.spin(0.6)
            h.check('amcl_converged_by_nomotion_updates', True, dict(updates=updates, sigma_xy=metrics()['sigma_xy']))
            h.check('trust_baseline_start', h.trigger('localization_trust/baseline_start').success)
            h.spin(5.0)
            r = h.trigger('localization_trust/baseline_finish')
            h.check('trust_baseline_finish', r.success, r.message)
            r = h.trigger('localization_trust/confirm_anchor')
            h.check('trust_anchor_confirmed', r.success, r.message)
            # docs/robot_docs/X2_LOCALIZATION_TRUST.md: a standing AMCL does not update by itself; keep requesting no-motion updates
            # until T2_covariance / T3_particles leave the status (the cloud contracts more slowly than sigma).
            gate_updates = 0
            while any(r.startswith(('T2_', 'T3_')) for r in h.state['trust'].get('reasons', []) or []):
                if gate_updates >= 100:
                    raise RuntimeError('T2/T3 did not clear in 100 further no-motion updates: '
                                       + json.dumps(h.state['trust'].get('reasons')))
                h.empty('request_nomotion_update')
                gate_updates += 1
                h.spin(0.6)
            h.check('trust_gates_cleared_by_nomotion_updates', True, dict(extra_updates=gate_updates))
            h.wait(lambda: h.state['trust'].get('trusted') is True, 60, 'trust READY/MOTION_OK')

        # --- 3. driver registration (explicit, once)
        r = h.trigger('motion_adapter/register')
        h.check('registration_requested_explicitly', r.success, r.message)
        h.wait(lambda: h.state['adapter'].get('registration') in ('registered', 'failed'), 20, 'registration result')
        h.check('mc_accepted_registration', h.state['adapter'].get('registration') == 'registered',
                h.state['adapter'].get('registration_error'))
        h.wait(lambda: h.state['coord'].get('ready', False), 180, 'coordinator ready')
        h.check('coordinator_ready_live_driver', h.state['coord'].get('driver_mode') == 'live', h.state['coord'])

        # --- 4. drive
        for label, target in (('out', goal), ('back', start)):
            hdl = h.send_goal(*target)
            h.check(f'goal_{label}_accepted', hdl.accepted, dict(coord=h.state['coord']) if not hdl.accepted else None, hard=False)
            if not hdl.accepted:               # e.g. the coordinator latched after a trust fault: a measurement, not a harness failure
                continue
            status = h.finish(hdl)
            h.check(f'goal_{label}_succeeded', status == SUCCEEDED, status, hard=False)
            h.wait(lambda: h.vel is not None and max(abs(v) for v in h.vel) < 0.01, 30, 'robot at rest')
            h.spin(1.0)
            xy, yaw = h.pose_error(target)
            h.check(f'goal_{label}_pose', xy <= args.xy_tol and yaw <= args.yaw_tol,
                    dict(pose=h.pose, xy_error=xy, yaw_error=yaw), hard=False)
            if label == 'out':
                h.check('mc_selected_our_source', str(mc.get('source', '')).startswith('sdk_'), dict(mc))
                h.check('mc_received_commands', len(mc_cmds) > 0, dict(n=len(mc_cmds)))

        # --- 5. invariants over the whole run
        h.check('mc_received_commands_overall', len(mc_cmds) > 0, dict(n=len(mc_cmds)))
        src = mc_cmds[0][0]
        h.check('command_limits', all(s == src and abs(v) <= 0.3 + 1e-6 and ly == 0.0 and abs(w) <= args.max_angular + 1e-6
                                      for s, v, ly, w in mc_cmds),
                dict(n=len(mc_cmds), vmax=max(abs(c[1]) for c in mc_cmds), wmax=max(abs(c[3]) for c in mc_cmds)))
        h.check('no_reverse', all(v >= -1e-9 for _, v, _, _ in mc_cmds))
        h.check('no_global_cmd_vel_publisher', not h.get_publishers_info_by_topic('/cmd_vel'))
        if args.localization == 'amcl':
            pos = sorted(e[0] for e in amcl_err)
            h.check('amcl_error_recorded', len(pos) >= 10,
                    dict(n=len(pos), median=pos[len(pos) // 2], p95=pos[int(len(pos) * .95)], max=pos[-1]))
        rc = 0
    except Exception as exc:                                   # noqa: BLE001 - recorded in results, exit code reflects it
        import traceback
        traceback.print_exc()
        print(f'SCENARIO ERROR: {exc!r}', file=sys.stderr)
        h.results.append(dict(name='scenario_error', ok=False, detail=repr(exc), hard=True))
    finally:
        out = pathlib.Path(args.results_dir) / args.name
        out.mkdir(parents=True, exist_ok=False)
        (out / 'trace.csv').write_text('t,x,y,yaw,v,w\n' + ''.join(','.join(map(str, r)) + '\n' for r in h.trace))
        (out / 'cmd_trace.csv').write_text('t,cmd_v,cmd_w\n' + ''.join(','.join(map(str, r)) + '\n' for r in cmd_trace))
        (out / 'results.json').write_text(json.dumps(dict(
            plant=args.plant, localization=args.localization, max_angular=args.max_angular, start=start, goal=goal, checks=h.results,
            failed=[r['name'] for r in h.results if not r['ok']], hard_failed=[r['name'] for r in h.results if not r['ok'] and r['hard']],
            events=h.events), indent=2, default=str))
        h.destroy_node()
        rclpy.try_shutdown()
    return rc


def _amcl_active(h, client) -> bool:
    fut = client.call_async(GetState.Request())
    h.wait(fut.done, 10, 'amcl get_state')
    return fut.result().current_state.id == 3


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--plant', choices=('official', 'lite'), default='official')
    ap.add_argument('--name', required=True, help='result directory name (must not exist)')
    ap.add_argument('--results-dir', default=os.environ.get('TWIN_RESULTS_DIR', '/results'))
    ap.add_argument('--start', default='-1.275,-4.825,0.0')
    ap.add_argument('--distance', type=float, default=1.5, help='goal distance ahead of START along +x, m')
    ap.add_argument('--localization', choices=('amcl', 'truth'), default='amcl', help='must match the stack launch argument')
    ap.add_argument('--max-angular', type=float, default=0.5)
    ap.add_argument('--xy-tol', type=float, default=0.10)
    ap.add_argument('--yaw-tol', type=float, default=0.10)
    sys.exit(run(ap.parse_args()))


if __name__ == '__main__':
    main()
