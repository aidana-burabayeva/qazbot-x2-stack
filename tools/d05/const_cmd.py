#!/usr/bin/env python3
"""D-05 fixed-command source: constant vx (wz = 0) for a fixed duration through the existing motion adapter path.

Entry point: /qazbot_d1_map/cmd_vel_authorized (geometry_msgs/Twist), the topic x2_motion_adapter.py consumes. In
production only goal_coordinator publishes there; this tool is a temporary stand-in for it and DOES NOT touch the
adapter, Goal Coordinator, Trust or priorities. It never publishes on /aima/mc/locomotion/velocity, never calls
any service (arming is a separate, explicit step, see README.md) and refuses to start unless the adapter reports
live + registered + armed + idle + ready, the RC is quiet and nobody else drives the command topic.

Timing: duration INCLUDES the settle phase. Messages go out at --rate (default 20 Hz = Nav2 controller and
velocity_smoother frequency). At the end, on SIGINT/SIGTERM, on adapter latch / status loss / RC activity and on
any exception an explicit zero Twist burst is sent. Start/settle_end/stop/abort stamps go to a CSV.
"""
import argparse
import csv
import datetime
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import const_cmd_core as core  # noqa: E402

NS = '/qazbot_d1_map'
CSV_FIELDS = ['run_id', 'event', 'wall_iso', 'ros_time_ns', 'mono_s', 'vx_cmd', 'duration_s', 'settle_s', 'rate_hz',
              'msgs_sent', 'reason', 'adapter_state', 'adapter_mc_source']

EXIT_OK, EXIT_REFUSED, EXIT_ABORTED, EXIT_ERROR, EXIT_LIMIT = 0, 3, 4, 5, 2


class CsvLog:
    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.new = not os.path.exists(path) or os.path.getsize(path) == 0

    def write(self, **row):
        with open(self.path, 'a', newline='') as f:
            w = csv.DictWriter(f, CSV_FIELDS)
            if self.new:
                w.writeheader()
                self.new = False
            w.writerow({k: row.get(k, '') for k in CSV_FIELDS})
            f.flush()
            os.fsync(f.fileno())


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f'Hard limits: |vx| in [{core.MIN_ABS_VX}, {core.MAX_ABS_VX}] m/s, duration <= {core.MAX_DURATION_S} s '
               f'(settle included), rate in [{core.NAV2_RATE_HZ}, {core.MAX_RATE_HZ}] Hz. Exit codes: 0 ok, '
               '2 limits, 3 refused to start, 4 aborted during run (zero sent), 5 error (zero sent).')
    ap.add_argument('--vx', type=float, required=True, help=f'constant forward velocity [m/s], |vx| <= {core.MAX_ABS_VX}')
    ap.add_argument('--duration', type=float, required=True,
                    help=f'total command duration [s] INCLUDING the settle phase, <= {core.MAX_DURATION_S}')
    ap.add_argument('--settle', type=float, default=core.DEFAULT_SETTLE_S,
                    help='settle phase at the start of the duration [s]; same command, only a settle_end stamp is logged '
                         '(default %(default)s)')
    ap.add_argument('--rate', type=float, default=core.NAV2_RATE_HZ,
                    help='publish rate [Hz] (default %(default)s = Nav2 controller/smoother rate)')
    ap.add_argument('--run-id', default='', help='free-text label written to every CSV row (e.g. run number)')
    ap.add_argument('--log-csv', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs',
                                                      'const_cmd_log.csv'),
                    help='CSV file for start/settle_end/stop/abort stamps, appended (default %(default)s)')
    ap.add_argument('--cmd-topic', default=f'{NS}/cmd_vel_authorized',
                    help='Twist topic consumed by the motion adapter (default %(default)s)')
    ap.add_argument('--adapter-status-topic', default=f'{NS}/motion_adapter/status',
                    help='adapter status (std_msgs/String JSON) (default %(default)s)')
    ap.add_argument('--gc-status-topic', default=f'{NS}/goal_coordinator/status',
                    help='Goal Coordinator status; if present it must be not busy/latched (default %(default)s)')
    ap.add_argument('--mc-velocity-topic', default='/aima/mc/locomotion/velocity',
                    help='observed only (never published) to detect RC activity (default %(default)s)')
    ap.add_argument('--preflight-timeout', type=float, default=5.0,
                    help='max time to collect status/RC observations before refusing [s] (default %(default)s)')
    ap.add_argument('--check-only', action='store_true',
                    help='run the start checks and exit without publishing any command')
    return ap


def utc_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        params = core.validate_params(args.vx, args.duration, args.settle, args.rate)
    except core.LimitError as e:
        print(f'LIMIT: {e}', file=sys.stderr)
        return EXIT_LIMIT
    schedule = core.Schedule(params)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    try:
        from aimdk_msgs.msg import McLocomotionVelocity
    except ImportError:
        print('REFUSED: aimdk_msgs not importable (source aimdk_env.sh); the RC-activity check cannot run, failing closed',
              file=sys.stderr)
        return EXIT_REFUSED

    stop = {'signal': None}

    def on_signal(signum, _frame):
        stop['signal'] = signal.Signals(signum).name

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('d05_const_cmd')
    log = node.get_logger()
    csv_log = CsvLog(args.log_csv)
    st = {'adapter': None, 'adapter_at': None, 'gc': None, 'rc_nz_at': None, 'sent': 0}

    def mono():
        return time.monotonic()

    def age(t):
        return None if t is None else mono() - t

    node.create_subscription(String, args.adapter_status_topic, lambda m: st.update(
        adapter=core.parse_status(m.data), adapter_at=mono()), 10)
    node.create_subscription(String, args.gc_status_topic, lambda m: st.update(gc=core.parse_status(m.data)), 10)

    def on_mc(m):
        if m.source == 'rc' and any(abs(v) > 1e-6 for v in (m.forward_velocity, m.lateral_velocity, m.angular_velocity)):
            st['rc_nz_at'] = mono()
    node.create_subscription(McLocomotionVelocity, args.mc_velocity_topic, on_mc,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    pub = node.create_publisher(Twist, args.cmd_topic, 1)

    def spin_for(deadline_mono, until=None):
        while stop['signal'] is None and rclpy.ok():
            remaining = deadline_mono - mono()
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.02, remaining))   # never sleep past the next tick
            if until and until():
                return True
        return until() if until else False

    def twist(vx):
        m = Twist()
        m.linear.x = float(vx)
        return m

    def send_zeros():
        """Explicit zero burst; never raises. Blocking by design: this is the shutdown path."""
        period = 1.0 / params.rate_hz
        for _ in range(core.ZERO_COUNT):
            try:
                pub.publish(twist(0.0))
            except Exception:
                break
            time.sleep(period)

    def snapshot():
        a = st['adapter'] or {}
        return a.get('state', ''), a.get('mc_source', '')

    def row(event, t0=None, reason=''):
        s, src = snapshot()
        csv_log.write(run_id=args.run_id, event=event, wall_iso=utc_iso(),
                      ros_time_ns=node.get_clock().now().nanoseconds, mono_s=f'{mono():.6f}', vx_cmd=params.vx,
                      duration_s=params.duration_s, settle_s=params.settle_s, rate_hz=params.rate_hz,
                      msgs_sent=st['sent'], reason=reason, adapter_state=s, adapter_mc_source=src)

    rc = EXIT_ERROR
    started = False
    try:
        # ---- start conditions: collect observations until all refusals clear or the timeout expires
        deadline = mono() + args.preflight_timeout
        t_listen_end = mono() + core.RC_QUIET_S + 0.2        # always listen to RC traffic for a full quiet window

        def refusals():
            others = sorted({i.node_name for i in node.get_publishers_info_by_topic(args.cmd_topic)
                             if i.node_name != 'goal_coordinator' and i.node_name != node.get_name()})
            r = core.preflight_refusals(st['adapter'], age(st['adapter_at']), st['gc'], others, age(st['rc_nz_at']),
                                        node.count_subscribers(args.cmd_topic))
            if mono() < t_listen_end:
                r.append('listening for RC activity')
            return r

        spin_for(deadline, until=lambda: not refusals())
        if stop['signal']:
            row('abort', reason='signal ' + stop['signal'])
            return EXIT_ABORTED
        why = refusals()
        if why:
            log.error('REFUSED to start: ' + '; '.join(why))
            row('refused', reason='; '.join(why))
            return EXIT_REFUSED
        if args.check_only:
            log.info('start checks passed (--check-only): nothing published')
            return EXIT_OK

        # ---- run
        t0 = mono()
        started = True
        row('start', reason=f'vx={params.vx} duration={params.duration_s} settle={params.settle_s}')
        log.info(f'START vx={params.vx} m/s for {params.duration_s} s (settle {params.settle_s} s) at {params.rate_hz} Hz')
        settle_logged, k, reason = False, 0, ''
        while True:
            now = mono()
            t = now - t0
            if stop['signal']:
                reason = 'signal ' + stop['signal']
                break
            if schedule.phase(t) == 'done':
                break
            if not settle_logged and schedule.phase(t) == 'steady':
                settle_logged = True
                row('settle_end')
            reason = core.abort_reason(t, st['adapter'], age(st['adapter_at']), age(st['rc_nz_at']))
            if reason:
                break
            pub.publish(twist(schedule.command(t)[0]))
            st['sent'] += 1
            k += 1
            spin_for(t0 + k / params.rate_hz)               # next tick on the nominal grid: no drift accumulation
        row('cmd_end', reason=reason or 'duration elapsed')   # stamped BEFORE the zero burst: last command instant
        send_zeros()
        row('stop' if not reason else 'abort', reason=reason or 'duration elapsed')
        started = False
        if reason:
            log.error(f'ABORT: {reason}; zero sent')
            rc = EXIT_ABORTED
        else:
            log.info(f'STOP after {params.duration_s} s, {st["sent"]} command messages, zero sent')
            rc = EXIT_OK
        return rc
    except BaseException as e:          # any exception (incl. KeyboardInterrupt): zero first, then report
        send_zeros()
        if started:
            try:
                row('abort', reason=f'exception {type(e).__name__}: {e}')
            except Exception:
                pass
        log.error(f'ERROR {type(e).__name__}: {e}; zero sent')
        return EXIT_ERROR
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
