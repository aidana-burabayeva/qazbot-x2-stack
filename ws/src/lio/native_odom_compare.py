"""PASSIVE comparison of native leg odometry (native_imu footprint) with LIO -- step "odom_compare" of
reports/20261027-native-odom-nav2-design.md. No TF, no velocity, nothing on topics the stack consumes.

In : /aima/mc/leg_odometry   (pose of lidar_imu_chest_front in leg_odom, ~140-170 Hz, sensor-data QoS)
     /qazbot_lio/odom_level  (level_bridge base_footprint = LiDAR-IMU footprint, 10 Hz; BEST_EFFORT here so this
                              node never back-pressures the publisher)
Out (only under /qazbot_native/compare/):
  odom_imu         Odometry, frame native_odom, child native_imu_footprint, source stamps, decimated to
                   publish_rate_hz (50); twist = level_bridge 0.3 s difference; not published while native is
                   faulted (latched non-finite / origin reset, stale > native_max_age, future stamps dropped).
  lio_plausibility String JSON per matched LIO sample: lio_speed (1 s), native_still, flag (DIAGNOSTIC ONLY, not
                   wired into trust).
  status           String JSON at 1 Hz: rates, ages, faults, guard counters, increment residuals (1 s, 10 s),
                   cumulative path/yaw, standstill noise (5 s), plausibility summary.
Log (log_dir, default logs/native_compare_<time>): pairs.csv (every LIO sample matched with native interpolated at
its stamp), status.jsonl, summary.json at exit.

Math: native_odom.py (tested by tests/lio/test_native_odom.py). Run: tools/stack/native_compare.sh.
"""
import csv
import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

import native_odom as no

OUT_PREFIX = '/qazbot_native/compare/'
BACKEND = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def pose7(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class NativeOdomCompare(Node):
    def __init__(self):
        super().__init__('qazbot_native_odom_compare')
        P = lambda n, v: self.declare_parameter(n, v).value     # noqa: E731
        leg_topic, lio_topic = P('leg_topic', '/aima/mc/leg_odometry'), P('lio_topic', '/qazbot_lio/odom_level')
        rate = float(P('publish_rate_hz', 50.0))
        if not 20.0 <= rate <= 100.0:
            raise ValueError('publish_rate_hz must be 20..100')
        self.native = no.native_guard(float(P('native_max_age', no.NATIVE_MAX_AGE)))
        self.lio = no.lio_guard(float(P('lio_max_age', no.LIO_MAX_AGE)))
        self.cmp = no.Compare(plaus_speed=float(P('plaus_lio_speed', no.PLAUS_LIO_SPEED)),
                              still_m=float(P('native_still_m', no.NATIVE_STILL_M)),
                              still_rad=float(P('native_still_rad', no.NATIVE_STILL_RAD)),
                              hold_s=float(P('plaus_hold_s', no.PLAUS_HOLD_S)))
        self.twist, self.decim = no.Twist0p3(), no.Decimator(rate)
        self.duration = float(P('duration_s', 0.0))           # 0: until SIGINT
        log_dir = P('log_dir', '') or os.path.join(BACKEND, 'logs', 'native_compare_' + time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir
        self.f_csv = open(os.path.join(log_dir, 'pairs.csv'), 'w', newline='')
        self.csv = csv.DictWriter(self.f_csv, fieldnames=no.Compare.ROW_KEYS)
        self.csv.writeheader()
        self.f_status = open(os.path.join(log_dir, 'status.jsonl'), 'w')
        topics = {k: OUT_PREFIX + k for k in ('odom_imu', 'lio_plausibility', 'status')}
        assert all(t.startswith(OUT_PREFIX) for t in topics.values())
        self.pub_odom = self.create_publisher(Odometry, topics['odom_imu'], 20)
        self.pub_plaus = self.create_publisher(String, topics['lio_plausibility'], 20)
        self.pub_status = self.create_publisher(String, topics['status'], 10)
        self.create_subscription(Odometry, leg_topic, self.on_leg, qos_profile_sensor_data)
        self.create_subscription(Odometry, lio_topic, self.on_lio, QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.n = dict(leg=0, lio=0, published=0, rows=0)
        self.n_prev, self.t_prev = dict(self.n), time.monotonic()
        self.t_start = time.monotonic()
        self.create_timer(1.0, self.on_status)
        self.get_logger().info(f'native_odom_compare started (passive, no TF): {leg_topic} vs {lio_topic} -> '
                               f'{OUT_PREFIX}*, {rate:g} Hz, log {log_dir}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_leg(self, m):
        self.n['leg'] += 1
        now, stamp = self.now(), stamp_s(m.header)
        st = self.native.add(stamp, m.header.frame_id, m.child_frame_id, pose7(m), now)
        if st == 'stamp_back':
            self.get_logger().warn('leg odometry stamp went back: clearing twist baseline and compare history')
            self.twist.clear()
            self.cmp.clear()
        elif st != 'ok':
            return
        _, x, y, z, yaw = self.native.pose
        self.cmp.add_native(stamp, x, y, yaw)
        tw = self.twist.add(stamp, x, y, yaw)
        if not self.decim.due(stamp) or tw is None or self.native.fault(now):
            return
        o = Odometry()
        o.header.stamp = m.header.stamp                       # source stamp, never re-stamped
        o.header.frame_id, o.child_frame_id = 'native_odom', 'native_imu_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(x), float(y), float(z)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.pose.covariance = no.cov_planar(m.pose.covariance)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = (float(v) for v in tw)
        self.pub_odom.publish(o)
        self.n['published'] += 1

    def on_lio(self, m):
        self.n['lio'] += 1
        now, stamp = self.now(), stamp_s(m.header)
        st = self.lio.add(stamp, m.header.frame_id, m.child_frame_id, pose7(m), now)
        if st == 'stamp_back':
            self.get_logger().warn('LIO stamp went back: clearing compare history')
            self.cmp.clear()
        elif st != 'ok':
            return
        _, x, y, _, yaw = self.lio.pose
        for row in self.cmp.add_lio(stamp, x, y, yaw):
            self.n['rows'] += 1
            self.csv.writerow(row)
            self.pub_plaus.publish(String(data=json.dumps(dict(
                t=row['t'], lio_speed=row['lio_speed'], native_still=row['native_still'], cond=row['plaus_cond'],
                lio_implausible=row['lio_implausible'], threshold=self.cmp.plaus_speed, diagnostic_only=True))))

    def on_status(self):
        now, mono = self.now(), time.monotonic()
        dt = max(1e-6, mono - self.t_prev)
        rates = {k: round((self.n[k] - self.n_prev[k]) / dt, 1) for k in self.n}
        self.n_prev, self.t_prev = dict(self.n), mono
        last = self.cmp.last or {}
        s = dict(t=now, mode='compare', publish_tf=False, rates_hz=rates, messages=dict(self.n),
                 age_s=dict(native=round(now - self.native.stamp, 4) if self.native.pose else None,
                            lio=round(now - self.lio.stamp, 4) if self.lio.pose else None),
                 fault=dict(native=self.native.fault(now), lio=self.lio.fault(now)),
                 guard=dict(native=self.native.counts, lio=self.lio.counts,
                            native_last_drop=self.native.last_drop, lio_last_drop=self.lio.last_drop),
                 latest={k: last.get(k) for k in ('t', 'res1_d', 'res1_yaw', 'res10_d', 'res10_yaw', 'lio_speed',
                                                  'native_still', 'lio_implausible')},
                 still_noise_5s=self.cmp.still_noise(self.lio.stamp) if self.lio.pose else None,
                 **self.cmp.summary())
        txt = json.dumps(s)
        self.pub_status.publish(String(data=txt))
        self.f_status.write(txt + '\n')
        self.f_status.flush()
        self.f_csv.flush()
        if self.duration and mono - self.t_start >= self.duration:
            self.get_logger().info(f'duration {self.duration:g} s reached')
            raise SystemExit

    def close(self):
        with open(os.path.join(self.log_dir, 'summary.json'), 'w') as f:
            json.dump(dict(messages=self.n, guard=dict(native=self.native.counts, lio=self.lio.counts),
                           latched=dict(native=self.native.latched, lio=self.lio.latched), **self.cmp.summary()),
                      f, indent=1)
        self.f_csv.close()
        self.f_status.close()


def main():
    rclpy.init()
    node = NativeOdomCompare()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
