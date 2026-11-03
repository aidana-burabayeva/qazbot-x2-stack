"""odom -> base_footprint from MC leg odometry (odom_source:=native_imu) -- step 1 of
reports/20261027-native-odom-nav2-design.md (sections 3.4-3.6).

base_footprint keeps its level_bridge definition (the LiDAR-IMU lidar_imu_chest_front projected on the floor, yaw =
heading of the LiDAR forward axis), so /qazbot_lio/scan, the AMCL laser pose and the trust frames stay valid. Only the
SOURCE of the pose changes: /aima/mc/leg_odometry (pose of lidar_imu_chest_front in the gravity-levelled leg_odom,
~140-170 Hz, PTP-stamped) instead of FAST-LIO (which drifted 1.77 m during an in-place turn, trial06).
The `odom` frame of the TF tree is leg_odom in this mode; its origin is wherever MC put it.

In : /aima/mc/leg_odometry (sensor-data QoS).
Out: TF odom -> base_footprint (launch remaps /tf -> /qazbot_d1_map/tf), only with publish_tf:=true:
       x, y, yaw only (z = 0, roll = pitch = 0, as level_bridge);
     /qazbot_native/odom  Odometry odom / base_footprint: same pose + IMU z, twist = level_bridge 0.3 s pose
       difference in base_footprint axes (none until the baseline is full; the vendor twist is not used: its
       angular part is always 0), pose covariance = leg covariance diagonal, twist covariance = NOMINAL diagonal
       (not a measurement; no consumer reads it);
     /qazbot_native/status String JSON 1 Hz: mode, rates, age, fault/latch reason, guard counters.
Both outputs carry the SOURCE stamp of the leg sample (never re-stamped) and are decimated to publish_rate_hz
(default 30, newest sample on source-stamp phase; TF consumers are Python nodes, AMCL interpolates at the scan stamp).

Guards (native_odom.SourceGuard; any of them -> nothing published, trust sees T1_odom_stale after 0.5 s = fail-closed):
  frame_id != leg_odom or child != lidar_imu_chest_front  dropped
  non-finite / non-unit quaternion                         LATCHED until restart
  origin reset (|dp| > 0.05 m + 1.5 m/s * dt or |dyaw| > 0.3 rad between samples)   LATCHED until restart
  stamp more than 0.2 s in the future                      dropped
  sample older than max_age (0.2 s) when due               withheld
  stamp went back                                          accepted, twist baseline cleared (as level_bridge)

Standing hold (odom_hold.py, reports/20261103-odom-standing-hold.md; parameter standing_hold, default true): while MC
reports standing (/aima/mc/common/state fsm 4, STAND_DEFAULT, RUNNING, body STAND) AND the raw leg pose is quiet over
0.5 s, the published x, y are frozen (the vendor standing drift of ~1 cm/min is absorbed into a constant offset, so
there is no jump when the hold ends); yaw is never held; the Odometry twist is zero while held. No state / no aimdk_msgs
= no hold = the pass-through above. Guards act on the raw leg pose, before the hold.

Exactly one node may own odom -> base_footprint: localization.launch.py starts this one only with
odom_source:=native_imu and then runs level_bridge with publish_tf:=false. Math: native_odom.py.
"""
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import native_odom as no
import odom_hold as oh

MODES = ('native_imu',)            # native_base (base_link footprint) is step 2: needs the scan in that frame
ODOM_FRAME, BASE_FRAME = 'odom', 'base_footprint'
RATE_RANGE = (10.0, 100.0)
#: Nominal twist variance (m/s)^2 / (rad/s)^2 on the diagonal -- documented placeholder, not measured.
TWIST_VAR_NOMINAL = 1e-3


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def pose7(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class NativeOdomBridge(Node):
    def __init__(self, **kwargs):          # kwargs: rclpy Node options (tests: parameter_overrides)
        super().__init__('qazbot_native_odom_bridge', **kwargs)
        P = lambda n, v: self.declare_parameter(n, v).value     # noqa: E731
        self.mode = P('mode', 'native_imu')
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}, got {self.mode!r} (native_base is not implemented)')
        self.publish_tf = bool(P('publish_tf', True))
        # dynamic typing: 'publish_rate_hz:=30' (int) and ':=30.0' are both accepted
        rate = float(self.declare_parameter('publish_rate_hz', 30.0,
                                            ParameterDescriptor(dynamic_typing=True)).value)
        if not RATE_RANGE[0] <= rate <= RATE_RANGE[1]:
            raise ValueError(f'publish_rate_hz must be {RATE_RANGE[0]:g}..{RATE_RANGE[1]:g}, got {rate:g}')
        self.rate = rate
        leg_topic = P('leg_topic', '/aima/mc/leg_odometry')
        odom_topic = P('odom_topic', '/qazbot_native/odom')
        status_topic = P('status_topic', '/qazbot_native/status')
        self.guard = no.native_guard(float(P('max_age', no.NATIVE_MAX_AGE)))
        self.twist, self.decim = no.Twist0p3(), no.Decimator(rate)
        self.hold = oh.StandingHold() if bool(P('standing_hold', True)) else None
        state_topic = P('state_topic', '/aima/mc/common/state')
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 20)
        self.pub_status = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Odometry, leg_topic, self.on_leg, qos_profile_sensor_data)
        if self.hold is not None:
            try:
                from aimdk_msgs.msg import McCommonState
                self.create_subscription(McCommonState, state_topic, self.on_state, qos_profile_sensor_data)
            except ImportError as e:
                self.get_logger().error(f'standing hold ENABLED but aimdk_msgs import failed ({e}): hold DISABLED, odometry is pass-through (status hold=null)')
                self.hold = None
        self.n = dict(leg=0, accepted=0, tf=0, odom=0, withheld_stale=0)
        self.n_prev, self.t_prev = dict(self.n), time.monotonic()
        self.latch_logged = False
        self.last_pub_stamp = None
        self.create_timer(1.0, self.on_status)
        self.get_logger().info(
            f'native_odom_bridge started: mode={self.mode} publish_tf={self.publish_tf} {leg_topic} -> '
            f'{"TF odom->base_footprint + " if self.publish_tf else ""}{odom_topic}, {rate:g} Hz, '
            f'max_age={self.guard.max_age:g} s standing_hold={self.hold is not None}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        self.hold.set_state(self.now(), m.fsm_state.current_state, m.action_info.action_desc,
                            m.action_info.status.value, m.body_status.value)

    def on_leg(self, m):
        self.n['leg'] += 1
        now, stamp = self.now(), stamp_s(m.header)
        st = self.guard.add(stamp, m.header.frame_id, m.child_frame_id, pose7(m), now)
        if st == 'stamp_back':
            self.get_logger().warn('leg odometry stamp went back: clearing twist baseline')
            self.twist.clear()
            if self.hold is not None:
                self.hold.clear_window()
        elif st != 'ok':
            if self.guard.latched and not self.latch_logged:
                self.latch_logged = True
                self.get_logger().error(f'leg odometry LATCHED ({self.guard.latched}): no TF / odom until restart')
            return
        self.n['accepted'] += 1
        _, x, y, z, yaw = self.guard.pose
        held = False
        if self.hold is not None:
            x, y, held = self.hold.update(stamp, now, x, y, yaw)
        tw = self.twist.add(stamp, x, y, yaw)
        if held and tw is not None:
            tw = (0.0, 0.0, 0.0)
        if not self.decim.due(stamp):
            return
        if self.guard.fault(now):
            self.n['withheld_stale'] += 1
            return
        yaw = no.wrap(yaw)
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = m.header.stamp                   # source stamp, never re-stamped
            t.header.frame_id, t.child_frame_id = ODOM_FRAME, BASE_FRAME
            t.transform.translation.x, t.transform.translation.y = float(x), float(y)
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)
            self.n['tf'] += 1
        self.last_pub_stamp = stamp
        if tw is None:
            return                     # no velocity yet: publish no Odometry rather than a zero twist
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id, o.child_frame_id = ODOM_FRAME, BASE_FRAME
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(x), float(y), float(z)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.pose.covariance = no.cov_planar(m.pose.covariance)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = (float(v) for v in tw)
        o.twist.covariance = [TWIST_VAR_NOMINAL if i % 7 == 0 else 0.0 for i in range(36)]
        self.pub_odom.publish(o)
        self.n['odom'] += 1

    def status(self):
        now, mono = self.now(), time.monotonic()
        dt = max(1e-6, mono - self.t_prev)
        rates = {k: round((self.n[k] - self.n_prev[k]) / dt, 1) for k in self.n}
        self.n_prev, self.t_prev = dict(self.n), mono
        g = self.guard
        return dict(t=now, mode=self.mode, publish_tf=self.publish_tf, publish_rate_hz=self.rate,
                    frames=[ODOM_FRAME, BASE_FRAME], rates_hz=rates, messages=dict(self.n),
                    age_s=round(now - g.stamp, 4) if g.pose else None,
                    last_published_age_s=None if self.last_pub_stamp is None else round(now - self.last_pub_stamp, 4),
                    fault=g.fault(now), latched=g.latched, guard=dict(g.counts), last_drop=g.last_drop,
                    hold=None if self.hold is None else dict(held=self.hold.held, standing=self.hold.standing,
                                                             offset_m=round(self.hold.offset(), 5),
                                                             **self.hold.counts))

    def on_status(self):
        self.pub_status.publish(String(data=json.dumps(self.status())))


def main():
    rclpy.init()
    node = NativeOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                    # SIGINT / external shutdown: clean exit, no traceback
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)    # launch + process-group SIGINT arrive twice: no 2nd interrupt in cleanup
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
