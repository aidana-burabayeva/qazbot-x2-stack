#!/usr/bin/env python3
"""Candidate native_odom_bridge with the standing hold, in-process (isolated domain 88, localhost; callbacks fed
directly, publishers captured, clock injected: no sleeps). Needs the navigation env (rclpy + aimdk_msgs).
Run: bash -c 'source ws/src/navigation/env.sh && source ws/src/navigation/aimdk_env.sh && env -u FASTRTPS_DEFAULT_PROFILES_FILE \
 ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q candidates/odom_hold/test_odom_hold_bridge.py'
"""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(HERE, '..', '..', 'ws', 'src', 'lio'))        # native_odom.py
os.environ.setdefault('ROS_DOMAIN_ID', '88')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('aimdk_msgs')
from aimdk_msgs.msg import McCommonState  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

import native_odom_bridge as nb  # noqa: E402

assert os.path.dirname(os.path.abspath(nb.__file__)) == HERE, 'must test the candidate, not the deployed bridge'
HZ = 150.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    if os.environ['ROS_DOMAIN_ID'] == '0':
        raise RuntimeError('never run on domain 0 (live stack)')
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.try_shutdown()


class Rig:
    def __init__(self, **params):
        self.node = nb.NativeOdomBridge(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()],
                                        cli_args=['--ros-args', '-r', '__node:=test_hold_bridge'])
        self.tf, self.odom = [], []
        self.node.tf.sendTransform = self.tf.append
        self.node.pub_odom.publish = self.odom.append
        self.t, self.x, self.y = 1000.0, 2.0, 3.0
        self.raw = {}                                # (sec, nanosec) -> raw (x, y); TF is decimated to 30 Hz
        self.node.now = lambda: self.t

    def state(self, fsm=4, action='STAND_DEFAULT', status=100, body=1):
        m = McCommonState()
        m.fsm_state.current_state, m.action_info.action_desc = fsm, action
        m.action_info.status.value, m.body_status.value = status, body
        self.node.on_state(m)

    def run(self, secs, vx=0.0, vy=0.0, state=None):
        for i in range(int(round(secs * HZ))):
            self.t += 1 / HZ
            self.x, self.y = self.x + vx / HZ, self.y + vy / HZ
            if state is not None and self.node.hold is not None and i % 15 == 0:
                self.state(*state)
            m = Odometry()
            m.header.stamp.sec, m.header.stamp.nanosec = int(self.t), int(round((self.t % 1) * 1e9))
            m.header.frame_id, m.child_frame_id = 'leg_odom', 'lidar_imu_chest_front'
            m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
            q = m.pose.pose.orientation          # leg_odom pose of the chest LiDAR IMU (11-02 bag), upright
            q.x, q.y, q.z, q.w = 0.117782068736791, -0.685816559248654, 0.13319875385597296, 0.7057202868811658
            self.raw[(m.header.stamp.sec, m.header.stamp.nanosec)] = (self.x, self.y)
            self.node.on_leg(m)

    def raw_of(self, tf):
        return self.raw[(tf.header.stamp.sec, tf.header.stamp.nanosec)]


def test_standing_hold_freezes_tf_and_zeroes_twist():
    r = Rig()
    r.run(5.0, vy=1.7e-4, state=(4, 'STAND_DEFAULT', 100, 1))
    assert r.node.hold.held
    last = [t.transform.translation.y for t in r.tf[-20:]]
    assert max(last) - min(last) == 0.0                               # frozen published y
    assert all(o.twist.twist.linear.x == 0.0 and o.twist.twist.linear.y == 0.0 and o.twist.twist.angular.z == 0.0
               for o in r.odom[-20:])
    assert r.y - last[-1] > 5e-4                                      # raw kept creeping
    assert r.node.status()['hold']['held'] is True


def test_walking_and_no_state_are_pass_through():
    for state in (None, (5, 'LOCOMOTION_DEFAULT', 100, 1)):
        r = Rig()
        r.run(3.0, vy=1.7e-4, state=state)
        assert not r.node.hold.held
        assert all(r.raw_of(t) == (t.transform.translation.x, t.transform.translation.y) for t in r.tf)   # raw
    r.run(1.0, vx=0.3, state=(5, 'LOCOMOTION_DEFAULT', 100, 1))
    assert r.raw_of(r.tf[-1])[0] == r.tf[-1].transform.translation.x
    assert abs(r.odom[-1].twist.twist.linear.x) > 0.2                          # real twist while walking


def test_hold_disabled_by_parameter():
    r = Rig(standing_hold=False)
    assert r.node.hold is None
    r.run(2.0, vy=1.7e-4, state=(4, 'STAND_DEFAULT', 100, 1))                  # on_state never reached: no subscription
    assert r.raw_of(r.tf[-1])[1] == r.tf[-1].transform.translation.y
    assert r.node.status()['hold'] is None


def test_exit_continuous_after_hold():
    r = Rig()
    r.run(5.0, vy=1.7e-4, state=(4, 'STAND_DEFAULT', 100, 1))
    before = r.tf[-1].transform.translation
    n = len(r.tf)
    r.run(1.0, vx=0.3, state=(5, 'LOCOMOTION_DEFAULT', 100, 1))
    first = r.tf[n].transform.translation
    assert math.hypot(first.x - before.x, first.y - before.y) < 0.02          # one 30 Hz step of 0.3 m/s = 1 cm
    assert abs(r.node.status()['hold']['offset_m'] - 1.7e-4 * 5) < 3e-4
