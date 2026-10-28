#!/usr/bin/env python3
"""SIM-container only. One velocity step run against the official MC in the twin.

Replicates the official py_examples/mc_locomotion_velocity.py command path exactly
(topic /aima/mc/locomotion/velocity, source 'node.locomotion_velocity', priority 40,
timeout 1000, ADD -> ENABLE fallback, 50 Hz timer) but WITHOUT its client-side
min-speed check (fwd >= 0.2, yaw >= 0.1), so sub-threshold commands can be tested.

Profile: PRE s of zero commands, CMD_S s of (fwd, lat, ang) at 50 Hz, POST s of zeros.
Records /aima/hal/odom/state (t, x, y, z, qx, qy, qz, qw, vx, vy, wz) and every published
command (t, fwd, lat, ang) on one monotonic clock. Saves OUT.npz.

Usage: sim_step_run.py OUT.npz FWD LAT ANG [CMD_S=5] [PRE=1] [POST=4]
"""
import os, sys, time
if any(os.environ.get(k) != v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):
    raise RuntimeError('SIM only')
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from aimdk_msgs.msg import McLocomotionVelocity, McInputAction, MessageHeader
from aimdk_msgs.srv import SetMcInputSource

SOURCE = os.environ.get('QAZBOT_PROBE_SOURCE', 'node.locomotion_velocity')
PRIORITY = int(os.environ.get('QAZBOT_PROBE_PRIORITY', '40'))
if SOURCE.startswith('sdk_') and not 20 <= PRIORITY <= 39:
    raise ValueError('SDK source priority outside 20..39')
out = sys.argv[1]
if os.path.exists(out):
    raise RuntimeError('refusing to overwrite existing trial')
F, L, A = (float(v) for v in sys.argv[2:5])
CMD_S = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0
PRE = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0
POST = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0

rclpy.init()
n = Node('sim_step_run')
pub = n.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', 10)
cli = n.create_client(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
odom, cmds = [], []
t0 = time.monotonic()


def on_odom(m):
    p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
    odom.append((time.monotonic() - t0, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z))


n.create_subscription(Odometry, '/aima/hal/odom/state', on_odom, qos_profile_sensor_data)


def call_src(action):
    req = SetMcInputSource.Request()
    req.action.value = action
    req.input_source.name = SOURCE
    req.input_source.priority = PRIORITY
    req.input_source.timeout = 1000
    fut = None
    for _ in range(8):
        req.request.header.stamp = n.get_clock().now().to_msg()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=0.25)
        if fut.done():
            break
    if fut is None or not fut.done():
        return False
    r = fut.result()
    return r is not None and r.response.header.code == 0


if not cli.wait_for_service(timeout_sec=8.0):
    print('ERR no SetMcInputSource service'); sys.exit(2)
reg = 'ADD' if call_src(McInputAction.INPUTACTION_ADD) else ('ENABLE' if call_src(McInputAction.INPUTACTION_ENABLE) else 'FAIL')
if reg == 'FAIL':
    print('ERR input source registration failed'); sys.exit(3)

start = time.monotonic() - t0
t_on, t_off, t_end = start + PRE, start + PRE + CMD_S, start + PRE + CMD_S + POST


def tick():
    t = time.monotonic() - t0
    f, l, a = (F, L, A) if t_on <= t < t_off else (0.0, 0.0, 0.0)
    m = McLocomotionVelocity()
    m.header = MessageHeader()
    m.header.stamp = n.get_clock().now().to_msg()
    m.source = SOURCE
    m.forward_velocity, m.lateral_velocity, m.angular_velocity = f, l, a
    pub.publish(m)
    cmds.append((t, f, l, a))


n.create_timer(0.02, tick)
while time.monotonic() - t0 < t_end:
    rclpy.spin_once(n, timeout_sec=0.005)

np.savez_compressed(out, odom=np.array(odom, dtype=np.float64), cmd=np.array(cmds, dtype=np.float64),
                    meta=np.array([F, L, A, CMD_S, PRE, POST, t_on, t_off, t_end]),
                    odom_cols=np.array('t x y z qx qy qz qw vx vy wz'.split()), cmd_cols=np.array('t fwd lat ang'.split()),
                    reg=np.array(reg), source=np.array(SOURCE), priority=np.array(PRIORITY))
print('OK reg=%s odom=%d cmd=%d t_on=%.3f t_off=%.3f' % (reg, len(odom), len(cmds), t_on, t_off))
n.destroy_node(); rclpy.shutdown()
