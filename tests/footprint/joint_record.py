#!/usr/bin/env python3
"""Passive kinematics recorder for the walking footprint. Subscribes only; publishes nothing.

Keeps the newest joint states and the base_link IMU orientation and stores one snapshot at
SNAP_HZ (default 50 Hz), instead of the ~500 Hz stream (the HAL state topics are shared with
MC; ros2_control for X2 also keeps only the newest sample for that reason).
Run on the robot with AMENT_PREFIX_PATH/PYTHONPATH/LD_LIBRARY_PATH pointing at /agibot/software/common.
Usage: joint_record.py OUT.npz SECONDS [SNAP_HZ]
"""
import sys
import signal
import time

import numpy as np
import rclpy
from aimdk_msgs.msg import JointStateArray
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

NAMES = None


class Rec(Node):
    def __init__(self, hz):
        super().__init__('qazbot_joint_record')
        self.q, self.imu, self.t, self.Q, self.I = {}, None, [], [], []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for part in ('arm', 'leg', 'waist', 'head'):
            self.create_subscription(JointStateArray, f'/aima/hal/joint/{part}/state', self.on_joints, qos)
        self.create_subscription(Imu, '/aima/hal/imu/torso/state', self.on_imu, qos)
        self.create_timer(1.0 / hz, self.snap)

    def on_joints(self, m):
        for j in m.joints:
            self.q[j.name] = j.position

    def on_imu(self, m):
        o = m.orientation
        self.imu = (o.x, o.y, o.z, o.w)

    def snap(self):
        global NAMES
        if self.imu is None or len(self.q) < 31:
            return
        if NAMES is None:
            NAMES = sorted(self.q)
        self.t.append(time.time())
        self.Q.append([self.q[n] for n in NAMES])
        self.I.append(self.imu)


# Signal handlers only set a flag, avoiding KeyboardInterrupt inside pybind take_message.
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
stop_requested = False
def request_stop(signum, frame):
    global stop_requested
    stop_requested = True
signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
hz = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0
n = Rec(hz)
end = time.time() + float(sys.argv[2])
try:
    while time.time() < end and not stop_requested and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.01)
except KeyboardInterrupt:
    pass          # SIGINT from stop_motion.sh: still save what was recorded
np.savez(sys.argv[1], t=np.array(n.t), names=np.array(NAMES or []), q=np.array(n.Q), imu_xyzw=np.array(n.I))
print('snapshots', len(n.t), 'joints', len(NAMES or []))

n.destroy_node()
rclpy.shutdown()
