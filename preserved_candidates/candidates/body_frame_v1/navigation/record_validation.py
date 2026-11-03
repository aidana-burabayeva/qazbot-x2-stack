#!/usr/bin/env python3
"""Finite bag recording for operator-controlled validation; no robot commands."""
import argparse
import signal
import subprocess
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
p.add_argument('--duration', type=float, default=60, help='Diagnostic capture window, seconds')
p.add_argument('--raw', action='store_true', help='Include raw cloud for offline LIO replay')
a = p.parse_args()
if a.duration <= 0 or a.output.exists():
    p.error('Duration must be positive and output must not exist')
topics = ['/qazbot_lio/scan', '/qazbot_lio/odometry', '/qazbot_lio/odom_level',
          '/qazbot_d1_map/tf', '/qazbot_d1_map/amcl_pose', '/qazbot_d1_map/map',
          '/aima/mc/leg_odometry', '/tf', '/tf_static',
          '/aima/hal/sensor/lidar_chest_front/imu']
if a.raw:
    topics.append('/aima/hal/sensor/lidar_chest_front/lidar_pointcloud')
proc = subprocess.Popen(['ros2','bag','record','-o',str(a.output)] + topics)
try:
    # A finite observation window, not a watchdog or recovery mechanism.
    proc.wait(timeout=a.duration)
    raise RuntimeError('Bag recorder exited before the capture window ended')
except subprocess.TimeoutExpired:
    proc.send_signal(signal.SIGINT)
    if proc.wait() != 0:
        raise RuntimeError('Bag recorder did not close successfully')
finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        proc.wait()
print('Capture complete:', a.output)
