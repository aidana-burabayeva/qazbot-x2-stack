#!/usr/bin/env python3
"""Passive per-message chain probe (subscribe only; no publishers/services/params changed).
Raw (serialized) subscriptions: header stamp parsed from CDR bytes -> no PointCloud2 deserialization cost.
Usage: chain_probe.py DURATION_S OUT.jsonl [--raw-cloud]
Each line: [topic_idx, stamp_s, receipt_realtime_s, receipt_monotonic_s, size_bytes] ; TF lines: [idx, stamp, rt, mono, 'parent->child']"""
import json, struct, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu, LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
dur = float(sys.argv[1]); out = sys.argv[2]; raw_cloud = '--raw-cloud' in sys.argv
TOP = [('/aima/hal/sensor/lidar_chest_front/imu', Imu),
       ('/qazbot_lio/cloud_registered', PointCloud2),
       ('/qazbot_lio/odometry', Odometry),
       ('/qazbot_lio/scan', LaserScan),
       ('/qazbot_lio/odom_level', Odometry),
       ('/qazbot_native/odom', Odometry),
       ('/aima/mc/leg_odometry', Odometry)]
if raw_cloud:
    TOP.append(('/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', PointCloud2))
TF_IDX = len(TOP); ST_IDX = TF_IDX + 1
rclpy.init(); node = rclpy.create_node('qazbot_chain_probe_ro')
rows = []; t0r = time.time()
def mk(i):
    def cb(b):
        rt = time.time(); mo = time.monotonic()
        sec, nsec = struct.unpack_from('<iI', b, 4)
        rows.append((i, sec + nsec * 1e-9, rt, mo, len(b)))
    return cb
be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
for i, (t, ty) in enumerate(TOP):
    node.create_subscription(ty, t, mk(i), be, raw=True)
def tfcb(m):
    rt = time.time(); mo = time.monotonic()
    for t in m.transforms:
        rows.append((TF_IDX, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, rt, mo, t.header.frame_id + '->' + t.child_frame_id))
node.create_subscription(TFMessage, '/qazbot_d1_map/tf', tfcb, QoSProfile(depth=50))
def stcb(m):
    rt = time.time(); mo = time.monotonic()
    try:
        j = json.loads(m.data); rows.append((ST_IDX, 0.0, rt, mo, json.dumps({'reasons': j.get('reasons'), 'input_ages': j.get('input_ages'), 'input_callbacks': j.get('input_callbacks')})))
    except Exception:
        pass
node.create_subscription(String, '/qazbot_d1_map/localization_trust/status', stcb, QoSProfile(depth=10))
t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    rclpy.spin_once(node, timeout_sec=0.02)
with open(out, 'w') as f:
    f.write(json.dumps({'topics': [t for t, _ in TOP] + ['/qazbot_d1_map/tf', 'trust_status'], 'start_rt': t0r, 'duration': dur}) + '\n')
    for r in rows:
        f.write(json.dumps(r) + '\n')
node.destroy_node(); rclpy.try_shutdown()
print('rows', len(rows))
