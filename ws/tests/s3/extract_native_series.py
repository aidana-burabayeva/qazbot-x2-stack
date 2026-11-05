#!/usr/bin/env python3
"""Offline: extract the inputs of the native-base stop limiter from a finished bag into a compact fixture.

Reads the bag read-only (sqlite ?mode=ro); no ROS graph, no publishers. Output (gzip JSON), per message:
  odom  /aima/mc/leg_odometry      [recv_unix, stamp, x, y, z, qx, qy, qz, qw]   (pose of lidar_imu_chest_front in leg_odom)
  tf    /tf, the dynamic edges of the base_link -> lidar_imu_chest_front chain: {edge: [[recv_unix, stamp, tx..qw], ...]}
  tf_static  every /tf_static edge (first seen): {edge: [parent, child, tx..qw]}
  lio   /qazbot_lio/odom_level     [recv_unix, stamp, x, y, yaw]   (diagnostic; level_bridge base_footprint)
  cmd   /qazbot_d1_map/cmd_vel_authorized [recv_unix, vx, wz]
recv_unix is the recorder receive time (bag timestamp), used as "now" when the tests replay the series.
Usage: python3 tests/s3/extract_native_series.py BAG_DIR OUT.json.gz
"""
import gzip
import json
import math
import sqlite3
import sys
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

CHAIN_DYNAMIC = ('pelvis->waist_yaw_link', 'waist_yaw_link->waist_pitch_link', 'waist_pitch_link->torso_link')


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main(bag, out):
    db = next(Path(bag).glob('*.db3')).resolve()
    con = sqlite3.connect(db.as_uri() + '?mode=ro', uri=True)
    topics = {name: i for i, name in con.execute('SELECT id,name FROM topics')}

    def rows(name):
        if name not in topics:
            return []
        return con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topics[name],))

    r = dict(source=str(Path(bag).resolve()), odom=[], tf={k: [] for k in CHAIN_DYNAMIC}, tf_static={}, lio=[], cmd=[])
    for ts, blob in rows('/aima/mc/leg_odometry'):
        m = deserialize_message(blob, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r['odom'].append([ts * 1e-9, stamp(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    for ts, blob in rows('/tf'):
        for t in deserialize_message(blob, TFMessage).transforms:
            k = t.header.frame_id + '->' + t.child_frame_id
            if k in r['tf']:
                a, q = t.transform.translation, t.transform.rotation
                r['tf'][k].append([ts * 1e-9, stamp(t.header), a.x, a.y, a.z, q.x, q.y, q.z, q.w])
    for ts, blob in rows('/tf_static'):
        for t in deserialize_message(blob, TFMessage).transforms:
            k = t.header.frame_id + '->' + t.child_frame_id
            a, q = t.transform.translation, t.transform.rotation
            r['tf_static'].setdefault(k, [t.header.frame_id, t.child_frame_id, a.x, a.y, a.z, q.x, q.y, q.z, q.w])
    for ts, blob in rows('/qazbot_lio/odom_level'):
        m = deserialize_message(blob, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r['lio'].append([ts * 1e-9, stamp(m.header), p.x, p.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])
    for ts, blob in rows('/qazbot_d1_map/cmd_vel_authorized'):
        m = deserialize_message(blob, Twist)
        r['cmd'].append([ts * 1e-9, m.linear.x, m.angular.z])
    con.close()
    with gzip.open(out, 'wt') as f:
        json.dump(r, f)
    print(json.dumps({k: (len(v) if isinstance(v, list) else {e: len(x) for e, x in v.items()}) for k, v in r.items()
                      if k != 'source'}))


if __name__ == '__main__':
    main(*sys.argv[1:3])
