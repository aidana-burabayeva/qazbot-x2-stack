#!/usr/bin/env python3
"""Extract the parity/integration fixture from a recorded bag (offline tool, not run by the tests).

    source /opt/ros/humble/setup.bash
    nice -n 19 python3 make_fixture.py BAG_DIR OUT.bin [--skip-s 60] [--clouds 50]

Takes the time-ordered window that starts at the first LIO odometry message >= skip_s after the bag start and
ends after `clouds` clouds: every lidar IMU message, every LIO odometry message and every cloud in it. Clouds
keep only x, y, z (float32; the node reads nothing else). Binary layout: see fixture.py.
"""
import argparse
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixture  # noqa: E402

CLOUD, ODOM, IMU = '/qazbot_lio/cloud_registered', '/qazbot_lio/odometry', '/aima/hal/sensor/lidar_chest_front/imu'
TYPES = {CLOUD: 'sensor_msgs/msg/PointCloud2', ODOM: 'nav_msgs/msg/Odometry', IMU: 'sensor_msgs/msg/Imu'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--skip-s', type=float, default=60.0)
    ap.add_argument('--clouds', type=int, default=50)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(TYPES)))
    classes = {t: get_message(ty) for t, ty in TYPES.items()}
    events, t_first, started, n_cl = [], None, False, 0
    while r.has_next() and n_cl < a.clouds:
        topic, data, ts = r.read_next()
        if t_first is None:
            t_first = ts
        if not started:
            if topic != ODOM or (ts - t_first) * 1e-9 < a.skip_s:
                continue
            started = True
        m = deserialize_message(data, classes[topic])
        s = m.header.stamp
        if topic == IMU:
            events.append(fixture.Event('imu', s.sec, s.nanosec, (m.linear_acceleration.x, m.linear_acceleration.y,
                                                                    m.linear_acceleration.z)))
        elif topic == ODOM:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            events.append(fixture.Event('odom', s.sec, s.nanosec,
                                        ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), tuple(m.pose.covariance))))
        else:
            xyz = fixture.cloud_xyz(m)
            n_cl += 1
            events.append(fixture.Event('cloud', s.sec, s.nanosec, (xyz, bool(m.is_dense))))
    fixture.write(a.out, events)
    print(f'{len(events)} events, {n_cl} clouds -> {a.out} ({os.path.getsize(a.out)} bytes)')


if __name__ == '__main__':
    main()
