#!/usr/bin/env python3
"""R5 offline extraction (read-only): windows of the P-D04 static bag -> pickle. No ROS graph."""
import pickle, sys, time
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG = '/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
T0 = int(sys.argv[1]); WINS = [tuple(map(float, w.split(':'))) for w in sys.argv[3:]]; OUT = sys.argv[2]
TOP = ['/qazbot_lio/scan_obstacles', '/qazbot_lio/scan_clearing', '/qazbot_d1_map/tf', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
       '/qazbot_lio/cloud_registered', '/qazbot_d1_map/nav2/local_costmap/costmap_raw']
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
rd.set_filter(rosbag2_py.StorageFilter(topics=TOP))
from sensor_msgs_py import point_cloud2 as pc2
res = []
for (a, b) in WINS:
    rd.seek(T0 + int(a * 1e9)); w = dict(a=a, b=b, ob=[], cl=[], tf=[], raw=[], reg=[], cm=[])
    while rd.has_next():
        top, data, t = rd.read_next()
        if t > T0 + b * 1e9: break
        if top.endswith('lidar_pointcloud') or top.endswith('cloud_registered'):
            # header only: deserialize full (needed) but keep stamp
            m = deserialize_message(data, get_message(types[top]))
            (w['raw'] if 'lidar' in top else w['reg']).append((t, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m.header.frame_id))
            continue
        m = deserialize_message(data, get_message(types[top]))
        if top.endswith('costmap_raw'):
            w['cm'].append((t, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)); continue
        if top.endswith('tf'):
            w['tf'].append((t, m)); continue
        P = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float32).reshape(-1, 3)
        (w['ob'] if 'obstacles' in top else w['cl']).append((t, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m.header.frame_id, P))
    res.append(w); print('win', a, b, {k: len(v) for k, v in w.items() if isinstance(v, list)}, flush=True)
pickle.dump(dict(T0=T0, types=types, wins=res), open(OUT, 'wb'))
