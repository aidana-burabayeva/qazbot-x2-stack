#!/usr/bin/env python3
"""Full-bag emulation of nav2_collision_monitor 1.1.18 decisions on the raw X2 LiDAR cloud (no ROS graph).

Reads every raw cloud of the bag (rosbag2_py), transforms it with the fixture TF from config/harness.yaml,
applies the CM polygons from config/collision_monitor_x2_offline.yaml for several min_height values and
stores one record per cloud. Used for (a) ground-truth comparison over the whole bag and (b) cross-checking
the real node runs (tools/analyze_run.py).

usage: offline_emulate.py OUT.pkl [--every N]
"""
import argparse
import os
import pickle
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'lib'))
import cm_eval_lib as L  # noqa: E402

BAG = ('/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/'
       'd04_static_bag_183015_0.mcap')
RAW = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
MIN_HEIGHTS = (0.03, 0.05, 0.10, 0.15, 0.20, 0.30)


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_cfg():
    cm = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'collision_monitor_x2_offline.yaml')))
    h = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'harness.yaml')))
    p = cm['collision_monitor']['ros__parameters']
    f = h['cm_fixture_static_tf']['ros__parameters']
    return p, quat_to_rot(*f['rotation_xyzw']), np.array(f['translation'], dtype=float)


def cloud_xyz(m):
    from sensor_msgs_py import point_cloud2 as pc2
    a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.column_stack((a['x'], a['y'], a['z'])).astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--every', type=int, default=1)
    a = ap.parse_args()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    p, R, t = load_cfg()
    polys = L.polygons_from_params(p)
    stop = next(q for q in polys if q['action'] == 'stop')
    slow = next(q for q in polys if q['action'] == 'slowdown')
    vin = (0.30, 0.0, 0.0)
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=[RAW]))
    recs, keep = [], []
    t0 = time.time()
    i = 0
    while rd.has_next():
        _, data, ts = rd.read_next()
        i += 1
        if i % a.every:
            continue
        m = deserialize_message(data, PointCloud2)
        P = cloud_xyz(m)
        slant = np.linalg.norm(P, axis=1)
        B = L.transform_cloud(P, R, t)
        near = B[(np.abs(B[:, 0]) < 3.0) & (np.abs(B[:, 1]) < 3.0)]
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        r = dict(ts=ts, stamp=stamp, n=len(P), slant_min=float(slant.min()) if len(P) else None)
        for mh in MIN_HEIGHTS:
            xy = L.height_filter(near, mh, p['lidar']['max_height'])
            act, vel, name = L.emulate_decision(xy, vin, polys)
            r[f'act_{mh:.2f}'] = act
            r[f'vx_{mh:.2f}'] = vel[0]
            r[f'nstop_{mh:.2f}'] = L.point_in_polygon_count(xy, stop['points'])
            r[f'nslow_{mh:.2f}'] = L.point_in_polygon_count(xy, slow['points'])
        # nearest in-band point ahead in the corridor |y|<0.45 (for distance-to-obstacle plots)
        xy = L.height_filter(near, 0.05, 2.0)
        cor = xy[(np.abs(xy[:, 1]) < 0.45) & (xy[:, 0] > 0.0)]
        r['x_near_corridor'] = float(cor[:, 0].min()) if len(cor) else None
        # keep compact region points (h > 0.02, r < 2 m) for later inspection
        rr = np.hypot(near[:, 0], near[:, 1])
        sel = near[(rr < 2.0) & (near[:, 2] > 0.02) & (near[:, 2] < 2.3)]
        keep.append((stamp, sel.astype(np.float32)))
        recs.append(r)
        if len(recs) % 500 == 0:
            print(f'{len(recs)} clouds {time.time() - t0:.0f}s', flush=True)
    with open(a.out, 'wb') as f:
        pickle.dump(dict(recs=recs, min_heights=MIN_HEIGHTS, params=p), f)
    with open(a.out.replace('.pkl', '_pts.pkl'), 'wb') as f:
        pickle.dump(keep, f)
    print(f'done {len(recs)} clouds {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
