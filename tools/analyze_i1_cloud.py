#!/usr/bin/env python3
"""Replay saved real inputs through both bridge callbacks, without DDS."""
import argparse
from collections import deque, Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.RANGE_MAX = 8.0
    return module


def replay_node():
    logger = NS(info=lambda *_: None, warn=lambda *_: None)
    return NS(acc=[], t_acc0=None, R_lv=None, R_wb=None, poses={}, track=deque(),
              last_key=None, tf=None, n_cloud=0, every=1, sub_imu=None,
              destroy_subscription=lambda *_: None, get_logger=lambda: logger,
              pub_scan=NS(publish=lambda _: None), pub_odom=NS(publish=lambda _: None),
              pub_clearing=NS(publish=lambda _: None), pub_obstacles=NS(publish=lambda _: None))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--candidate', type=Path, default=Path(__file__).resolve().parents[1]/'ws/src/lio/level_bridge.py')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    modules = [load('i1_baseline', args.baseline), load('i1_candidate', args.candidate)]
    nodes = [replay_node(), replay_node()]
    scans, clearing, marking = [[], []], [], []
    for i, node in enumerate(nodes):
        node.pub_scan.publish = scans[i].append
    nodes[1].pub_clearing.publish = clearing.append
    nodes[1].pub_obstacles.publish = marking.append
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    violations, radii, hits = Counter(), [], []
    clouds = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ('/qazbot_lio/cloud_registered', '/qazbot_lio/odometry',
                         '/aima/hal/sensor/lidar_chest_front/imu'):
            continue
        msg = deserialize_message(data, types[topic])
        method = 'on_cloud' if topic.endswith('cloud_registered') else 'on_odom' if topic.endswith('/odometry') else 'on_imu'
        before = len(scans[0])
        for module, node in zip(modules, nodes):
            getattr(module.LevelBridge, method)(node, msg)
        if method != 'on_cloud' or len(scans[0]) == before:
            continue
        clouds += 1
        a, b = np.asarray(scans[0][-1].ranges), np.asarray(scans[1][-1].ranges)
        violations['changed_marking_scan'] += int(not np.array_equal(a, b, equal_nan=True))
        hits.append(int(np.isfinite(a).sum()))
        marked = pc2.read_points_numpy(marking[-1], field_names=('x', 'y', 'z'))
        marked_r = np.sort(np.hypot(marked[:, 0], marked[:, 1]))
        expected_r = np.sort(a[np.isfinite(a)])
        violations['lost_or_changed_obstacle_ranges'] += int(
            len(marked_r) != len(expected_r) or not np.allclose(marked_r, expected_r, rtol=0, atol=1e-5))
        clear = pc2.read_points_numpy(clearing[-1], field_names=('x', 'y', 'z'))
        cr, ca = np.hypot(clear[:, 0], clear[:, 1]), np.arctan2(clear[:, 1], clear[:, 0])
        radii.extend(cr.tolist())
        violations['outside_fov'] += int(((ca < math.radians(-46)-1e-6) | (ca > math.radians(44.5)+1e-6)).sum())
        key = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        _, _, yaw, origin = nodes[1].poses[key]
        q = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(float) @ nodes[1].R_lv.T - origin
        q = q[np.isfinite(q).all(axis=1)]
        r = np.hypot(q[:, 0], q[:, 1])
        c, s = math.cos(-yaw), math.sin(-yaw)
        az = np.arctan2(s*q[:, 0]+c*q[:, 1], c*q[:, 0]-s*q[:, 1])
        idx = ((az+math.pi)/(2*math.pi)*1440).astype(int) % 1440
        obstacle = np.full(1440, np.inf)
        inside = (r > .45) & (r < 8) & (q[:, 2] > -.65+.035*r) & (q[:, 2] < .8)
        np.minimum.at(obstacle, idx[inside], r[inside])
        # Independent witness: an actual same-bearing measurement reaches this
        # endpoint within the existing height slice, with no nearer 2D hit.
        for distance, angle in zip(cr, ca):
            delta = np.arctan2(np.sin(az-angle), np.cos(az-angle))
            witness = (abs(delta) < 1e-6) & (r >= distance-1e-5) & (r > 0)
            z = q[witness, 2]*distance/r[witness]
            supported = ((z >= -.65+.035*distance-1e-5) & (z <= .8+1e-5) &
                         (distance <= obstacle[idx[witness]]+1e-5))
            violations['unproven_clearing_rays'] += int(not supported.any())
    result = {'bag': str(args.bag), 'baseline_sha256': hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
              'candidate_sha256': hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
              'usable_clouds_after_level_initialization': clouds, 'violations': dict(violations),
              'finite_obstacle_bins': {'min': min(hits), 'max': max(hits)},
              'clearing_rays': len(radii),
              'clearing_range_m': {'min': min(radii), 'median': float(np.median(radii)),
                                   'p95': float(np.percentile(radii, 95)), 'max': max(radii)},
              'rays_at_least_1m': sum(x >= 1 for x in radii),
              'rays_at_least_1_5m': sum(x >= 1.5 for x in radii),
              'scope': 'Actual saved cloud/odometry/IMU callbacks; no DDS or motion. Proof uses existing levelled IMU-origin 2D-slice model, not an entire vertical column.'}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if clouds and not any(violations.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
