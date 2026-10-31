#!/usr/bin/env python3
"""Offline replay of verified recorded cloud/odom/IMU through production callbacks. Never initializes ROS/DDS."""
import argparse
from collections import deque
import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sqlite3
import subprocess
import tempfile
from types import SimpleNamespace

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[2]
BAG = ROOT / 'docs/evidence/i1_cloud_registered_udp_20261028_092700Z'


def extract():
    manifest = json.loads((BAG / 'archive_manifest.json').read_text())
    chunks = []
    for item in manifest['parts']:
        data = (ROOT / item['path']).read_bytes()
        if hashlib.sha256(data).hexdigest() != item['sha256']:
            raise ValueError('bag part hash mismatch')
        chunks.append(data)
    packed = b''.join(chunks)
    if hashlib.sha256(packed).hexdigest() != manifest['gzip_sha256']:
        raise ValueError('gzip hash mismatch')
    raw = gzip.decompress(packed)
    if hashlib.sha256(raw).hexdigest() != manifest['raw_sha256']:
        raise ValueError('raw bag hash mismatch')
    spec = importlib.util.spec_from_file_location('production_level_bridge', ROOT / 'ws/src/lio/level_bridge.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    parameter_blob = subprocess.check_output(['git','-C',str(ROOT),'show','origin/robot/stable:docs/evidence/x2_twin_20261030/snapshot/actual_parameters.json'])
    parameters = json.loads(parameter_blob)['nodes']['/qazbot_level_bridge']
    module.RANGE_MAX = float(parameters['range_max']['double_value'])
    output = {'scan': [], 'obstacles': [], 'clearing': []}
    sink = lambda key: SimpleNamespace(publish=lambda msg: output[key].append(msg))
    bridge = SimpleNamespace(acc=[], t_acc0=None, R_lv=None, R_wb=None, poses={}, track=deque(), last_key=None,
                             tf=None, sub_imu=None, every=1, n_cloud=0,
                             pub_scan=sink('scan'), pub_obstacles=sink('obstacles'), pub_clearing=sink('clearing'),
                             pub_odom=SimpleNamespace(publish=lambda m: None),
                             get_logger=lambda: SimpleNamespace(info=lambda m: None, warn=lambda m: None),
                             destroy_subscription=lambda s: None)
    frames, stamps, ages, counts = [], [], [], {}
    first = None
    with tempfile.TemporaryDirectory(prefix='qazbot-lidar-fixture-') as tmp:
        path = Path(tmp) / 'recording.db3'; path.write_bytes(raw)
        del raw, packed, chunks
        con = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
        topics = {i: name for i, name in con.execute('select id,name from topics')}
        kinds = {'/qazbot_lio/odometry': (Odometry, module.LevelBridge.on_odom),
                 '/qazbot_lio/cloud_registered': (PointCloud2, module.LevelBridge.on_cloud),
                 module.IMU_TOPIC: (Imu, module.LevelBridge.on_imu)}
        for topic_id, received, blob in con.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
            name = topics[topic_id]; counts[name] = counts.get(name, 0) + 1
            if name not in kinds:
                continue
            cls, callback = kinds[name]
            msg = deserialize_message(blob, cls)
            before = len(output['scan'])
            callback(bridge, msg)
            if len(output['scan']) == before:
                continue
            scan, obs, clear = output['scan'][-1], output['obstacles'][-1], output['clearing'][-1]
            ranges = np.asarray(scan.ranges); idx = np.flatnonzero(np.isfinite(ranges))
            angles = scan.angle_min + idx * scan.angle_increment
            cp = np.asarray(pc2.read_points_numpy(clear, field_names=('x','y','z')))
            op = np.asarray(pc2.read_points_numpy(obs, field_names=('x','y','z')))
            stamp = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
            stamps.append(stamp); ages.append((received - stamp) * 1e-9)
            frames.append(dict(finite_bins=len(idx), bearing_min_deg=float(np.degrees(angles).min()),
                               bearing_max_deg=float(np.degrees(angles).max()), range_min_m=float(ranges[idx].min()),
                               range_max_m=float(ranges[idx].max()), obstacle_points=len(op), clearing_points=len(cp)))
            if first is None:
                first = dict(stamp_ns=stamp, frame_id=scan.header.frame_id, bins=len(ranges), angle_min=scan.angle_min,
                             angle_increment=scan.angle_increment, range_min=scan.range_min, range_max=scan.range_max,
                             finite_bin_indices=idx.tolist(), finite_ranges=ranges[idx].tolist(),
                             obstacles_xyz=op.tolist(), clearing_xyz=cp.tolist())
            # Captured publications are consumed immediately; no unbounded cloud history.
            for key in output:
                output[key].clear()
        con.close()
    return dict(source_raw_sha256=manifest['raw_sha256'], source_gzip_sha256=manifest['gzip_sha256'],
                published_parameters_sha256=hashlib.sha256(parameter_blob).hexdigest(), replay_range_max=module.RANGE_MAX,
                bridge_sha256=hashlib.sha256((ROOT/'ws/src/lio/level_bridge.py').read_bytes()).hexdigest(),
                qualification='RECORDED_CALLBACK_REPLAY_ONLY; no DDS, no physics/noise/dropout calibration', counts=counts,
                usable_frames=len(frames), frames=frames, fixture=first,
                header_period_s=np.diff(stamps).astype(float).__mul__(1e-9).tolist(), receive_minus_header_s=ages,
                limitations=['height-sliced processed cloud, not raw E1R FOV', 'finite angular gaps cannot identify dropout',
                             'scene range dispersion cannot identify sensor noise', 'Oct 28 recording, not new Oct 30 measurement'])


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', type=Path, required=True); args = ap.parse_args()
    data = extract(); args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps({k:v for k,v in data.items() if k not in ('fixture','frames','header_period_s','receive_minus_header_s')}, indent=2))
