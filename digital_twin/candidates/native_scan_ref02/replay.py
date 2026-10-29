"""Read-only physical-bag input check inside the twin dev container; no ROS node.

Correction magnitude is NOT error against ground truth. This checks frame/time
availability and exercises the candidate on real payloads, not AMCL/Nav2 quality.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from deskew import deskew

LEG = '/aima/mc/leg_odometry'
RAW = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def run_bag(path, budget, limit, extrinsic):
    c = sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)
    ids = dict(c.execute('select name,id from topics'))
    ts, positions, quaternions, frames = [], [], [], set()
    native_sha = hashlib.sha256()
    for (data,) in c.execute('select data from messages where topic_id=? order by timestamp', (ids[LEG],)):
        native_sha.update(data)
        m = deserialize_message(data, Odometry)
        frames.add((m.header.frame_id, m.child_frame_id))
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ts.append(stamp(m))
        positions.append((p.x, p.y, p.z))
        quaternions.append((q.x, q.y, q.z, q.w))
    if frames != {('leg_odom', 'lidar_imu_chest_front')}:
        raise ValueError('unexpected native pose frames: ' + str(frames))
    raw_ids = [r[0] for r in c.execute('select id from messages where topic_id=? order by timestamp', (ids[RAW],))]
    selected_ids = {raw_ids[k] for k in np.unique(np.linspace(0, len(raw_ids) - 1, min(limit, len(raw_ids))).astype(int))}
    # Include the actual source-time gaps, rather than hoping a uniform sample
    # happens to hit them. The generous receive-time window only selects input;
    # deskew still checks exact point timestamps against the pose history.
    source_gaps = np.flatnonzero(np.diff(ts) > budget)
    for i in source_gaps:
        nearby = c.execute('select id,data from messages where topic_id=? and timestamp between ? and ?',
                           (ids[RAW], int((ts[i] - .2) * 1e9), int((ts[i + 1] + .4) * 1e9)))
        for mid, payload in nearby:
            candidate = deserialize_message(payload, PointCloud2)
            begin = stamp(candidate)
            if begin < ts[i + 1] and begin + .1 > ts[i]:
                selected_ids.add(mid)
    rows, raw_sha = [], hashlib.sha256()
    for mid in sorted(selected_ids):
        data = c.execute('select data from messages where id=?', (mid,)).fetchone()[0]
        raw_sha.update(data)
        m = deserialize_message(data, PointCloud2)
        if m.header.frame_id != 'lidar_chest_front':
            raise ValueError('unexpected LiDAR frame: ' + m.header.frame_id)
        p = point_cloud2.read_points(m, field_names=('x', 'y', 'z', 'timestamp'), skip_nans=False)
        points = np.column_stack([p[k] for k in ('x', 'y', 'z')]).astype(float)
        offsets = p['timestamp'].astype(float)
        ref = stamp(m) + float(np.max(offsets)) * 1e-9
        row = dict(message_id=mid, header_stamp=stamp(m), reference_stamp=ref, point_count=len(points))
        try:
            start = time.perf_counter()
            corrected = deskew(points, offsets, stamp(m), ts, positions, quaternions,
                               extrinsic[0], extrinsic[1], ref, budget, 2., 5.)
            row['elapsed_s'] = time.perf_counter() - start
            at_original = points @ extrinsic[0].T + extrinsic[1]
            delta = np.linalg.norm(corrected.points - at_original, axis=1)
            row.update(accepted=True, interpolation_gap_s=corrected.maximum_interpolation_gap_s,
                       correction_rms_m=float(np.sqrt(np.mean(delta**2))), correction_p95_m=float(np.percentile(delta, 95)))
        except ValueError as e:
            row.update(accepted=False, reason=str(e))
        rows.append(row)
    c.close()
    return dict(bag=str(path), pose_frames=sorted(frames), native_samples=len(ts), raw_messages=len(raw_ids),
                source_gaps_exceeding_budget=len(source_gaps), selection='uniform sample plus clouds near native source gaps',
                selected_raw_messages_sha256=raw_sha.hexdigest(), native_messages_sha256=native_sha.hexdigest(),
                sampled=len(rows), accepted=sum(r['accepted'] for r in rows), rows=rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+', type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--gap-budget-s', required=True, type=float)
    ap.add_argument('--limit', type=int, default=40)
    a = ap.parse_args()
    ex = json.load(open('/backend_logs/x2_extrinsics.json'))
    extrinsic = np.asarray(ex['R_imu_lidar']), np.asarray(ex['T_imu_lidar'])
    result = dict(experiment='REF02 physical payload replay; no ground truth or navigation qualification',
                  gap_budget_s=a.gap_budget_s,
                  max_linear_rate_mps=2., max_angular_rate_radps=5.,
                  budget_status='experimental supplied value; not a production safety threshold',
                  bags=[run_bag(p, a.gap_budget_s, a.limit, extrinsic) for p in a.bags])
    a.out.mkdir(parents=True, exist_ok=False)
    (a.out / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(out=str(a.out), gap_budget_s=a.gap_budget_s,
                          bags=[{k: b[k] for k in ('bag', 'native_samples', 'raw_messages', 'sampled', 'accepted')}
                                for b in result['bags']]), indent=2))


if __name__ == '__main__':
    main()
