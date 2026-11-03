#!/usr/bin/env python3
"""Evaluate base_link vs IMU trajectories using finalized recorded vendor TF.

Reads one bag in full. Does not publish TF/odometry, call MC, or select a pose.
"""
import argparse
import json
import math
from pathlib import Path
import sqlite3
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException
from control_geometry import transform, world_base, planar_pose


def stamp(msg):
    return msg.sec * 10**9 + msg.nanosec


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def summarize(poses):
    a = np.asarray(poses)
    yaw = np.unwrap(a[:, 2])
    dist = np.linalg.norm(a[:, :2]-a[0, :2], axis=1)
    return {'samples': len(a), 'max_displacement_m': float(dist.max()),
            'end_displacement_m': float(dist[-1]),
            'yaw_span_deg': math.degrees(float(np.ptp(yaw))),
            'end_yaw_change_deg': math.degrees(float(yaw[-1]-yaw[0]))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if not (args.bag/'metadata.yaml').is_file():
        parser.error('Input bag must be finalized')
    db = sqlite3.connect(next(args.bag.glob('*.db3')).resolve().as_uri()+'?mode=ro', uri=True)
    types = {i: (name, get_message(typ)) for i, name, typ in db.execute('SELECT id,name,type FROM topics')}
    wanted = ['/qazbot_lio/odometry', '/qazbot_lio/odom_level', '/tf', '/tf_static']
    rows = [(i, t, b) for i, t, b in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp') if types[i][0] in wanted]
    db.close()
    duration_ns = rows[-1][1]-rows[0][1]
    rclpy.init()
    try:
        # Retain the complete recorded interval, including interpolation at its
        # endpoints; this is archive storage, not a live transform timeout.
        buffer = Buffer(cache_time=Duration(nanoseconds=duration_ns+10**9))
        raw, level = {}, {}
        for tid, _, data in rows:
            name, cls = types[tid]
            msg = deserialize_message(data, cls)
            if name in ['/tf', '/tf_static']:
                for tf in msg.transforms:
                    if name == '/tf_static': buffer.set_transform_static(tf, 'recorded_vendor')
                    else: buffer.set_transform(tf, 'recorded_vendor')
            else:
                target = raw if name == '/qazbot_lio/odometry' else level
                target[stamp(msg.header.stamp)] = msg
        paired = sorted(raw.keys() & level.keys())
        wp = np.array([[raw[k].pose.pose.position.x, raw[k].pose.pose.position.y, raw[k].pose.pose.position.z] for k in paired])
        lp = np.array([[level[k].pose.pose.position.x, level[k].pose.pose.position.y, level[k].pose.pose.position.z] for k in paired])
        # The bridge rotates positions with a fixed R_lv and no translation.
        # Recover that recorded rotation exactly instead of assuming world-z up.
        fit, _, rank, singular = np.linalg.lstsq(wp, lp, rcond=None)
        R = fit.T
        if rank != 3:
            raise RuntimeError('Recorded motion cannot identify the leveling rotation (rank < 3)')
        residual = float(np.max(np.abs(wp @ R.T-lp)))
        out = {'qualification': 'Offline same-stamp body geometry only; no external position ground truth.',
               'paired_samples': len(paired), 'level_fit_rank': int(rank),
               'level_fit_singular_values': singular.tolist(), 'level_fit_max_component_error_m': residual,
               'level_rotation': R.tolist(), 'tf_failures': [], 'samples': []}
        body_poses, imu_poses, offsets, heading = [], [], [], []
        for index, k in enumerate(paired):
            try:
                tf = buffer.lookup_transform('base_link', 'lidar_imu_chest_front', Time(nanoseconds=k))
            except TransformException as exc:
                out['tf_failures'].append({'stamp_ns': k, 'reason': str(exc)})
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            bi = transform([p.x,p.y,p.z], [q.x,q.y,q.z,q.w])
            wi = pose_matrix(raw[k].pose.pose)
            body = planar_pose(R, world_base(wi, bi))
            imu = lp[index, :2]
            body_poses.append(body)
            lq = level[k].pose.pose.orientation
            lyaw = math.atan2(2*(lq.w*lq.z+lq.x*lq.y), 1-2*(lq.y*lq.y+lq.z*lq.z))
            imu_poses.append([imu[0], imu[1], lyaw])
            offsets.append(float(np.linalg.norm(body[:2]-imu)))
            heading.append(math.degrees(math.atan2(math.sin(body[2]-lyaw), math.cos(body[2]-lyaw))))
            out['samples'].append({'stamp_ns': k, 'body_xy_yaw': body.tolist(), 'virtual_imu_xy_yaw': imu_poses[-1]})
        if not body_poses:
            raise RuntimeError('No source pose has a same-stamp body TF')
        out.update(body=summarize(body_poses), virtual_imu=summarize(imu_poses),
                   planar_reference_offset_m={'min': min(offsets), 'max': max(offsets)},
                   body_minus_virtual_heading_deg={'min': min(heading), 'max': max(heading)})
        args.output.write_text(json.dumps(out, indent=2)+'\n')
        print(json.dumps({k:v for k,v in out.items() if k not in ['samples','tf_failures']}, indent=2))
        print('Samples without usable TF:', len(out['tf_failures']))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
