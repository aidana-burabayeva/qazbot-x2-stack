#!/usr/bin/env python3
"""P-D04 offline extraction (read-only bag reader, no ROS graph).

Pass 0: small topics (odom, imu, scan_obstacles, scan_clearing, costmap_raw, d1_map tf) + a few clouds for
        the levelling fit (fit of the LIVE R_lv against recorded scan_obstacles, replay_d04 code).
Pass 1: every cloud_registered whose stamp has a recorded scan_obstacles (= the clouds the live bridge used):
        candidate C on_cloud -> obstacles/clearing (body); levelled region points for segmentation.
        Raw lidar cloud: levelled region points (extrinsics + odom orientation, robot static).
Output: out_dir/{small.pkl, frames.pkl, raw.pkl}
"""
import math
import os
import pickle
import sys
import time

import numpy as np

sys.path.insert(0, '/home/agi/control_backend/tests/d04')
import replay_d04 as R  # noqa: E402

BAG = '/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap'
OUT = sys.argv[1]
T_CLOUD = '/qazbot_lio/cloud_registered'
T_ODOM = '/qazbot_lio/odometry'
T_IMU = '/aima/hal/sensor/lidar_chest_front/imu'
T_OB = '/qazbot_lio/scan_obstacles'
T_CL = '/qazbot_lio/scan_clearing'
T_CM = '/qazbot_d1_map/nav2/local_costmap/costmap_raw'
T_TF = '/qazbot_d1_map/tf'
T_RAW = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
EXT = R.EXTRINSICS


def reader(topics):
    import rosbag2_py
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    return rd, types


def ns(st):
    return st.sec * 10**9 + st.nanosec


def xyz(m):
    return R.cloud_xyz(m)


def region(Pb, fz, rmax=3.0):
    """body-frame points (x fwd, y left, z rel. IMU) -> region points with h = z - floor."""
    r = np.hypot(Pb[:, 0], Pb[:, 1])
    b = np.degrees(np.arctan2(Pb[:, 1], Pb[:, 0]))
    h = Pb[:, 2] - fz
    m = (r > 0.15) & (r < rmax) & (np.abs(b) <= 135) & (h > 0.02) & (h < 2.3)
    q = np.column_stack((Pb[m, 0], Pb[m, 1], h[m])).astype(np.float32)
    # 2 cm voxel downsample (keep max h per xy cell kept implicitly by keeping all z voxels)
    if len(q):
        k = np.floor(q / 0.02).astype(np.int32)
        _, i = np.unique(k, axis=0, return_index=True)
        q = q[i]
    return q


def main():
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    A = R.load_bridge(R.RUNNING, 'lb_A')
    C = R.load_bridge(R.CANDIDATE, 'lb_C')
    # ---------------- pass 0
    rd, types = reader([T_ODOM, T_IMU, T_OB, T_CL, T_CM, T_TF, T_CLOUD])
    cls = {k: get_message(v) for k, v in types.items()}
    cache = os.path.join(OUT, 'pass0.pkl')
    if os.path.exists(cache):
        small, R_lv = pickle.load(open(cache, 'rb'))
        print('pass0 from cache', small['fit'], flush=True)
    else:
        small, R_lv = pass0(rd, types, cls, A, deserialize_message, t0)
        pickle.dump((small, R_lv), open(cache, 'wb'))
    pass1(small, R_lv, A, C, cls, deserialize_message, t0)


def pass0(rd, types, cls, A, deserialize_message, t0):
    probe = R.stub_node(A)
    odom, ob, cl, cm, tfs = {}, {}, {}, [], []
    fit_clouds = {}
    n_cloud = 0
    imu_first = None
    while rd.has_next():
        name, data, ts = rd.read_next()
        if name == T_CLOUD:
            n_cloud += 1
            # keep raw bytes of a sparse sample for the fit (every 150th cloud ~ 15 s)
            if n_cloud % 150 == 7:
                fit_clouds[ts] = data
            continue
        m = deserialize_message(data, cls[name])
        if name == T_ODOM:
            odom[ns(m.header.stamp)] = m
            A.LevelBridge.on_odom(probe, m)
        elif name == T_IMU:
            if probe.R_lv is None:
                imu_first = imu_first or ts
                A.LevelBridge.on_imu(probe, m)
        elif name == T_OB:
            ob[ns(m.header.stamp)] = (ts, xyz(m).astype(np.float32))
        elif name == T_CL:
            cl[ns(m.header.stamp)] = (ts, xyz(m).astype(np.float32))
        elif name == T_CM:
            xy, g = None, None
            md = m.metadata
            g = np.asarray(m.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
            cm.append(dict(ts=ts, stamp=ns(m.header.stamp), ox=md.origin.position.x, oy=md.origin.position.y,
                           res=md.resolution, grid=g.copy()))
        elif name == T_TF:
            for tr in m.transforms:
                if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                    q = tr.transform.rotation
                    tfs.append((ns(tr.header.stamp), tr.transform.translation.x, tr.transform.translation.y,
                                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), tr))
    print(f'pass0 {time.time()-t0:.0f}s clouds={n_cloud} odom={len(odom)} ob={len(ob)} cl={len(cl)} cm={len(cm)} tf={len(tfs)}',
          flush=True)
    R0 = probe.R_lv
    # fit pairs: clouds whose stamp has recorded obstacles
    pairs = []
    for ts, data in sorted(fit_clouds.items()):
        m = deserialize_message(data, cls[T_CLOUD])
        k = ns(m.header.stamp)
        if k not in ob or k not in odom:
            # neighbouring cloud is the processed one; skip
            continue
        pairs.append((odom[k], m, ob[k][1].astype(float)))
    print('fit pairs', len(pairs), flush=True)
    if len(pairs) < 8:
        # take processed clouds directly in a short extra pass
        rd2, _ = reader([T_CLOUD])
        want = sorted(k for k in ob if k in odom)[::max(1, len(ob) // 25)]
        want = set(want)
        while rd2.has_next() and len(pairs) < 24:
            name, data, ts = rd2.read_next()
            m = deserialize_message(data, cls[T_CLOUD])
            k = ns(m.header.stamp)
            if k in want:
                pairs.append((odom[k], m, ob[k][1].astype(float)))
        print('fit pairs (extra pass)', len(pairs), flush=True)
    R_lv, info = R.fit_levelling(A, R0, pairs[:24])
    print('fit', info, flush=True)
    del fit_clouds, pairs
    small = dict(ob=ob, cl=cl, cm=cm, tfs=[t[:4] for t in tfs], R_lv=R_lv, R0=R0, fit=info,
                 odom_keys=sorted(odom))
    tf_msgs = [t[4] for t in tfs]
    with open(os.path.join(OUT, 'tf_msgs.pkl'), 'wb') as f:
        pickle.dump(tf_msgs, f)
    return small, R_lv


def pass1(small, R_lv, A, C, cls, deserialize_message, t0):
    ob = small['ob']
    # ---------------- pass 1
    ext = __import__('json').load(open(EXT))
    R_il = np.array(ext['R_imu_lidar'])
    T_il = np.array(ext['T_imu_lidar'])
    sA = R.stub_node(A)
    sA.R_lv = R_lv
    sC = R.stub_node(C)
    sC.R_lv = R_lv
    rd, _ = reader([T_CLOUD, T_ODOM, T_RAW])
    frames, raws = [], []
    fz = None
    last_odom = None
    nraw = 0
    while rd.has_next():
        name, data, ts = rd.read_next()
        m = deserialize_message(data, cls[name])
        if name == T_ODOM:
            A.LevelBridge.on_odom(sA, m)
            last_odom = m
            continue
        if name == T_RAW:
            nraw += 1
            if last_odom is None or fz is None:
                continue
            from sensor_msgs_py import point_cloud2 as pc2
            sp = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            P = np.column_stack((sp['x'], sp['y'], sp['z'])).astype(np.float64)
            if not len(P):
                continue
            pose = sA.poses[max(sA.poses)]
            R_wb = A.quat_to_rot(last_odom.pose.pose.orientation)
            Pi = P @ R_il.T + T_il                    # lidar -> imu
            Q = Pi @ (R_lv @ R_wb).T                  # levelled, relative to imu
            Pb = R.to_body(Q, pose[2])
            raws.append(dict(ts=ts, stamp=ns(m.header.stamp), n=len(P), reg=region(Pb, fz)))
            continue
        k = ns(m.header.stamp)
        if k not in ob:
            continue
        pose = sA.poses.get(k)
        if pose is None:
            continue
        fr = dict(msg=m, pose=pose, R_lv=R_lv, key=k)
        Q = R.level_points(fr)
        Pb = R.to_body(Q, pose[2])
        if fz is None:
            rr = np.hypot(Pb[:, 0], Pb[:, 1])
            fl = Pb[(rr > 0.6) & (rr < 3.0) & (Pb[:, 2] < -0.8)]
            fz = float(np.median(fl[:, 2]))
            print('floor_z', fz, flush=True)
        sC.poses = {k: pose}
        sC.pub_obstacles.msg = sC.pub_clearing.msg = None
        C.LevelBridge.on_cloud(sC, m)
        frames.append(dict(ts=ts, key=k, pose=tuple(float(v) for v in pose[:3]),
                           obC=xyz(sC.pub_obstacles.msg).astype(np.float32),
                           clC=xyz(sC.pub_clearing.msg).astype(np.float32),
                           reg=region(Pb, fz), n=len(Pb)))
        if len(frames) % 300 == 0:
            print(f'pass1 frames={len(frames)} raws={len(raws)} {time.time()-t0:.0f}s', flush=True)
    small['floor_z'] = fz
    with open(os.path.join(OUT, 'small.pkl'), 'wb') as f:
        pickle.dump(small, f)
    with open(os.path.join(OUT, 'frames.pkl'), 'wb') as f:
        pickle.dump(frames, f)
    with open(os.path.join(OUT, 'raw.pkl'), 'wb') as f:
        pickle.dump(raws, f)
    print(f'done {time.time()-t0:.0f}s frames={len(frames)} raws={len(raws)} nraw={nraw}', flush=True)


if __name__ == '__main__':
    main()
