"""F1 (V1): live-matched levelling. The replay harness must be able to reproduce the RECORDED
scan_obstacles by fitting the live node's R_lv (bag levelling differs from it by ~0.6 deg on real
X2 data). Synthetic fixture only: no real data, no sleeps, no ROS graph.
"""
import json
import math
import sqlite3

import numpy as np
import pytest

import replay_d04 as R

A = R.load_bridge(R.RUNNING, 'lb_running_fit')
UP = np.array([1.0, 0.0, 0.0])                       # FAST-LIO world x ~ up on the X2
R0 = A.level_rotation(UP)
TILT_MRAD = (9.0, -6.0)                              # ~0.62 deg: same class as the real offset
R_TRUE = R.tilt_rotation(TILT_MRAD[0] * 1e-3, TILT_MRAD[1] * 1e-3) @ R0


def _stamp(msg, k):
    msg.header.stamp.sec, msg.header.stamp.nanosec = 100 + k // 10, (k % 10) * 10**8


def _scene(rng, n=1500):
    """Levelled-frame points: a sloped wall + clutter spanning the whole height band."""
    x = rng.uniform(0.8, 4.0, n)
    y = rng.uniform(-2.0, 2.0, n)
    z = rng.uniform(-0.55, 0.7, n)
    x = x + 0.6 * z                                  # sloped wall: nearest hit per bin depends on z
    return np.column_stack((x, y, z))


def _pair(k, rng):
    from nav_msgs.msg import Odometry
    from sensor_msgs_py import point_cloud2 as pc2
    od = Odometry()
    od.pose.pose.position.x, od.pose.pose.position.y = 0.05 * k, -0.02 * k
    od.pose.pose.orientation.w = 1.0
    _stamp(od, k)
    Q = _scene(rng)
    p_lv = R_TRUE @ np.array([od.pose.pose.position.x, od.pose.pose.position.y, 0.0])
    # a point cloud whose levelling by the TRUE R_lv gives Q (Q = P R^T - p)
    P = (Q + p_lv) @ R_TRUE
    cl = pc2.create_cloud_xyz32(od.header, P.astype(np.float32).tolist())
    return od, cl


def _recorded(od, cl):
    return R.bridge_obstacles(A, R_TRUE, od, cl)


@pytest.fixture(scope='module')
def pairs():
    rng = np.random.default_rng(7)
    out = []
    for k in range(6):
        od, cl = _pair(k, rng)
        rec = _recorded(od, cl)
        assert rec is not None and len(rec) > 100
        out.append((od, cl, rec))
    return out


def test_wrong_levelling_does_not_reproduce(pairs):
    cost = R._match_cost(A, R0, pairs)
    assert cost > 1e-3                                # ~mm..cm off with the bag-start levelling


def test_fitted_rlv_reproduces_recorded_scan_obstacles(pairs):
    Rf, info = R.fit_levelling(A, R0, pairs, maxiter=120)
    assert info['cost_after_m'] < 0.1 * info['cost_before_m']
    assert abs(info['tilt_mrad'][0] - TILT_MRAD[0]) < 1.5 and abs(info['tilt_mrad'][1] - TILT_MRAD[1]) < 1.5
    for od, cl, rec in pairs:
        o = R.bridge_obstacles(A, Rf, od, cl)
        assert o.shape == rec.shape
        assert np.abs(o - rec).max() < 2e-3            # within 2 mm
    assert np.allclose(Rf @ Rf.T, np.eye(3), atol=1e-9)


def test_load_rlv_explicit_and_validation(tmp_path):
    p = tmp_path / 'r.json'
    p.write_text(json.dumps({'R_lv': R_TRUE.tolist()}))
    assert np.allclose(R.load_rlv(str(p)), R_TRUE)
    p.write_text(json.dumps([[2, 0, 0], [0, 1, 0], [0, 0, 1]]))
    with pytest.raises(AssertionError):
        R.load_rlv(str(p))


def _write_bag(path, with_obstacles):
    """Tiny sqlite3 bag: 1.3 s of IMU (gravity along world x), then odom+cloud(+scan_obstacles)."""
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    c = sqlite3.connect(path)
    c.execute('create table topics(id integer primary key, name text, type text, '
              'serialization_format text, offered_qos_profiles text)')
    c.execute('create table messages(id integer primary key, topic_id int, timestamp int, data blob)')
    topics = [('/imu', 'sensor_msgs/msg/Imu'), ('/odom', 'nav_msgs/msg/Odometry'),
              ('/cloud', 'sensor_msgs/msg/PointCloud2'), ('/obs', 'sensor_msgs/msg/PointCloud2')]
    for i, (n, t) in enumerate(topics, 1):
        c.execute('insert into topics values(?,?,?,?,?)', (i, n, t, 'cdr', ''))
    rows = []
    od0, _ = _pair(0, np.random.default_rng(3))      # on_imu needs R_wb from an odom first
    od0.header.stamp.sec, od0.header.stamp.nanosec = 1, 0
    rows.append((2, -1, serialize_message(od0)))
    for k in range(130):
        m = Imu()
        m.header.stamp.sec, m.header.stamp.nanosec = 99, 0
        m.header.stamp.sec, m.header.stamp.nanosec = 90 + k // 100, (k % 100) * 10**7
        m.linear_acceleration.x = 9.81
        rows.append((1, k * 10**7, serialize_message(m)))
    rng = np.random.default_rng(3)
    for k in range(8):
        od, cl = _pair(k, rng)
        ts = 2 * 10**9 + k * 10**8
        rows.append((2, ts, serialize_message(od)))
        rows.append((3, ts + 1, serialize_message(cl)))
        if with_obstacles:
            from sensor_msgs_py import point_cloud2 as pc2
            rec = _recorded(od, cl)
            rows.append((4, ts + 2, serialize_message(
                pc2.create_cloud_xyz32(od.header, rec.astype(np.float32).tolist()))))
    c.executemany('insert into messages(topic_id,timestamp,data) values(?,?,?)', rows)
    c.commit()
    c.close()


def _register(monkeypatch, name, db, obstacles):
    spec = dict(db=db, cloud='/cloud', odom='/odom', imu='/imu')
    if obstacles:
        spec['obstacles'] = '/obs'
    monkeypatch.setitem(R.BAGS, name, spec)


def test_prepare_auto_fits_when_bag_has_scan_obstacles(tmp_path, monkeypatch):
    db = str(tmp_path / 'with.db3')
    _write_bag(db, True)
    _register(monkeypatch, 'fx_obs', db, True)
    frames, src, _ = R.prepare('fx_obs')               # default level='auto'
    assert src == 'fit' and frames
    assert R.PREP_INFO['fx_obs']['fit']['cost_after_m'] < R.PREP_INFO['fx_obs']['fit']['cost_before_m']
    assert np.allclose(frames[0]['R_lv'], R_TRUE, atol=2.5e-3)
    legacy, lsrc, _ = R.prepare('fx_obs', level='bag')
    assert lsrc == 'imu' and np.allclose(legacy[0]['R_lv'], R0, atol=1e-9)


def test_prepare_without_scan_obstacles_keeps_bag_levelling(tmp_path, monkeypatch):
    db = str(tmp_path / 'without.db3')
    _write_bag(db, False)
    _register(monkeypatch, 'fx_noobs', db, False)
    frames, src, _ = R.prepare('fx_noobs')
    assert src == 'imu' and np.allclose(frames[0]['R_lv'], R0, atol=1e-9)
    with pytest.raises(ValueError):
        R.prepare('fx_noobs', level='fit')
    p = tmp_path / 'r.json'
    p.write_text(json.dumps(R_TRUE.tolist()))
    frames, src, _ = R.prepare('fx_noobs', level=str(p))
    assert src == 'explicit' and np.allclose(frames[0]['R_lv'], R_TRUE)
