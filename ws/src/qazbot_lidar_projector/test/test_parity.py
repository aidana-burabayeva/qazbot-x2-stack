"""Parity of the C++ core library against the Python reference candidates/d04/level_bridge.py (D-04 variant C).

The recorded fixture (50 clouds, lidar IMU, LIO odometry of bag d04_static_bag_204153) is replayed through
  * the real Python LevelBridge.on_imu / on_odom / on_cloud (called with a stub `self`, as tests/d04/replay_d04.py
    does; R_lv comes from the fixture's own first second of IMU, identical for both sides), and
  * `parity_dump` (C++ library: levelling, pose tracker, cloud processor).
Every output is compared: R_lv, /odom_level (pose, twist, covariance), scan ranges, obstacle points, clearing points.

TOLERANCES (derived, nothing is tuned).  eps = 2^-53 (double unit roundoff).  The inputs are float32 widened
exactly to double, both sides compute in double and round the outputs to float32.  The two implementations can
only differ through double rounding of identical formulas: numpy's matmul/BLAS/atan2 versus
std C++ (the C++ core is built with -ffp-contract=off so that it does not add FMA rounding differences).
Per point (P = input vector, p = levelled position, R entries <= 1):
  * levelled coordinate  |dq|  <= 4 eps (|P|_1 + |p|_1)            (3-term dot product + subtraction, gamma_4)
  * body coordinate      |db|  <= sqrt2 |dq| + 2 eps (|qx| + |qy|)  (rotation by -yaw)
  * range                |dr|  <= |db| ;  bearing |da| <= |db| / r + pi eps
  * bin coordinate u = (a + pi) / 2pi * N:  |du| <= |da| N / 2pi + 2 eps N
  A point whose u lies within |du| of an integer, or whose z / r lies within the propagated error of a band
  bound or of range_min / range_max, can legitimately fall on either side on the two implementations
  ("ambiguous point"): that point's bin and its two neighbours are "ambiguous bins".
  * clearing end of a ray = nextafter(r t, 0), t from range_max / r or Z / slope: relative error
    |dr|/r + (|dslope| / |slope|) + 4 eps ; two rays of one bin whose ends differ by less than the sum of their
    errors are a "tie": either implementation may keep either ray (this is common for far points, all of which
    end at nextafter(range_max)).
Expected outcome and acceptance:
  * every float32 output value equal within 1 float32 ulp + the derived absolute error of its point
    (float32 rounding of two doubles that differ by < the derived error moves the result by at most 1 ulp);
  * any other difference (different bin occupancy, different selected ray) is accepted ONLY inside an ambiguous
    bin; it is counted and reported.  Unexplained differences fail the test.
Odometry: pose |dp| <= 4 eps |p|_1 ; yaw |dyaw| <= 8 eps / |f_xy| (f = R_lv R_wb fwd) ; twist
  |dv| <= (2 sqrt2 (dp + dp0) + 2 |d| dyaw) / dt, |dwz| <= 2 (dyaw + dyaw0) / dt ; covariance
  |dC| <= 12 eps (|B| |Sigma| |B|^T) elementwise (two chained 6-term dot products, gamma_6 each).
R_lv: |dR| <= 4 n eps with n = number of IMU samples averaged (worst-case summation error of the mean).
"""
import importlib.util
import math
import os
import struct
import subprocess
import sys
import tempfile
import types
from pathlib import Path

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'tools'))
import fixture  # noqa: E402

CANDIDATE = os.environ.get('CANDIDATE', str(Path(__file__).resolve().parents[4] / 'docs/engineering/d04_d05_20261102/candidate/level_bridge.py'))
FIXTURE = os.environ.get('FIXTURE', os.path.join(HERE, 'data', 'static_204153_w50.bin'))
PARITY_DUMP = os.environ.get('PARITY_DUMP', '')
EPS = 2.0 ** -53
N = 1440


def load_candidate():
    spec = importlib.util.spec_from_file_location('level_bridge_candidate', CANDIDATE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Logger:
    def info(self, *_):
        pass

    warn = info


def python_reference(mod, events, range_max, every):
    mod.RANGE_MAX = range_max
    s = types.SimpleNamespace(acc=[], t_acc0=None, R_lv=None, R_wb=None, poses={}, track=__import__('collections').deque(),
                              last_key=None, tf=None, sub_imu=object(), every=every, n_cloud=0,
                              pub_scan=Capture(), pub_obstacles=Capture(), pub_clearing=Capture(), pub_odom=Capture(),
                              get_logger=lambda: Logger(), destroy_subscription=lambda _s: None)
    odom_in = {}
    for e in events:
        stamp = Time(sec=e.sec, nanosec=e.nanosec)
        if e.kind == 'imu':
            m = Imu()
            m.header.stamp = stamp
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = e.data
            mod.LevelBridge.on_imu(s, m)
        elif e.kind == 'odom':
            m = Odometry()
            m.header.stamp = stamp
            (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) = e.data[0]
            (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
             m.pose.pose.orientation.w) = e.data[1]
            m.pose.covariance = list(e.data[2])
            before = len(s.pub_odom.msgs)
            mod.LevelBridge.on_odom(s, m)
            if len(s.pub_odom.msgs) > before:
                odom_in[(e.sec, e.nanosec)] = e.data
        else:
            xyz, dense = e.data
            m = PointCloud2()
            m.header.stamp = stamp
            m.height, m.width = 1, len(xyz)
            m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
            m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(xyz), dense
            m.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
            mod.LevelBridge.on_cloud(s, m)
    return s, odom_in


def cloud_to_array(m):
    return np.frombuffer(bytes(m.data), dtype='<f4').reshape(-1, 3)


def read_dump(path):
    buf = open(path, 'rb').read()
    pos, R, odoms, clouds = 0, None, [], []
    while pos < len(buf):
        tag = buf[pos:pos + 1]
        pos += 1
        if tag == b'R':
            R = np.array(struct.unpack_from('<9d', buf, pos)).reshape(3, 3)
            pos += 72
        elif tag == b'O':
            key, = struct.unpack_from('<q', buf, pos)
            d = struct.unpack_from('<7d', buf, pos + 8)
            cov = np.array(struct.unpack_from('<36d', buf, pos + 64)).reshape(6, 6)
            pos += 64 + 288
            odoms.append((key, d, cov))
        else:
            key, = struct.unpack_from('<q', buf, pos)
            pos += 8
            arrs = []
            for _ in range(3):
                n, = struct.unpack_from('<I', buf, pos)
                pos += 4
                arrs.append(np.frombuffer(buf, dtype='<f4', count=n, offset=pos).copy())
                pos += 4 * n
            clouds.append((key, arrs[0], arrs[1].reshape(-1, 3), arrs[2].reshape(-1, 3)))
    return R, odoms, clouds


def ulp32(v):
    return np.spacing(np.abs(np.asarray(v, dtype=np.float32))).astype(np.float64)


def point_errors(P, Rlv, p_lv, yaw, mod, range_max):
    """Per-point derived error bounds and the ambiguity flags described in the module docstring."""
    P64 = P.astype(np.float64)
    Q = P64 @ Rlv.T - p_lv
    dq = 4 * EPS * (np.abs(P64).sum(1) + np.abs(p_lv).sum())
    c, s = math.cos(-yaw), math.sin(-yaw)
    bx, by = c * Q[:, 0] - s * Q[:, 1], s * Q[:, 0] + c * Q[:, 1]
    r = np.hypot(bx, by)
    a = np.arctan2(by, bx)
    db = math.sqrt(2) * dq + 2 * EPS * (np.abs(Q[:, 0]) + np.abs(Q[:, 1]))
    with np.errstate(divide='ignore', invalid='ignore'):
        da = db / r + math.pi * EPS
    du = da * N / (2 * math.pi) + 2 * EPS * N
    u = (a + math.pi) / (2 * math.pi) * N
    edge = np.abs(u - np.rint(u)) < du
    flat = np.hypot(Q[:, 0], Q[:, 1])
    z = Q[:, 2]
    m = mod.FLOOR_MARGIN_PER_M
    dz = dq + m * db
    thr = ((np.abs(z - (mod.Z_MIN + m * flat)) < dz) | (np.abs(z - (mod.Z_MIN + m * r)) < dz) |
           (np.abs(z - mod.Z_MAX) < dz) | (np.abs(r - mod.RANGE_MIN) < db) | (np.abs(r - range_max) < db))
    idx = ((a + math.pi) / (2 * math.pi) * N).astype(int) % N
    with np.errstate(divide='ignore', invalid='ignore'):
        slope = z - m * r
        rel = db / r + dz / np.abs(slope) + 4 * EPS
    ok = np.isfinite(Q).all(axis=1) & (r > 0)
    return dict(Q=Q, bx=bx, by=by, r=r, a=a, z=z, idx=idx, edge=edge & ok, thr=thr & ok, db=db, rel=rel, ok=ok,
                end=None)


def ambiguous_bins(info, mod, range_max):
    amb = np.zeros(N, dtype=bool)
    for k in np.flatnonzero(info['edge'] | info['thr']):
        for d in (-1, 0, 1):
            amb[(info['idx'][k] + d) % N] = True
    # ties among clearing ends
    ok = info['ok']
    r, z = info['r'][ok], info['z'][ok]
    idx = info['idx'][ok]
    m = mod.FLOOR_MARGIN_PER_M
    t = np.minimum(1.0, range_max / r)
    slope = z - m * r
    down, up = slope < 0, z > 0
    t[down] = np.minimum(t[down], mod.Z_MIN / slope[down])
    t[up] = np.minimum(t[up], mod.Z_MAX / z[up])
    end = np.nextafter(r * t, 0.0)
    tol = end * info['rel'][ok]
    order = np.lexsort((-end, idx))
    idx_s, end_s, tol_s = idx[order], end[order], tol[order]
    same = idx_s[1:] == idx_s[:-1]
    tie = same & ((end_s[:-1] - end_s[1:]) <= (tol_s[:-1] + tol_s[1:]))
    for b in idx_s[1:][tie]:
        for d in (-1, 0, 1):
            amb[(b + d) % N] = True
    return amb


def bin_of_rows(xy):
    a = np.arctan2(xy[:, 1].astype(np.float64), xy[:, 0].astype(np.float64))
    return ((a + math.pi) / (2 * math.pi) * N).astype(int) % N


def compare_rows(py, cp, amb, dmax, stats, what, bins_py=None):
    """Rows are in bin order on both sides; match by bearing, compare within ulp32 + derived error."""
    bp, bc = bin_of_rows(py), bin_of_rows(cp)
    i = j = 0
    while i < len(py) or j < len(cp):
        if i < len(py) and j < len(cp):
            da = abs(math.atan2(py[i, 1], py[i, 0]) - math.atan2(cp[j, 1], cp[j, 0]))
            same_dir = da < math.pi / N
        else:
            same_dir = False
        if same_dir:
            tol = np.maximum(ulp32(np.maximum(abs(py[i, :2]), abs(cp[j, :2]))), 0) + dmax
            if np.all(np.abs(py[i, :2].astype(np.float64) - cp[j, :2]) <= tol):
                stats[what + '_within_tol'] += 1
            elif amb[bp[i]] or amb[bc[j]]:
                stats[what + '_explained'] += 1
                dist = float(np.hypot(*(py[i, :2].astype(np.float64) - cp[j, :2])))
                dr = abs(float(np.hypot(*py[i, :2].astype(np.float64))) - float(np.hypot(*cp[j, :2].astype(np.float64))))
                stats[what + '_explained_max_dist_m'] = max(stats.get(what + '_explained_max_dist_m', 0.0), dist)
                stats[what + '_explained_max_dr_m'] = max(stats.get(what + '_explained_max_dr_m', 0.0), dr)
            else:
                stats[what + '_UNEXPLAINED'] += 1
            i += 1
            j += 1
        else:
            # one side has a row the other does not
            if i < len(py) and (j >= len(cp) or math.atan2(py[i, 1], py[i, 0]) <= math.atan2(cp[j, 1], cp[j, 0])):
                b, i = bp[i], i + 1
            else:
                b, j = bc[j], j + 1
            stats[what + ('_explained' if amb[b] else '_UNEXPLAINED')] += 1


@pytest.fixture(scope='module')
def events():
    return fixture.read(FIXTURE)


@pytest.mark.skipif(not os.path.exists(PARITY_DUMP), reason='parity_dump not built')
@pytest.mark.parametrize('range_max', [8.0, 25.0])
def test_cpp_matches_python_candidate(events, range_max):
    mod = load_candidate()
    # Python and C++ both process EVERY cloud (scan_every 1) so that all 50 clouds are compared.
    s, odom_in = python_reference(mod, events, range_max, 1)
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'cpp.bin')
        subprocess.run([PARITY_DUMP, FIXTURE, out, repr(range_max), '1'], check=True, timeout=120)
        R_cpp, odoms_cpp, clouds_cpp = read_dump(out)

    n_imu = sum(1 for e in events if e.kind == 'imu')
    assert s.R_lv is not None and R_cpp is not None
    n_used = len(s.acc)
    assert np.max(np.abs(R_cpp - s.R_lv)) <= 4 * n_used * EPS, np.max(np.abs(R_cpp - s.R_lv))

    # ---- /odom_level
    py_odom = {(m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec): m for m in s.pub_odom.msgs}
    assert set(py_odom) == {k for k, _, _ in odoms_cpp}, 'twist-bearing odometry set differs'
    assert len(odoms_cpp) > 10
    max_rel = 0.0
    for key, (x, y, z, yaw, vx, vy, wz), cov in odoms_cpp:
        m = py_odom[key]
        sec, ns = divmod(key, 10 ** 9)
        p, q, sigma_in = odom_in[(sec, ns)]
        p = np.array(p)
        Rwb = mod.quat_to_rot(types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
        f = s.R_lv @ Rwb @ mod.LIDAR_FWD_IN_IMU
        dyaw = 8 * EPS / max(math.hypot(f[0], f[1]), 1e-3)
        dp = 4 * EPS * np.abs(p).sum()
        assert abs(m.pose.pose.position.x - x) <= dp and abs(m.pose.pose.position.y - y) <= dp
        assert abs(m.pose.pose.position.z - z) <= dp
        pyaw = 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
        assert abs(math.remainder(pyaw - yaw, 2 * math.pi)) <= dyaw + 4 * EPS
        # twist: baseline sample of the python track is not exposed; use the full derived bound with dt >= baseline
        dt = mod.TWIST_BASELINE_S
        dist = 1.0   # |d| < 1 m over the baseline for this walking-speed fixture (checked below)
        assert math.hypot(vx, vy) * dt < dist
        dv = (2 * math.sqrt(2) * 2 * dp + 2 * dist * dyaw) / dt
        assert abs(m.twist.twist.linear.x - vx) <= dv and abs(m.twist.twist.linear.y - vy) <= dv
        assert abs(m.twist.twist.angular.z - wz) <= 2 * 2 * dyaw / dt + 4 * EPS
        B = np.kron(np.eye(2), s.R_lv)
        pc = np.array(m.pose.covariance).reshape(6, 6)
        bound = 12 * EPS * (np.abs(B) @ np.abs(np.array(sigma_in).reshape(6, 6)) @ np.abs(B).T)
        # python input covariance is the fixture's; the C++ result rotated the same input
        assert np.all(np.abs(pc - cov) <= bound + 1e-300)
        max_rel = max(max_rel, float(np.max(np.abs(pc - cov))))

    # ---- scan / obstacles / clearing
    py_scan = {(m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec): m for m in s.pub_scan.msgs}
    py_obs = {(m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec): m for m in s.pub_obstacles.msgs}
    py_clr = {(m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec): m for m in s.pub_clearing.msgs}
    assert set(py_scan) == {k for k, *_ in clouds_cpp}, 'processed cloud set differs'
    assert len(clouds_cpp) >= 35
    ev_by_key = {e.sec * 10 ** 9 + e.nanosec: e for e in events if e.kind == 'cloud'}
    odom_by_key = {}
    for e in events:
        if e.kind == 'odom':
            odom_by_key[e.sec * 10 ** 9 + e.nanosec] = e
    stats = {k: 0 for k in ('range_equal', 'range_within_ulp', 'range_explained', 'range_UNEXPLAINED',
                            'obs_within_tol', 'obs_explained', 'obs_UNEXPLAINED',
                            'clr_within_tol', 'clr_explained', 'clr_UNEXPLAINED')}
    n_amb_pts = n_pts = 0
    for key, ranges_c, obs_c, clr_c in clouds_cpp:
        xyz = ev_by_key[key].data[0]
        o = odom_by_key[key].data
        Rwb = mod.quat_to_rot(types.SimpleNamespace(x=o[1][0], y=o[1][1], z=o[1][2], w=o[1][3]))
        p_lv = s.R_lv @ np.array(o[0])
        yaw = math.atan2(*(s.R_lv @ Rwb @ mod.LIDAR_FWD_IN_IMU)[[1, 0]])
        info = point_errors(xyz, s.R_lv, p_lv, yaw, mod, range_max)
        amb = ambiguous_bins(info, mod, range_max)
        n_amb_pts += int((info['edge'] | info['thr']).sum())
        n_pts += len(xyz)
        dmax = float(np.max(info['db'][info['ok']] + info['r'][info['ok']] * 0) if info['ok'].any() else 0.0)
        dmax += float(np.max(info['db'][info['ok']] if info['ok'].any() else 0))   # + r*da ~ db
        ranges_p = np.array(py_scan[key].ranges, dtype=np.float32)
        # scan ranges, bin by bin
        for b in range(N):
            rp, rc = float(ranges_p[b]), float(ranges_c[b])
            if rp == rc:
                stats['range_equal'] += 1
            elif np.isfinite(rp) and np.isfinite(rc) and abs(rp - rc) <= ulp32(max(rp, rc)) + dmax:
                stats['range_within_ulp'] += 1
            elif amb[b]:
                stats['range_explained'] += 1
            else:
                stats['range_UNEXPLAINED'] += 1
        op, cp_ = cloud_to_array(py_obs[key]), obs_c
        # obstacle rows <-> finite range bins on each side (same bins by construction)
        bins_p, bins_c = np.flatnonzero(np.isfinite(ranges_p)), np.flatnonzero(np.isfinite(ranges_c))
        i = j = 0
        while i < len(bins_p) or j < len(bins_c):
            if i < len(bins_p) and j < len(bins_c) and bins_p[i] == bins_c[j]:
                tol = ulp32(np.maximum(abs(op[i, :2]), abs(cp_[j, :2]))) + dmax
                if np.all(np.abs(op[i, :2].astype(np.float64) - cp_[j, :2]) <= tol):
                    stats['obs_within_tol'] += 1
                elif amb[bins_p[i]]:
                    stats['obs_explained'] += 1
                    dist = float(np.hypot(*(op[i, :2].astype(np.float64) - cp_[j, :2])))
                    stats['obs_explained_max_dist_m'] = max(stats.get('obs_explained_max_dist_m', 0.0), dist)
                else:
                    stats['obs_UNEXPLAINED'] += 1
                assert op[i, 2] == 0 and cp_[j, 2] == 0
                i += 1
                j += 1
            else:
                b = bins_p[i] if (j >= len(bins_c) or (i < len(bins_p) and bins_p[i] < bins_c[j])) else bins_c[j]
                stats['obs_explained' if amb[b] else 'obs_UNEXPLAINED'] += 1
                if i < len(bins_p) and bins_p[i] == b:
                    i += 1
                else:
                    j += 1
        compare_rows(cloud_to_array(py_clr[key]), clr_c, amb, dmax, stats, 'clr')
    print(f'\nPARITY range_max={range_max}: clouds={len(clouds_cpp)} imu_msgs={n_imu} points={n_pts} '
          f'ambiguous_points={n_amb_pts} max|dCov|={max_rel:.3g}\n  {stats}')
    report = os.environ.get('PARITY_REPORT')
    if report:
        with open(report, 'a') as f:
            f.write(f'range_max={range_max} clouds={len(clouds_cpp)} points={n_pts} ambiguous_points={n_amb_pts} '
                    f'{stats}\n')
    for k, v in stats.items():
        if k.endswith('UNEXPLAINED'):
            assert v == 0, (k, v, stats)
    assert stats['range_equal'] + stats['range_within_ulp'] > 0.99 * N * len(clouds_cpp)
    # A4: "explained" rows (ulp-level arg-max ties among clearing rays / bins next to a threshold) must stay small
    # in number and in size. Within a bin the ray may only differ in bearing (<= one bin arc at range_max) and
    # not in length (the length is the bin-wide min(end, obstacle) in both implementations).
    bin_arc = range_max * 2 * math.pi / N
    for what in ('obs', 'clr'):
        assert stats.get(what + '_explained_max_dist_m', 0.0) <= bin_arc, (what, stats)
    assert stats.get('clr_explained_max_dr_m', 0.0) <= 1e-5, stats
    n_rows = stats['obs_within_tol'] + stats['obs_explained'] + stats['clr_within_tol'] + stats['clr_explained']
    n_expl = stats['obs_explained'] + stats['clr_explained'] + stats['range_explained']
    assert n_expl <= 0.05 * n_rows, (n_expl, n_rows, stats)
    assert stats['range_explained'] <= 0.001 * N * len(clouds_cpp), stats
