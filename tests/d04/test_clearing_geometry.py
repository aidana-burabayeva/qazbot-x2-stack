"""D-04 candidate: unit tests for the bounded-ray clearing geometry.

Run (ROS env needed only for the module imports):
    source /opt/ros/humble/setup.bash
    nice -n 19 python3 -m pytest -q /home/agi/control_backend/tests/d04
"""
import ast
import math
import types

import numpy as np
import pytest

from replay_d04 import CANDIDATE, RUNNING, load_bridge, stub_node, bridge_class

C = load_bridge(CANDIDATE, 'lb_candidate_t')
A = load_bridge(RUNNING, 'lb_running_t')
RMAX = 8.0                          # launch range_max
H = -0.97                           # floor height below the LiDAR IMU (measured: -0.969 udp bag)


def pt(bearing_deg, flat, z):
    a = math.radians(bearing_deg)
    return [flat * math.cos(a), flat * math.sin(a), z]


def bearings(xy):
    return np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))


def bins_of(xy):
    a = np.arctan2(xy[:, 1], xy[:, 0])
    return set((((a + math.pi) / (2 * math.pi) * C.N_BINS).astype(int) % C.N_BINS).tolist())


# --- edge sectors -------------------------------------------------------------------------
@pytest.mark.parametrize('b', [-58.8, -55.0, -50.0, -46.5, 45.0, 49.0, 52.0])
def test_edge_bearing_with_real_point_emits_clearing_ray(b):
    q = np.array([pt(b, 1.5, H)])                  # floor return at an edge bearing
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert len(out) == 1
    assert bearings(out)[0] == pytest.approx(b, abs=1e-9)


@pytest.mark.parametrize('b', [-58.8, -50.0, 49.0, 52.0])
def test_running_file_drops_edge_rays_documenting_d04(b):
    q = np.array([pt(b, 1.5, H)])
    assert len(A.clearing_endpoints(q, 0.0, RMAX)) == 0


def test_inside_old_window_candidate_equals_running_bit_for_bit():
    rng = np.random.default_rng(4)
    n = 5000
    b = rng.uniform(-45.9, 44.4, n)
    r = rng.uniform(0.2, 12.0, n)
    z = rng.uniform(-1.1, 1.2, n)
    q = np.array([pt(bb, rr, zz) for bb, rr, zz in zip(b, r, z)])
    for yaw in (0.0, 0.7, -2.5):
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        qw = q @ Rz.T
        np.testing.assert_array_equal(C.clearing_endpoints(qw, yaw, RMAX), A.clearing_endpoints(qw, yaw, RMAX))


def test_yaw_is_removed_before_binning():
    yaw = math.radians(30)
    q = np.array([pt(30 + 50, 1.5, H)])                   # world bearing 80 = body +50
    out = C.clearing_endpoints(q, yaw, RMAX)
    assert bearings(out)[0] == pytest.approx(50.0, abs=1e-9)


# --- blind bins: absence is not free --------------------------------------------------------
def test_no_ray_for_bearings_without_points():
    q = np.array([pt(b, 2.0, H) for b in np.arange(-55, 52, 0.25) + 0.1])   # bin centres, not edges
    out = C.clearing_endpoints(q, 0.0, RMAX)
    bb = bearings(out)
    assert bb.min() >= -54.9 - 1e-9 and bb.max() <= 51.85 + 1e-9
    assert bins_of(out) <= bins_of(q[:, :2])


def test_empty_and_nonfinite_cloud_emit_nothing():
    assert len(C.clearing_endpoints(np.empty((0, 3)), 0.0, RMAX)) == 0
    q = np.array([[np.nan, 1, 0], [np.inf, 0, 0], [0.0, 0.0, -0.5]])   # NaN, inf, r == 0
    assert len(C.clearing_endpoints(q, 0.0, RMAX)) == 0


def test_rear_and_side_points_only_clear_their_own_bins():
    q = np.array([pt(170.0, 1.0, 0.0), pt(-90.0, 1.0, 0.0)])   # e.g. a rear return
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert bins_of(out) == bins_of(q[:, :2])


# --- floor-band rule ------------------------------------------------------------------------
@pytest.mark.parametrize('b', [0.0, -55.0, 50.0])
@pytest.mark.parametrize('r', [0.7, 1.0, 2.0, 3.0])
def test_floor_ray_stops_where_it_leaves_the_band(b, r):
    q = np.array([pt(b, r, H)])
    out = C.clearing_endpoints(q, 0.0, RMAX)
    t = C.Z_MIN / (H - C.FLOOR_MARGIN_PER_M * r)            # z(t)=t*H meets Z_MIN+m*t*r
    end = np.hypot(out[0, 0], out[0, 1])
    # ends at the band exit (to float precision, ~1 ulp from the planar re-scaling)
    assert end <= r * t * (1 + 1e-12) and end == pytest.approx(r * t, rel=1e-12)
    if r * t <= C.RANGE_MIN:
        pytest.fail('test geometry must give a ray longer than RANGE_MIN')


def test_floor_ray_shorter_than_range_min_is_dropped():
    q = np.array([pt(0.0, 0.6, H)])                      # band exit at ~0.39 m < 0.45
    assert len(C.clearing_endpoints(q, 0.0, RMAX)) == 0


def test_upward_ray_stops_at_z_max():
    q = np.array([pt(-50.0, 4.0, 2.0)])
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert np.hypot(*out[0, :2]) == pytest.approx(4.0 * C.Z_MAX / 2.0, rel=1e-12)


def test_ray_truncated_at_range_max():
    q = np.array([pt(51.0, 20.0, 0.0)])
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert np.hypot(*out[0, :2]) == pytest.approx(RMAX, rel=1e-12)
    assert np.hypot(*out[0, :2]) < RMAX


def test_in_band_return_caps_longest_ray_in_its_bin():
    b = -52.0
    q = np.array([pt(b, 0.9, 0.0),                # obstacle (in band) at 0.9 m
                  pt(b + 0.01, 6.0, 0.2)])        # same bin, longer ray over/through it
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert len(out) == 1
    assert np.hypot(*out[0, :2]) <= 0.9 * (1 + 1e-12)       # never past the hit (float precision)


def test_endpoints_are_planar_and_never_reach_the_hit():
    rng = np.random.default_rng(7)
    q = np.array([pt(b, r, z) for b, r, z in zip(rng.uniform(-60, 55, 3000), rng.uniform(0.3, 9, 3000),
                                                  rng.uniform(-1.0, 1.0, 3000))])
    out = C.clearing_endpoints(q, 0.0, RMAX)
    assert np.all(out[:, 2] == 0)
    r = np.hypot(q[:, 0], q[:, 1])
    hit = (r > C.RANGE_MIN) & (r < RMAX) & (q[:, 2] > C.Z_MIN + C.FLOOR_MARGIN_PER_M * r) & (q[:, 2] < C.Z_MAX)
    a = np.arctan2(q[:, 1], q[:, 0])
    idx = ((a + math.pi) / (2 * math.pi) * C.N_BINS).astype(int) % C.N_BINS
    nearest = np.full(C.N_BINS, np.inf)
    np.minimum.at(nearest, idx[hit], r[hit])
    ao = np.arctan2(out[:, 1], out[:, 0])
    io = ((ao + math.pi) / (2 * math.pi) * C.N_BINS).astype(int) % C.N_BINS
    assert np.all(np.hypot(out[:, 0], out[:, 1]) <= nearest[io] * (1 + 1e-12))
    assert np.all(np.hypot(out[:, 0], out[:, 1]) > C.RANGE_MIN)


# --- node path: marking span == clearing span -----------------------------------------------
def _cloud_msg(points_world):
    from sensor_msgs_py import point_cloud2 as pc2
    from std_msgs.msg import Header
    h = Header()
    h.stamp.sec, h.stamp.nanosec, h.frame_id = 100, 0, 'camera_init'
    return pc2.create_cloud_xyz32(h, np.asarray(points_world, np.float32).tolist())


def _run_on_cloud(mod, q_level):
    s = stub_node(mod)
    s.R_lv = np.eye(3)
    s.poses = {100 * 10**9: (0.0, 0.0, 0.0, np.zeros(3))}
    bridge_class(mod).on_cloud(s, _cloud_msg(q_level))
    from sensor_msgs_py import point_cloud2 as pc2
    rd = lambda m: pc2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(float)
    return rd(s.pub_obstacles.msg), rd(s.pub_clearing.msg)


def test_node_every_marked_edge_bearing_has_a_clearing_ray_in_candidate():
    # person-like returns at both edges + floor returns behind them, same bearings
    q = []
    for b in (-58.0, -52.0, 47.0, 51.5):
        q.append(pt(b, 0.6, -0.2))                   # in band, marks
        q.append(pt(b + 0.3, 1.8, H))                # floor return in a neighbouring bin
    obst, clear = _run_on_cloud(C, np.array(q))
    assert len(obst) == 4
    ob_b, cl_b = bearings(obst), bearings(clear)
    for b in (-58.0, -52.0, 47.0, 51.5):
        assert np.any(np.abs(cl_b - (b + 0.3)) < 0.01), b     # float32 message rounding
    assert cl_b.min() < -57.0 and cl_b.max() > 51.0
    _, clear_running = _run_on_cloud(A, np.array(q))
    assert len(clear_running) == 0 or (bearings(clear_running).min() >= -46.0 and
                                       bearings(clear_running).max() <= 44.5)
    assert ob_b.min() < -57.0                         # marking has no window in either file


# --- candidate scope guard --------------------------------------------------------------------
def test_candidate_changes_only_clearing_endpoints():
    ta, tc = ast.parse(open(RUNNING).read()), ast.parse(open(CANDIDATE).read())
    # Normalize the rename only; the archived candidate retains the historical name.
    def normalize(s):
        return (s.replace('X2LioPlanarBridge', 'LevelBridge')
                 .replace('x2_lio_planar_bridge', 'level_bridge')) if s is not None else None
    da = {normalize(getattr(n, 'name', ast.dump(n)[:60])): normalize(ast.dump(n)) for n in ta.body}
    dc = {normalize(getattr(n, 'name', ast.dump(n)[:60])): normalize(ast.dump(n)) for n in tc.body}
    changed = {k for k in da.keys() | dc.keys() if da.get(k) != dc.get(k)}
    assert changed == {'clearing_endpoints'}


def test_positional_call_signature_used_by_raw_scan_bridge_still_works():
    q = np.array([pt(10.0, 2.0, H)])
    assert C.clearing_endpoints(q, 0.0, RMAX).shape == (1, 3)
