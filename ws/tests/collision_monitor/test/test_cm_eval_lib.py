"""Unit tests for lib/cm_eval_lib.py (emulation of nav2_collision_monitor 1.1.18 decision logic)."""
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
import cm_eval_lib as L  # noqa: E402

SQUARE = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]])
CFG = os.path.join(os.path.dirname(__file__), '..', 'config', 'collision_monitor_x2_offline.yaml')


def polys():
    p = yaml.safe_load(open(CFG))['collision_monitor']['ros__parameters']
    return L.polygons_from_params(p)


def test_point_in_polygon():
    pts = np.array([[0, 0], [0.99, 0.99], [1.01, 0], [0, -1.5], [-0.5, 0.2]])
    assert L.point_in_polygon_mask(pts, SQUARE).tolist() == [True, True, False, False, True]
    assert L.point_in_polygon_count(np.zeros((0, 2)), SQUARE) == 0


def test_circle_count_strict():
    pts = np.array([[0.45, 0.0], [0.44, 0.0], [0.0, 0.1]])
    assert L.circle_count(pts, 0.45) == 2  # strict < like Circle::getPointsInside


def test_height_filter_inclusive():
    p = np.array([[1, 0, 0.05], [1, 0, 0.049], [1, 0, 2.0], [1, 0, 2.01]])
    assert L.height_filter(p, 0.05, 2.0).shape == (2, 2)


def test_transform_cloud_identity_and_shift():
    p = np.array([[1.0, 2.0, 3.0]])
    out = L.transform_cloud(p, np.eye(3), [0.0, 0.0, 1.0])
    assert out.tolist() == [[1.0, 2.0, 4.0]]


def test_stop_needs_more_than_max_points():
    ps = polys()
    three = np.array([[0.5, 0.0]] * 3)
    four = np.array([[0.5, 0.0]] * 4)
    a3, v3, _ = L.emulate_decision(three, (0.3, 0, 0), ps)
    a4, v4, n4 = L.emulate_decision(four, (0.3, 0, 0), ps)
    assert a3 != L.STOP  # 3 points: not > max_points(3) for stop; approach may still slow down
    assert a4 == L.STOP and v4 == (0.0, 0.0, 0.0) and n4 == 'PolygonStop'


def test_slowdown_zone():
    ps = polys()
    pts = np.array([[1.3, 0.0]] * 10)  # in slow zone, outside stop, beyond approach horizon (0.45+0.45)
    a, v, n = L.emulate_decision(pts, (0.3, 0, 0), ps)
    assert a == L.SLOWDOWN and v[0] == pytest.approx(0.09) and n == 'PolygonSlow'


def test_approach_scales_with_time():
    ps = [dict(name='A', action='approach', radius=0.45, max_points=3,
               time_before_collision=1.5, simulation_time_step=0.1)]
    pts = np.array([[0.75, 0.0]] * 10)  # gap 0.30 m at 0.3 m/s -> ~1.0 s
    a, v, _ = L.emulate_decision(pts, (0.3, 0, 0), ps)
    assert a == L.APPROACH
    assert 0.15 < v[0] < 0.25
    a2, v2, _ = L.emulate_decision(pts, (0.0, 0, 0), ps)  # not moving -> no collision predicted
    assert a2 == L.PASS


def test_approach_uses_commanded_velocity_only():
    """D-05: actual speed > commanded is invisible to the approach model."""
    pts = np.array([[1.0, 0.0]] * 10)
    t_cmd = L.collision_time_circle(pts, (0.30, 0, 0), 0.45, 3, 1.5, 0.1)
    t_act = L.collision_time_circle(pts, (0.30 * 1.61, 0, 0), 0.45, 3, 1.5, 0.1)
    # gap 0.55 m: at the commanded 0.30 m/s contact is 1.83 s away (> 1.5 s horizon -> no action),
    # at the actual 1.61x speed it is ~1.1 s away -> approach would have had to slow down already.
    assert t_cmd == -1.0
    assert 0.9 < t_act < 1.3


def test_empty_cloud_passes():
    a, v, _ = L.emulate_decision(np.zeros((0, 2)), (0.3, 0, 0), polys())
    assert a == L.PASS and v == (0.3, 0.0, 0.0)


def test_classify_output():
    vin = (0.3, 0.0, 0.0)
    assert L.classify_output((0, 0, 0), vin, 0.3) == L.STOP
    assert L.classify_output((0.3, 0, 0), vin, 0.3) == L.PASS
    assert L.classify_output((0.09, 0, 0), vin, 0.3) == L.SLOWDOWN
    assert L.classify_output((0.2, 0, 0), vin, 0.3) == L.APPROACH


def test_latency_stats():
    s = L.latency_stats([0.1, 0.2, None, float('nan'), 0.3])
    assert s['n'] == 3 and s['p50'] == pytest.approx(0.2) and s['max'] == pytest.approx(0.3)
    assert L.latency_stats([])['n'] == 0
