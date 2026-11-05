"""Unit tests for sim_model.py (pure logic, no ROS). Run: python3 -m pytest -q tests/controllers/test_sim_model.py"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_model as sm  # noqa: E402


def run(model, cmd, secs, dt=0.02):
    t = 0.0
    while t < secs:
        model.step(t, dt, cmd, t)
        t += dt
    return model


def test_ideal_straight():
    m = run(sm.RobotModel('ideal', 0, 0, 0), (0.3, 0.0), 2.0)
    assert m.x == pytest.approx(0.6, abs=0.01) and abs(m.y) < 1e-9


def test_ideal_arc_exact():
    m = run(sm.RobotModel('ideal', 0, 0, 0), (0.3, 0.3), math.pi / 0.3)   # half circle r=1
    assert m.y == pytest.approx(2.0, abs=0.02) and abs(m.x) < 0.02


def test_cmd_timeout_stops():
    m = sm.RobotModel('ideal', 0, 0, 0)
    m.step(1.0, 0.02, (0.3, 0.0), 0.0)        # stamp 1 s old > 0.15 s
    assert m.v == 0.0


def test_mc_hyp_gain_and_dead_time():
    m = sm.RobotModel('mc_hyp', 0, 0, 0)
    run(m, (0.3, 0.0), 0.38)
    assert m.v == 0.0                          # still inside the 0.4 s dead time
    run(m, (0.3, 0.0), 3.0)
    assert m.v == pytest.approx(0.34 + 0.53 * 0.3, abs=0.005)   # 0.499 = R 1.66 at 0.30
    assert sm.RobotModel.mc_forward(0.8) == pytest.approx(0.764, abs=1e-3)
    assert sm.RobotModel.mc_forward(0.01) == 0.0


def test_raycast_and_box():
    occ = np.zeros((100, 100), dtype=bool)
    w = sm.World(occ, (0.0, 0.0), 0.05)
    w.boxes['b'] = (3.0, 2.5, 0.2, 0.2)
    b, r, hit = w.raycast(1.0, 2.5, 0.0)
    i = int(np.argmin(np.abs(b)))
    assert hit[i] and r[i] == pytest.approx(1.8, abs=0.03)
    assert not hit[0]


def test_polygon_clearance_sign():
    fp = sm.transform_footprint(0, 0, 0)
    assert sm.polygon_clearance(fp, np.array([[0.0, 0.0]])) < 0          # inside
    assert sm.polygon_clearance(fp, np.array([[0.27, 0.0]])) == pytest.approx(0.10, abs=1e-6)


def test_path_projection():
    p = sm.PathRef([(0, 0), (1, 0), (1, 1)])
    d, s = p.project(0.5, 0.2)
    assert d == pytest.approx(0.2) and s == pytest.approx(0.5)
    (x, y), yaw = p.point_at(1.5)
    assert (x, y) == pytest.approx((1.0, 0.5)) and yaw == pytest.approx(math.pi / 2)


def test_sign_flips():
    rate, n = sm.sign_flips_per_s([0, 1, 2, 3, 4], [0.1, -0.1, 0.01, -0.2, 0.3])
    assert n == 2 and rate == pytest.approx(0.5)
