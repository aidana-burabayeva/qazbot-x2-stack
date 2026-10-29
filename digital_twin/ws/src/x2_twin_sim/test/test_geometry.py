import math

import pytest

from x2_twin_sim.geometry import FrameAligner, compose, inverse, planar_pose, wrap_angle


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi, abs=1e-9) or wrap_angle(3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert wrap_angle(0.3) == pytest.approx(0.3)


def test_planar_pose_upright_matches_yaw():
    yaw = 0.7
    x, y, h = planar_pose(1.0, 2.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert (x, y) == (1.0, 2.0)
    assert h == pytest.approx(yaw)


def test_planar_pose_heading_survives_pitch():
    # pitch 0.3 rad about y, then yaw 1.0 about z: heading of the projected x axis is still 1.0
    cy, sy, cp, sp = math.cos(0.5), math.sin(0.5), math.cos(0.15), math.sin(0.15)
    qw, qx, qy, qz = cy * cp, -sy * sp, cy * sp, sy * cp       # q = yaw(z) * pitch(y)
    assert planar_pose(0, 0, qx, qy, qz, qw)[2] == pytest.approx(1.0, abs=1e-6)


def test_compose_inverse_identity():
    a = (1.0, -2.0, 0.9)
    x, y, th = compose(a, inverse(a))
    assert (x, y, th) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_frame_aligner_first_pose_is_start_and_motion_is_relative():
    al = FrameAligner(start=(-1.275, -4.825, 0.0))
    assert al.align((10.0, 3.0, 1.0)) == pytest.approx((-1.275, -4.825, 0.0), abs=1e-12)
    # the robot then drives 1 m along ITS heading (1.0 rad in the sim world): in the map frame that is 1 m along +x
    x, y, th = al.align((10.0 + math.cos(1.0), 3.0 + math.sin(1.0), 1.0))
    assert (x, y, th) == pytest.approx((-0.275, -4.825, 0.0), abs=1e-9)
