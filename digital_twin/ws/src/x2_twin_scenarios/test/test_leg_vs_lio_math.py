import math

import numpy as np
import pytest

from x2_twin_scenarios.analysis.leg_vs_lio import quat_to_R, ratio, rot_angle


def _quat_about(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(axis * math.sin(angle / 2)), math.cos(angle / 2))


@pytest.mark.parametrize('axis', [(0, 0, 1), (1, 0, 0), (0.3, -0.5, 0.8)])
def test_rotation_angle_is_independent_of_the_axis(axis):
    r = quat_to_R(_quat_about(axis, 0.8))
    assert rot_angle(r) == pytest.approx(0.8, abs=1e-9)


def test_relative_rotation_ignores_a_tilted_mounting():
    tilt = quat_to_R(_quat_about((0, 1, 0), 1.2))                  # sensor frame tilted by 69 deg
    turn = quat_to_R(_quat_about((0, 0, 1), 0.5))
    assert rot_angle(tilt.T @ (tilt @ turn)) == pytest.approx(0.5, abs=1e-9)


def test_ratio_guards_small_commanded_values():
    assert ratio(1.0, 0.02) == '-'
    assert ratio(1.0, 0.5) == '2.00'
