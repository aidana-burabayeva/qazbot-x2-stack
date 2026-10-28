"""Bounds on actual projected rays; no claim of real deskew-origin validation."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

PATH = Path(__file__).resolve().parents[2] / 'ws/src/lio/level_bridge.py'
spec = importlib.util.spec_from_file_location('i1_current_bridge', PATH)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_near_floor_does_not_authorize_far_clearing():
    end = bridge.clearing_endpoints([[1, 0, -1]], 0)
    assert end.shape == (1, 3)
    assert end[0, 0] == pytest.approx(0.65 / 1.035)
    assert end[0, 0] < 1.0 < 3.0


def test_independent_measured_far_beam_can_extend_short_floor_ray():
    end = bridge.clearing_endpoints([[1, 0, -1], [8, 0, 0]], 0)
    assert end.shape == (1, 3)
    assert end[0, 0] == pytest.approx(8)


def test_far_observed_horizontal_ray_can_clear_observed_area():
    end = bridge.clearing_endpoints([[8, 0, 0]], 0, range_max=8)
    assert end[0, 0] == pytest.approx(8)
    assert end[0, 0] <= 8


@pytest.mark.parametrize('points', [[], [[np.nan, 0, 0]], [[np.inf, 0, 0]], [[0, 0, 0]]])
def test_missing_invalid_or_empty_cloud_has_no_clearing(points):
    assert bridge.clearing_endpoints(points, 0).shape == (0, 3)


def test_no_dilation_from_partial_cloud():
    end = bridge.clearing_endpoints([[3, 0, 0]], 0)
    assert end.shape == (1, 3)
    assert end[0, 1] == 0


def test_fov_boundary_and_no_extrapolation():
    a = np.radians([-46.1, -46.0, 44.5, 44.6])
    points = np.column_stack((4*np.cos(a), 4*np.sin(a), np.zeros(4)))
    end = bridge.clearing_endpoints(points, 0)
    assert len(end) == 2
    assert np.degrees(np.arctan2(end[:, 1], end[:, 0])) == pytest.approx([-46, 44.5])


def test_real_obstacle_limits_clearing_even_with_farther_return():
    end = bridge.clearing_endpoints([[0.8, 0, 0], [8, 0, 0]], 0)
    assert end.shape == (1, 3)
    assert end[0, 0] <= 0.8


def test_blind_zone_return_alone_proves_nothing_outside_blind_zone():
    assert len(bridge.clearing_endpoints([[0.1, 0, 0]], 0)) == 0
    # A separate measured far ray, not the close return, supplies evidence.
    assert bridge.clearing_endpoints([[0.1, 0, 0], [8, 0, 0]], 0)[0, 0] == pytest.approx(8)


def test_ceiling_bounds_ray_at_slice_exit():
    end = bridge.clearing_endpoints([[4, 0, 4]], 0)
    assert end[0, 0] <= 0.8


def test_yaw_rotates_into_scan_frame():
    end = bridge.clearing_endpoints([[0, 4, 0]], np.pi/2)
    assert end[0] == pytest.approx([4, 0, 0], abs=1e-12)


def test_real_cloud_callback_preserves_obstacle_and_caps_floor_clearing():
    scans, clearing, obstacles = [], [], []
    node = SimpleNamespace(
        n_cloud=0, every=1, poses={0: (0, 0, 0, np.zeros(3))},
        R_lv=np.eye(3), pub_scan=SimpleNamespace(publish=scans.append),
        pub_clearing=SimpleNamespace(publish=clearing.append),
        pub_obstacles=SimpleNamespace(publish=obstacles.append))
    cloud = point_cloud2.create_cloud_xyz32(Header(), [[1, 0, -1], [0.8, 0, 0]])
    bridge.LevelBridge.on_cloud(node, cloud)
    assert scans[0].ranges[bridge.N_BINS//2] == pytest.approx(0.8)
    assert scans[0].header.frame_id == 'base_footprint'
    pts = point_cloud2.read_points_numpy(clearing[0], field_names=('x', 'y', 'z'))
    assert pts.shape == (1, 3)
    assert pts[0, 0] < 1
    assert clearing[0].header == scans[0].header
    marked = point_cloud2.read_points_numpy(obstacles[0], field_names=('x', 'y', 'z'))
    assert marked.shape == (1, 3)
    assert marked[0] == pytest.approx([.8, 0, 0])
    # Both actual rays stay on y=0; LaserScan angle float32 re-projection
    # otherwise puts marking into the adjacent costmap row at grid boundaries.
    assert marked[0, 1] == pts[0, 1] == 0
