"""Fidelity tests of the ObstacleLayer emulation used by the D-04 replay."""
import math

import numpy as np
import pytest

import costmap_emul as ce


def ref_raytrace(size_x, x0, y0, x1, y1, max_length=2**32 - 1, min_length=0):
    """Scalar transcription of Costmap2D::raytraceLine/bresenham2D (Humble header)."""
    out = []
    dx_full, dy_full = x1 - x0, y1 - y0
    dist = math.hypot(dx_full, dy_full)
    if dist < min_length:
        return out
    if dist > 0.0:
        min_x0, min_y0 = int(x0 + dx_full / dist * min_length), int(y0 + dy_full / dist * min_length)
    else:
        min_x0, min_y0 = x0, y0
    offset = min_y0 * size_x + min_x0
    dx, dy = x1 - min_x0, y1 - min_y0
    abs_dx, abs_dy = abs(dx), abs(dy)
    sgn = lambda v: 1 if v > 0 else -1
    offset_dx, offset_dy = sgn(dx), sgn(dy) * size_x
    scale = 1.0 if dist == 0.0 else min(1.0, max_length / dist)

    def bres(abs_da, abs_db, error_b, offset_a, offset_b, offset, max_len):
        end = min(max_len, abs_da)
        for _ in range(end):
            out.append(offset)
            offset += offset_a
            error_b += abs_db
            if error_b >= abs_da:
                offset += offset_b
                error_b -= abs_da
        out.append(offset)
    if abs_dx >= abs_dy:
        bres(abs_dx, abs_dy, abs_dx // 2, offset_dx, offset_dy, offset, int(scale * abs_dx))
    else:
        bres(abs_dy, abs_dx, abs_dy // 2, offset_dy, offset_dx, offset, int(scale * abs_dy))
    return out


@pytest.mark.parametrize('max_len,min_len', [(None, 0), (70, 0), (13, 0), (70, 5)])
def test_vectorised_bresenham_matches_header_transcription(max_len, min_len):
    rng = np.random.default_rng(1)
    n = 80
    x0, y0 = 40, 40
    x1 = rng.integers(0, n, 400)
    y1 = rng.integers(0, n, 400)
    for a, b in zip(x1, y1):
        cx, cy = ce.bresenham_cells(x0, y0, np.array([a]), np.array([b]),
                                    max_len, min_len)
        got = sorted((cy * n + cx).tolist())
        ref = sorted(ref_raytrace(n, x0, y0, int(a), int(b),
                                  2**32 - 1 if max_len is None else max_len, min_len))
        assert got == ref, (a, b)


def _layer():
    L = ce.ObstacleLayer(4.0, 0.05, 2.0, None)
    L.update_origin(0.0, 0.0)
    return L


def test_clear_then_mark_keeps_current_hit():
    L = _layer()
    p = np.array([[1.0, 0.0, 0.0]])
    obs = ce.Observation(p, (0, 0, 0), 3.0, 0.0, 3.5, 0.0, 0.0, 0.0)
    L.update((0, 0, 0), [obs], [obs])
    mx, my, _ = L.world_to_map(1.0, 0.0)
    assert L.g[my, mx] == ce.LETHAL


def test_clearing_ray_removes_stale_mark_and_endpoint_cell():
    L = _layer()
    mx, my, _ = L.world_to_map(0.8, 0.0)
    L.g[my, mx] = ce.LETHAL
    clr = ce.Observation(np.array([[0.8, 0.0, 0.0]]), (0, 0, 0), 2.5, 0.0, 3.5, 0.0, 0.0, 0.0)
    L.update((0, 0, 0), [], [clr])
    assert L.g[my, mx] == ce.FREE                      # bresenham acts on the end cell too


def test_unobserved_cells_untouched():
    L = _layer()
    seeds = [(-1.0, 0.0), (0.0, 1.2), (0.7, -0.9)]
    for x, y in seeds:
        mx, my, _ = L.world_to_map(x, y)
        L.g[my, mx] = ce.LETHAL
    clr = ce.Observation(np.array([[1.5, 0.3, 0.0]]), (0, 0, 0), 2.5, 0.0, 3.5, 0.0, 0.0, 0.0)
    L.update((0, 0, 0), [], [clr])
    for x, y in seeds:
        mx, my, _ = L.world_to_map(x, y)
        assert L.g[my, mx] == ce.LETHAL


def test_height_filter_zero_zero_keeps_only_exact_zero():
    o = ce.Observation(np.array([[1, 0, 0.0], [1, 0, 1e-6], [1, 0, -1e-6]]), (0, 0, 0))
    assert len(o.points) == 1


def test_marking_uses_3d_distance_and_obstacle_range():
    L = _layer()
    p = np.array([[1.9, 0.0, 0.0], [0.5, 0.0, 2.99]])     # second: 3D dist 3.03 > 3.0
    o = ce.Observation(p, (0, 0, 0), 3.0, 0.0, 3.5, 0.0, -5.0, 5.0)
    L.max_obstacle_height = 5.0
    L.mark(o)
    assert (L.g == ce.LETHAL).sum() == 1


def test_raytrace_max_range_limits_clearing():
    L = _layer()
    for x in (1.0, 1.9):
        mx, my, _ = L.world_to_map(x, 0.0)
        L.g[my, mx] = ce.LETHAL
    clr = ce.Observation(np.array([[1.95, 0.0, 0.0]]), (0, 0, 0), 2.5, 0.0, 1.5, 0.0, 0.0, 0.0)
    L.raytrace_freespace(clr)
    m1 = L.world_to_map(1.0, 0.0)
    m2 = L.world_to_map(1.9, 0.0)
    assert L.g[m1[1], m1[0]] == ce.FREE and L.g[m2[1], m2[0]] == ce.LETHAL


def test_footprint_clearing_and_rolling_origin():
    fp = [(0.17, 0.229), (0.008, 0.34), (-0.182, 0.34), (-0.305, 0.229),
          (-0.305, -0.261), (-0.182, -0.372), (0.008, -0.372), (0.17, -0.261)]
    L = ce.ObstacleLayer(4.0, 0.05, 2.0, fp, 0.01)
    L.update_origin(0, 0)
    L.g[:] = ce.LETHAL
    L.update((0.0, 0.0, 0.0))
    mx, my, _ = L.world_to_map(0.0, 0.0)
    assert L.g[my, mx] == ce.FREE
    mx, my, _ = L.world_to_map(0.5, 0.0)
    assert L.g[my, mx] == ce.LETHAL
    L.update_origin(0.12, 0.0)                         # 2 whole cells (int truncation)
    assert L.ox == pytest.approx(-2.0 + 0.10)


def test_rpp_check_flags_mark_in_front_right_like_d04():
    fp = ce.pad_footprint([(0.17, 0.229), (0.008, 0.34), (-0.182, 0.34), (-0.305, 0.229),
                           (-0.305, -0.261), (-0.182, -0.372), (0.008, -0.372), (0.17, -0.261)], 0.01)
    L = _layer()
    a = math.radians(-50)
    mx, my, _ = L.world_to_map(0.45 * math.cos(a), 0.45 * math.sin(a))
    L.g[my, mx] = ce.LETHAL
    assert ce.rpp_collision_imminent(L, fp, (0, 0, 0), 0.3, 0.0)
    L.g[:] = 0
    assert not ce.rpp_collision_imminent(L, fp, (0, 0, 0), 0.3, 0.0)
