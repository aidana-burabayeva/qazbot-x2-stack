"""Unit tests of the I1 candidate: scan semantics of level_bridge.build_ranges (finite = return, +inf = observed free, NaN = unknown).

+inf requires COVERAGE: the sensor delivered some point at that bearing (floor, far wall, anything). A missing return alone proves
nothing. Run inside the dev container:
  make test-i1
"""
import importlib.util
import math

import numpy as np
import pytest

pytest.importorskip('rclpy')
spec = importlib.util.spec_from_file_location('level_bridge_i1', __file__.replace('test_level_scan.py', 'level_bridge.py'))
lb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lb)

N = lb.N_BINS
INC = 2 * math.pi / N
CENTRE = -math.pi + (np.arange(N) + 0.5) * INC
FOV = (CENTRE >= math.radians(lb.FOV_DEG[0])) & (CENTRE <= math.radians(lb.FOV_DEG[1]))


def wall(bearing_deg, half_width_deg, r, step_deg=0.2):
    a = np.radians(np.arange(bearing_deg - half_width_deg, bearing_deg + half_width_deg, step_deg))
    return np.full(a.shape, r), a


def scene(*parts):
    r, a = zip(*parts)
    return np.concatenate(r), np.concatenate(a)


def full_coverage():
    """A healthy sensor delivers points (at least floor returns) over its whole sector."""
    return lb.coverage_bins(np.radians(np.arange(-60.0, 60.0, 0.1)), N)


def test_coverage_marks_the_bearings_with_points():
    c = lb.coverage_bins(np.radians([-10.0, 0.0, 10.0]), N)
    assert c.sum() == 3 and c[int((math.radians(10.0) + math.pi) / INC)]


def test_free_route_in_fov_is_observed_free_and_outside_is_unknown():
    r, a = wall(-0.75, 45.25, 5.0)                             # a wall exactly across the FOV (-46..+44.5 deg), 5 m away
    s = lb.build_ranges(r, a, full_coverage())
    assert np.all(np.isfinite(s[FOV][10:-10])), 'a dense wall leaves no bin in the FOV empty'
    assert np.all(np.isnan(s[~FOV])), 'outside the FOV nothing is observed: NaN, never +inf'
    assert not np.any(np.isinf(s[~FOV]))


def test_gap_in_returns_is_free_only_when_covered_and_not_next_to_a_hit():
    r, a = scene(wall(-30.0, 14.0, 6.0), wall(+30.0, 14.0, 6.0))           # two wall segments, a 32 deg opening between them
    opening = FOV & (CENTRE > math.radians(-15)) & (CENTRE < math.radians(15))
    s = lb.build_ranges(r, a, full_coverage())
    assert np.all(np.isinf(s[opening][4:-4])), 'the opening is covered and empty: observed free'
    assert not np.any(np.isinf(lb.build_ranges(r, a, None))), 'without coverage nothing can be proven free'
    hit = np.flatnonzero(np.isfinite(s))
    for i in hit:
        for k in (1, 2):
            for j in ((i + k) % N, (i - k) % N):
                assert np.isfinite(s[j]) or np.isnan(s[j]), 'the bins next to a return are unknown, not free'


def test_real_obstacle_stays_a_finite_return():
    box = wall(-4.0, 4.0, 0.7)
    r, a = scene(wall(0.0, 46.0, 7.0), box)
    s = lb.build_ranges(r, a, full_coverage())
    idx = np.unique(((box[1] + math.pi) / INC).astype(int))
    assert np.all(s[idx] == pytest.approx(0.7, abs=1e-3)), 'every bin that sees the obstacle keeps its 0.7 m return'


def test_sparse_obstacle_with_sampling_gaps_is_not_cleared_by_its_neighbours():
    a = np.radians(np.array([-4.0, -2.5, -1.0, 0.5]))           # a real obstacle seen by few beams (1.5 deg between them)
    r, aa = scene(wall(0.0, 46.0, 7.0), (np.full(a.shape, 0.8), a))
    s = lb.build_ranges(r, aa, full_coverage())
    idx = ((a + math.pi) / INC).astype(int)
    assert np.all(np.isfinite(s[idx]))
    gap = np.arange(idx[0] + 1, idx[1])
    assert not np.any(np.isinf(s[gap][:2])), 'the guard keeps the bins beside a hit from reading as free'


def test_dropout_clears_nothing_whatever_the_number_of_returns():
    """No point at all (dead lidar) - or returns but no coverage - never gives +inf; there is no count above which it does."""
    nothing = lb.coverage_bins(np.array([]), N)
    for n_hits in (0, 3, 19, 20, 21, 100, 400):
        a = np.radians(np.linspace(-40.0, 40.0, max(n_hits, 1)))[:n_hits]
        s = lb.build_ranges(np.full(a.shape, 2.0), a, nothing)
        assert not np.any(np.isinf(s)), f'{n_hits} returns without coverage must not read as free'


def test_free_space_with_very_few_returns_is_still_observed_free_when_covered():
    """The opposite case: a healthy sensor in an open hall delivers floor points everywhere and almost no slice returns. 19, 20 and
    21 returns behave identically (no cliff at a count): the free bearings are cleared in all three."""
    results = []
    for n_hits in (0, 1, 19, 20, 21, 22):
        a = np.radians(np.linspace(-44.0, 44.0, n_hits)) if n_hits else np.array([])
        s = lb.build_ranges(np.full(a.shape, 7.5), a, full_coverage())
        near = np.zeros(N, bool)
        h = np.isfinite(s)
        near |= h
        for k in (1, 2):
            near |= np.roll(h, k) | np.roll(h, -k)
        free = FOV & ~near
        assert np.all(np.isinf(s[free])), f'{n_hits} returns: every covered, hit-free FOV bin must be observed free'
        results.append(int(np.isinf(s).sum()))
    assert results == sorted(results, reverse=True), 'more returns can only remove +inf bins, never add them'


def test_half_blocked_sensor_only_clears_what_it_saw():
    """A blocked sector delivers no points: the marks there must survive although plenty of other returns exist."""
    seen = lb.coverage_bins(np.radians(np.arange(-60.0, 0.0, 0.1)), N)          # only the right half delivers points
    r, a = wall(-22.0, 22.0, 6.0)
    s = lb.build_ranges(r, a, seen)
    left = FOV & (CENTRE > math.radians(2.0))
    assert not np.any(np.isinf(s[left])), 'the blocked left half is unknown'
    assert not np.any(np.isinf(s[~FOV]))


def test_unobserved_sector_never_cleared_even_with_an_obstacle_return_there():
    r, a = scene(wall(0.0, 46.0, 7.0), wall(-60.0, 3.0, 0.6))   # an obstacle just beyond the right edge of the FOV
    s = lb.build_ranges(r, a, full_coverage())
    beyond = ~FOV & (CENTRE < math.radians(-50)) & (CENTRE > math.radians(-70))
    assert np.any(np.isfinite(s[beyond]))                       # a return there is kept (it marks)
    assert not np.any(np.isinf(s[~FOV])), 'bins outside the observed FOV are never +inf'


def test_property_inf_only_where_covered_in_fov_and_hit_free():
    rng = np.random.default_rng(7)
    for _ in range(200):
        a = np.radians(rng.uniform(-90, 90, rng.integers(0, 400)))
        r = rng.uniform(0.5, 8.0, a.shape)
        cov = lb.coverage_bins(np.radians(rng.uniform(-90, 90, rng.integers(0, 600))), N)
        s = lb.build_ranges(r, a, cov)
        inf = np.isposinf(s)
        seen = cov.copy()
        for k in range(1, lb.COVER_DILATE_BINS + 1):
            seen |= np.roll(cov, k) | np.roll(cov, -k)
        assert not np.any(inf & ~FOV) and not np.any(inf & ~seen) and not np.any(inf & np.isfinite(s))
