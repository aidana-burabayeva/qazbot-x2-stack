"""Scan semantics of the I1 candidate (finite = return, +inf = observed free, NaN = unknown); no ROS imports.

Verbatim copy of ``coverage_bins``/``build_ranges`` and their constants from candidates/i1_rpp_collision/level_bridge.py (the bridge
itself is not run in the twin: the plant produces the scan). test_i1_parity.py keeps the two copies identical.
"""
import math

import numpy as np

N_BINS = 1440

#: Observed field of view of the chest LiDAR in the level scan (bearing from base_footprint forward, measured on the physical X2:
#: first/last bin with returns in the recorded scans), the guard around a real return and the bridge of gaps between beams when a
#: bearing is judged "covered". Bins outside FOV_DEG are UNKNOWN (NaN).
FOV_DEG = (-46.0, 44.5)
INF_GUARD_BINS = 2
COVER_DILATE_BINS = 3


def coverage_bins(a_all, n_bins=N_BINS):
    """Bearings the sensor actually delivered ANY point for (floor, far wall, anything: before the height slice and the range limit).
    A bearing with no point at all - dropout, blocked or absorbing sector - is not observed free space."""
    covered = np.zeros(n_bins, dtype=bool)
    covered[((a_all + math.pi) / (2 * math.pi / n_bins)).astype(int) % n_bins] = True
    return covered


def build_ranges(r, a, covered=None, n_bins=N_BINS, fov_deg=FOV_DEG, guard_bins=INF_GUARD_BINS, cover_dilate_bins=COVER_DILATE_BINS):
    """Bin returns (range r, bearing a, radians) into a 360 deg scan with explicit semantics for the ObstacleLayer
    (obstacle_layer.scan.inf_is_valid: true):
      finite  - nearest return in the bin (marks, and raytrace-clears up to it);
      +inf    - OBSERVED free: inside the FOV, covered (the sensor delivered some point at this bearing, gaps of up to
                cover_dilate_bins bridged) and neither the bin nor its +-guard_bins neighbours has a return;
      NaN     - unknown: outside the FOV, uncovered (no point at all), or next to a hit. Nav2 ignores NaN: nothing there is
                marked or cleared.
    Without ``covered`` nothing can be proven free, so no +inf is produced (the legacy behaviour of never clearing).
    """
    ranges = np.full(n_bins, np.nan, dtype=np.float32)
    inc = 2 * math.pi / n_bins
    idx = ((a + math.pi) / inc).astype(int) % n_bins
    np.fmin.at(ranges, idx, r.astype(np.float32))      # fmin ignores the NaN fill
    hit = np.isfinite(ranges)
    near = hit.copy()
    for k in range(1, guard_bins + 1):
        near |= np.roll(hit, k) | np.roll(hit, -k)
    centre = -math.pi + (np.arange(n_bins) + 0.5) * inc
    in_fov = (centre >= math.radians(fov_deg[0])) & (centre <= math.radians(fov_deg[1]))
    if covered is not None:
        seen = covered.copy()
        for k in range(1, cover_dilate_bins + 1):
            seen |= np.roll(covered, k) | np.roll(covered, -k)
        ranges[in_fov & seen & ~near] = np.inf
    return ranges
