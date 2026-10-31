"""Conservative planar SIM observations. Finite hits only; neither FOV coverage nor infinity proves free range.

This restores production message contracts, not E1R/3D cloud fidelity. The clearing
FOV is production level_bridge's explicit policy, not a new hardware-FOV estimate.
"""
import numpy as np


def bounded_observations(ranges, angles, observed, range_min, range_max, clearing_fov=(-46.0, 44.5)):
    r = np.asarray(ranges, dtype=float)
    a = np.asarray(angles, dtype=float)
    seen = np.asarray(observed, dtype=bool)
    if r.shape != a.shape or r.shape != seen.shape or r.ndim != 1:
        raise ValueError('ranges, angles and observed mask must have matching 1D shape')
    valid = seen & np.isfinite(r) & np.isfinite(a) & (r > range_min) & (r < range_max)
    mark = np.column_stack((r[valid]*np.cos(a[valid]), r[valid]*np.sin(a[valid]), np.zeros(valid.sum())))
    # Keep a float32 representable endpoint strictly before the finite return.
    # Inf, NaN, no-return, dropout and empty scans never become max-range clearing.
    valid &= (a >= np.radians(clearing_fov[0])) & (a <= np.radians(clearing_fov[1]))
    end = np.nextafter(r[valid].astype(np.float32), np.float32(0)).astype(float)
    clear = np.column_stack((end*np.cos(a[valid]), end*np.sin(a[valid]), np.zeros(valid.sum())))
    return mark, clear
