"""RayCaster transient discs: real obstacles that are not in the map (appear/disappear at run time)."""
import math

import numpy as np

from x2_twin_sim.occupancy import OccupancyMap, RayCaster


def caster(fov=103.0, rmax=8.0):
    return RayCaster(OccupancyMap(np.zeros((200, 200), bool), 0.05, (-5.0, -5.0)), n_beams=1440, range_min=0.45, range_max=rmax,
                     fov_deg=fov)


def test_empty_world_has_no_returns_and_disc_returns_its_near_surface():
    rc = caster()
    assert np.all(np.isinf(rc.scan((0.0, 0.0, 0.0))))
    s = rc.scan((0.0, 0.0, 0.0), [(1.0, 0.0, 0.2)])
    ahead = int(np.argmin(np.abs(rc.angle_min + rc.angle_increment * np.arange(rc.n))))
    assert s[ahead] == np.float64(s[ahead]) and abs(s[ahead] - 0.8) < 1e-9
    assert np.isfinite(s).sum() > 5
    assert np.isinf(s[~np.isfinite(s)]).all()


def test_disc_behind_the_robot_or_outside_fov_or_beyond_range_is_not_seen():
    rc = caster(fov=103.0)
    assert np.all(np.isinf(rc.scan((0.0, 0.0, 0.0), [(-1.0, 0.0, 0.2)])))              # behind
    assert np.all(np.isinf(rc.scan((0.0, 0.0, 0.0), [(0.0, 1.0, 0.2)])))               # 90 deg left, outside the 103 deg sector
    assert np.all(np.isinf(caster(rmax=2.0).scan((0.0, 0.0, 0.0), [(3.0, 0.0, 0.2)])))  # beyond the range


def test_disc_follows_the_robot_heading():
    rc = caster()
    s = rc.scan((0.0, 0.0, math.pi / 2), [(0.0, 1.0, 0.2)])                            # robot faces +y, disc at +y
    assert np.isfinite(s).any()
