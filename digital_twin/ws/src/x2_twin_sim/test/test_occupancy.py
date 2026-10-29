import numpy as np
import pytest
import yaml
from PIL import Image

from x2_twin_sim.occupancy import OccupancyMap, RayCaster


@pytest.fixture
def square_room(tmp_path):
    """10 m x 10 m room, 5 cm cells, walls 1 cell thick; origin (-5, -5); free inside."""
    n = 200
    img = np.full((n, n), 254, np.uint8)
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 0
    Image.fromarray(img).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(
        {'image': 'map.pgm', 'resolution': 0.05, 'origin': [-5.0, -5.0, 0.0], 'negate': 0,
         'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    return str(tmp_path / 'map.yaml')


def test_distance_field(square_room):
    m = OccupancyMap.from_yaml(square_room)
    d = m.dist_at(np.array([[0.0, 0.0], [-4.9, 0.0], [99.0, 0.0]]))
    assert d[0] == pytest.approx(5.0, abs=0.1)
    assert d[1] < 0.15
    assert np.isinf(d[2])


def test_raycast_hits_the_wall_at_the_right_range(square_room):
    rc = RayCaster(OccupancyMap.from_yaml(square_room), n_beams=360)
    r = rc.scan((0.0, 0.0, 0.0))
    # beam index 180 points along +x (angle_min = -pi): the +x wall is at x = 4.95 (inner face of the 1-cell wall)
    assert r[180] == pytest.approx(4.95, abs=0.08)
    assert r[90] == pytest.approx(5.0, abs=0.12)                       # -y wall


def test_raycast_rotates_with_the_sensor(square_room):
    rc = RayCaster(OccupancyMap.from_yaml(square_room), n_beams=360)
    a, b = rc.scan((1.0, 0.0, 0.0)), rc.scan((1.0, 0.0, np.pi / 2))
    assert a[180] == pytest.approx(3.95, abs=0.08)                      # facing +x from x=1
    assert b[180] == pytest.approx(5.0, abs=0.12)                       # facing +y from y=0


def test_forward_wedge_leaves_the_rest_unobserved(square_room):
    """Like the robot's scan: full-circle bins, but only the forward field of view returns; the rest is inf, never 'free'."""
    rc = RayCaster(OccupancyMap.from_yaml(square_room), n_beams=1440, range_min=0.45, fov_deg=103.0)
    r = rc.scan((0.0, 0.0, 0.0))
    angles = -np.pi + (2 * np.pi / 1440) * np.arange(1440)
    inside = np.abs(angles) <= np.radians(51.5) + 1e-9
    assert np.isfinite(r[inside]).all()
    assert np.isinf(r[~inside]).all()
    assert 0.40 * 1440 / 360 * 103 < inside.sum() < 1.05 * 1440 / 360 * 103       # ~412 bins
    assert r[720] == pytest.approx(4.95, abs=0.08)                                  # bin 720 looks straight ahead (+x)


def test_minimum_range_hides_nearby_walls(square_room):
    rc = RayCaster(OccupancyMap.from_yaml(square_room), n_beams=360, range_min=0.45)
    assert np.isinf(rc.scan((4.8, 0.0, 0.0))[180])                                  # the +x wall is 0.15 m away: below range_min
