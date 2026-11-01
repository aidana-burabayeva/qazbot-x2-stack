"""Doorway geometry: D1 map == the MuJoCo world generated from it, on the real map and robot configuration."""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/twin/operator'))
import doorway_check as D  # noqa: E402

pytestmark = pytest.mark.skipif(not Path(D.DEFAULT_ROBOT_DIR).exists(), reason='vendor robot configuration not present on this host')
MAP = str(ROOT / 'ws/src/maps/d1_qazbot_20261029/map.yaml')


@pytest.mark.parametrize('door,width,passable_disc', [((-6.485, -6.934), 0.805, False), ((-11.535, -4.484), 1.605, True)])
def test_world_geometry_matches_map_at_doorways(door, width, passable_disc, tmp_path):
    rep = D.check(MAP, [-8.344, -1.213, -2.145], door, work=tmp_path)
    assert rep['cells_mismatching_map_vs_mujoco'] == 0 and rep['geometry_consistent']
    assert abs(rep['door_width_map_m'] - width) < 0.03 and abs(rep['door_width_mujoco_m'] - width) < 0.03
    assert bool(rep['door_width_map_m'] > 2 * 0.454) is passable_disc          # production inflation radius vs the gap
    assert rep['passable_by_width'] is True                                 # but both are wider than the 0.712 m robot body
