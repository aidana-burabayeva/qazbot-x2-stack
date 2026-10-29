import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from PIL import Image

from x2_twin_sim import world


def test_rectangles_cover_exactly_and_do_not_overlap():
    rng = np.random.default_rng(7)
    for _ in range(200):
        mask = rng.random((int(rng.integers(1, 25)), int(rng.integers(1, 30)))) > 0.55
        cover = np.zeros(mask.shape, int)
        for x0, y0, x1, y1 in world.rectangles(mask):
            assert x0 < x1 and y0 < y1
            cover[y0:y1, x0:x1] += 1
        assert np.array_equal(cover, mask.astype(int))                 # exact, and no cell counted twice


def test_rectangles_merge_identical_runs_vertically():
    mask = np.zeros((5, 6), bool)
    mask[1:4, 2:5] = True
    assert world.rectangles(mask) == [(2, 1, 5, 4)]


def test_map_to_sim_follows_the_start_heading():
    assert world.map_to_sim((1.0, 3.0), (1.0, 2.0, math.pi / 2)) == pytest.approx((1.0, 0.0), abs=1e-12)
    assert world.map_to_sim((-1.275, -4.825), (-1.275, -4.825, 0.0)) == pytest.approx((0.0, 0.0))


@pytest.fixture
def room(tmp_path):
    """10 m square room, 5 cm cells, walls 2 cells thick, one unknown patch; origin (-5, -5)."""
    n = 200
    img = np.full((n, n), 254, np.uint8)
    img[:2, :] = img[-2:, :] = img[:, :2] = img[:, -2:] = 0
    img[90:110, 150:170] = 205                                            # unknown patch (grey)
    Image.fromarray(img).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(
        {'image': 'map.pgm', 'resolution': 0.05, 'origin': [-5.0, -5.0, 0.0], 'negate': 0, 'mode': 'trinary',
         'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    robot = tmp_path / 'vendor_robot' / 'lx2501_3_t2d5'
    (robot / 'model_info').mkdir(parents=True)
    (robot / 'simulator').mkdir()
    (robot / 'simulator' / 'default.yaml').write_text('x: 1\n')
    (robot / 'model_info' / 'x2.xml').write_text('<mujoco/>')
    scene = '<mujoco><worldbody><geom name="floor" type="plane" size="1 1 1"/></worldbody></mujoco>'
    (robot / 'model_info' / 'scene.xml').write_text(scene)
    return tmp_path


def test_build_world_writes_walls_and_never_touches_the_vendor_tree(room):
    robot = room / 'vendor_robot' / 'lx2501_3_t2d5'
    before = (robot / 'model_info' / 'scene.xml').read_text()
    manifest = world.build_world(room / 'map.yaml', (0.0, 0.0, 0.0), robot, room / 'out')
    assert (robot / 'model_info' / 'scene.xml').read_text() == before                # vendor file untouched
    out = room / 'out' / 'robot' / 'lx2501_3_t2d5'
    assert (out / 'simulator').is_symlink() and (out / 'simulator' / 'default.yaml').read_text() == 'x: 1\n'   # linked, readable
    assert not (out / 'model_info' / 'scene.xml').is_symlink()
    geoms = ET.parse(out / 'model_info' / 'scene.xml').getroot().find('worldbody').findall('geom')
    walls = [g for g in geoms if g.get('name', '').startswith('map_wall_')]
    assert len(walls) == manifest['rectangles'] and manifest['coverage_exact'] and not manifest['unknown_is_solid']
    assert any(g.get('name') == 'floor' for g in geoms)                               # the original scene is kept
    assert json.loads((room / 'out' / 'manifest.json').read_text())['blocked_cells'] == manifest['blocked_cells']


def test_wall_boxes_sit_where_the_map_says(room):
    """The +x wall's inner face is at map x = 4.9 (cells 198..199 -> x in [4.9, 5.0]); the sim frame is the map shifted by START."""
    robot = room / 'vendor_robot' / 'lx2501_3_t2d5'
    world.build_world(room / 'map.yaml', (1.0, 0.0, 0.0), robot, room / 'out')
    geoms = ET.parse(room / 'out' / 'robot' / 'lx2501_3_t2d5' / 'model_info' / 'scene.xml').getroot().find('worldbody').findall('geom')
    inner_faces = []
    for g in geoms:
        if g.get('name', '').startswith('map_wall_'):
            px, py, _ = (float(v) for v in g.get('pos').split())
            hx, hy, _ = (float(v) for v in g.get('size').split())
            if px > 0 and hy > 4.0:                                                  # the long wall on the +x side
                inner_faces.append(px - hx)
    assert inner_faces and min(inner_faces) == pytest.approx(4.9 - 1.0, abs=1e-6)     # 4.9 m in the map = 3.9 m ahead of START


def test_spawn_must_be_free_with_clearance(room):
    robot = room / 'vendor_robot' / 'lx2501_3_t2d5'
    with pytest.raises(ValueError, match='clearance'):
        world.build_world(room / 'map.yaml', (4.5, 0.0, 0.0), robot, room / 'near_wall')     # 0.4 m from the +x wall
    with pytest.raises(ValueError, match='free space'):
        world.build_world(room / 'map.yaml', (4.95, 0.0, 0.0), robot, room / 'in_wall')
    with pytest.raises(ValueError, match='free space'):
        world.build_world(room / 'map.yaml', (3.0, 0.0, 0.0), robot, room / 'unknown')       # inside the unknown patch (grey)
