#!/usr/bin/env python3
"""Doorway geometry check: D1 map  <->  the physical MuJoCo world the official simulator will load.

Builds the MuJoCo world with the production generator (x2_twin_sim.world.build_world), reads the wall boxes back from the generated
scene.xml, transforms them to the map frame with the INVERSE of the documented transform and compares, independently of the
generator's own cover test:
  1. the occupancy implied by the boxes equals the map's occupied cells in a window around the doorway (cell-exact);
  2. the free gap through the doorway centre (minimum over directions) measured in the map and in the MuJoCo geometry agree to
     one map cell, and is compared with the robot width (X2 envelope from the footprint study, read-only constant).
No simulation is started. Exit 0 only if both hold.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'digital_twin/ws/src/x2_twin_sim'))
from x2_twin_sim import world  # noqa: E402

ROBOT_WIDTH = 0.712     # m, docs/nightly/map/main_office_analysis.json robot.width_m (footprint study)
DEFAULT_ROBOT_DIR = '/home/adn/QazBot/digital_twin/vendor/sim/configuration/robot/lx2501_3_t2d5'


def boxes_in_map(scene_xml, start):
    """[(cx, cy, hx, hy, theta)] of every map_wall_* geom, expressed in the MAP frame (box axes rotated by theta)."""
    c, s = math.cos(start[2]), math.sin(start[2])
    out = []
    for g in ET.parse(scene_xml).getroot().find('worldbody').iter('geom'):
        if not g.get('name', '').startswith('map_wall_'):
            continue
        px, py, _ = (float(v) for v in g.get('pos').split())
        hx, hy, _ = (float(v) for v in g.get('size').split())
        euler_z = float(g.get('euler').split()[2])
        out.append((c * px - s * py + start[0], s * px + c * py + start[1], hx, hy, euler_z + start[2]))
    return out


def raster_boxes(boxes, xs, ys):
    """Boolean raster of points (meshgrid xs, ys in map frame) inside any oriented box."""
    X, Y = np.meshgrid(xs, ys)
    hit = np.zeros(X.shape, bool)
    for cx, cy, hx, hy, th in boxes:
        if cx + hx + hy < xs[0] or cx - hx - hy > xs[-1] or cy + hx + hy < ys[0] or cy - hx - hy > ys[-1]:
            continue
        c, s = math.cos(th), math.sin(th)
        u = c * (X - cx) + s * (Y - cy)
        v = -s * (X - cx) + c * (Y - cy)
        hit |= (np.abs(u) <= hx) & (np.abs(v) <= hy)
    return hit


def min_gap(occ, xs, ys, cx, cy, step_deg=5):
    """Smallest free run through (cx, cy) over directions: the door width. occ is True where solid."""
    res = xs[1] - xs[0]
    best = (float('inf'), None)
    ix, iy = int(round((cx - xs[0]) / res)), int(round((cy - ys[0]) / res))
    if occ[iy, ix]:
        return best
    for deg in range(0, 180, step_deg):
        a = math.radians(deg)
        run = 0.0
        for sign in (1, -1):
            d = 0.0
            while d < 3.0:
                px, py = ix + sign * math.cos(a) * d / res, iy + sign * math.sin(a) * d / res
                jx, jy = int(round(px)), int(round(py))
                if not (0 <= jx < occ.shape[1] and 0 <= jy < occ.shape[0]) or occ[jy, jx]:
                    break
                d += res / 2
            run += d
        if run < best[0]:
            best = (run, deg)
    return best


def check(map_yaml, start, door, robot_dir=DEFAULT_ROBOT_DIR, window=2.5, work=None):
    work = Path(work or tempfile.mkdtemp(prefix='doorway_'))
    manifest = world.build_world(map_yaml, start, robot_dir, work / 'world')
    scene = next((work / 'world/robot').glob('*/model_info/scene.xml'))
    boxes = boxes_in_map(scene, start)
    blocked, free, res, origin, _ = world.load_grid(Path(map_yaml))
    ox, oy, _ang = origin
    # raster window in the map frame at 0.01 m, and the map's own occupancy sampled at the same points (grid row 0 = bottom, y up)
    f = 5
    xs = np.arange(door[0] - window, door[0] + window, res / f)
    ys = np.arange(door[1] - window, door[1] + window, res / f)
    world_occ = raster_boxes(boxes, xs, ys)
    jx = np.clip(np.floor((xs - ox) / res).astype(int), 0, blocked.shape[1] - 1)
    jy = np.clip(np.floor((ys - oy) / res).astype(int), 0, blocked.shape[0] - 1)
    map_occ = blocked[np.ix_(jy, jx)]
    mismatch = int(np.sum(world_occ != map_occ))
    wmap = min_gap(map_occ, xs, ys, door[0], door[1])
    wworld = min_gap(world_occ, xs, ys, door[0], door[1])
    rep = dict(map=str(map_yaml), start_map=list(start), door_center_map=list(door), world_manifest=dict(rectangles=manifest['rectangles'],
               blocked_cells=manifest['blocked_cells'], coverage_exact=manifest['coverage_exact'], image_sha256=manifest['image_sha256']),
               window_m=window, raster_points=int(world_occ.size), cells_mismatching_map_vs_mujoco=mismatch,
               door_width_map_m=round(wmap[0], 3), door_width_mujoco_m=round(wworld[0], 3), door_axis_deg_map=wmap[1],
               robot_width_m=ROBOT_WIDTH, side_margin_m=round((wmap[0] - ROBOT_WIDTH) / 2, 3),
               geometry_consistent=bool(mismatch == 0 and abs(wmap[0] - wworld[0]) <= res),
               passable_by_width=bool(wmap[0] > ROBOT_WIDTH),
               note='unknown map cells are NOT solid in MuJoCo (world.py) but are obstacles for the synthetic scan; check the door neighbourhood for unknown cells')
    unk = int(np.sum(~blocked[np.ix_(jy, jx)] & ~free[np.ix_(jy, jx)]))
    rep['unknown_cells_in_window_samples'] = int(unk)
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-yaml', default=str(ROOT / 'ws/src/maps/d1_qazbot_20261029/map.yaml'))
    ap.add_argument('--start', default='-8.344,-1.213,-2.145')
    ap.add_argument('--door', required=True, help='X,Y of the doorway centre in the map frame')
    ap.add_argument('--robot-dir', default=DEFAULT_ROBOT_DIR)
    ap.add_argument('--window', type=float, default=2.5)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rep = check(a.map_yaml, [float(v) for v in a.start.split(',')], [float(v) for v in a.door.split(',')], a.robot_dir, a.window)
    text = json.dumps(rep, indent=1)
    print(text)
    if a.out:
        Path(a.out).write_text(text)
    return 0 if rep['geometry_consistent'] else 1


if __name__ == '__main__':
    sys.exit(main())
