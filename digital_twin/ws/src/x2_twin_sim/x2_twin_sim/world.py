"""Physical walls for the official MuJoCo simulator, extruded from an occupancy map (no ROS imports).

Without walls the simulated robot walks through the map's obstacles (only the synthetic scan 'sees' them). This module writes a
copy of the official robot configuration whose ``scene.xml`` additionally holds one box per run of occupied cells, so the body
collides with exactly what the map says. Nothing in the vendor tree is edited: the copy symlinks every vendor entry except
``model_info/scene.xml``. Start the simulator with ``start_sim.sh -c <out>`` (never together with ``-s``: the vendor script then
clears the chosen configuration).

Rules: only OCCUPIED cells become walls (unknown space is not invented geometry, it is not solid); the pelvis spawns at sim
(0, 0) which corresponds to the map pose ``start`` (the transform is written to ``manifest.json``); the spawn must be free with a
clearance for the body envelope; the rectangle cover is verified to be exact.
"""
import argparse
import hashlib
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image

Rect = Tuple[int, int, int, int]            # x0, y0, x1, y1 in cells (half-open), y up
WALL_HEIGHT_M = 1.6                         # full height: the box is centred at WALL_HEIGHT_M / 2
MIN_SPAWN_CLEARANCE_M = 0.52                # body envelope radius about the control point + margin (X2 footprint study)


def rectangles(mask: np.ndarray) -> List[Rect]:
    """Disjoint cell rectangles covering ``mask`` exactly: identical horizontal runs on consecutive rows are merged vertically."""
    active, done = {}, []
    for y, row in enumerate(mask):
        padded = np.r_[False, row, False].astype(np.int8)
        edges = np.flatnonzero(np.diff(padded))
        current = {}
        for a, b in zip(edges[::2], edges[1::2]):
            key = (int(a), int(b))
            x0, y0, x1, _ = active.pop(key, (key[0], y, key[1], y))
            current[key] = (x0, y0, x1, y + 1)
        done.extend(active.values())
        active = current
    return done + list(active.values())


def load_grid(map_yaml: Path):
    """(blocked, free, resolution, origin) with row 0 = bottom (y up). Unknown cells are neither blocked nor free."""
    meta = yaml.safe_load(map_yaml.read_text())
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('only trinary maps are supported')
    img = np.asarray(Image.open(map_yaml.parent / meta['image']).convert('L'), float) / 255.0
    prob = img if meta.get('negate', 0) else 1.0 - img
    blocked = np.flipud(prob > float(meta['occupied_thresh']))
    free = np.flipud(prob < float(meta['free_thresh']))
    return blocked, free, float(meta['resolution']), [float(v) for v in meta['origin']], meta


def spawn_clearance(free: np.ndarray, ix: int, iy: int, res: float, window_m: float = 1.5) -> float:
    """Distance (m) from a cell to the nearest NON-free cell (occupied or unknown), searched within ``window_m``."""
    w = int(window_m / res)
    y0, y1, x0, x1 = max(iy - w, 0), min(iy + w + 1, free.shape[0]), max(ix - w, 0), min(ix + w + 1, free.shape[1])
    ys, xs = np.nonzero(~free[y0:y1, x0:x1])
    if len(ys) == 0:
        return window_m
    return float(min(np.hypot(ys + y0 - iy, xs + x0 - ix).min() * res, window_m))


def map_to_sim(map_xy: Sequence[float], start: Sequence[float]) -> Tuple[float, float]:
    """Map-frame point -> sim world frame (pelvis spawns at (0, 0) facing the start heading)."""
    sx, sy, heading = start
    c, s = math.cos(heading), math.sin(heading)
    return c * (map_xy[0] - sx) + s * (map_xy[1] - sy), -s * (map_xy[0] - sx) + c * (map_xy[1] - sy)


def build_world(map_yaml, start: Sequence[float], robot_dir, out_dir) -> dict:
    """Write ``out_dir/robot/<name>/...`` (symlinks + a modified scene.xml) and ``out_dir/manifest.json``; return the manifest."""
    map_yaml, robot_dir, out_dir = Path(map_yaml).resolve(), Path(robot_dir).resolve(), Path(out_dir).resolve()
    blocked, free, res, origin, meta = load_grid(map_yaml)
    ox, oy, angle = origin
    if res <= 0 or not np.isfinite([res, ox, oy, angle, *start]).all():
        raise ValueError('invalid map transform')
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = start[0] - ox, start[1] - oy
    ix, iy = int(math.floor((c * dx + s * dy) / res)), int(math.floor((-s * dx + c * dy) / res))
    if not (0 <= iy < free.shape[0] and 0 <= ix < free.shape[1]) or not free[iy, ix]:
        raise ValueError('spawn is not in free space')
    clearance = spawn_clearance(free, ix, iy, res)
    if clearance < MIN_SPAWN_CLEARANCE_M:
        raise ValueError(f'spawn clearance {clearance:.3f} m < {MIN_SPAWN_CLEARANCE_M} m')
    boxes = rectangles(blocked)
    cover = np.zeros(blocked.shape, np.uint8)
    for x0, y0, x1, y1 in boxes:
        cover[y0:y1, x0:x1] += 1
    if not np.array_equal(cover, blocked.astype(np.uint8)):
        raise RuntimeError('rectangle cover is not exact')

    dest = out_dir / 'robot' / robot_dir.name
    model = dest / 'model_info'
    model.mkdir(parents=True)
    for entry in robot_dir.iterdir():
        if entry.name != 'model_info':
            (dest / entry.name).symlink_to(os.path.relpath(entry, dest), target_is_directory=entry.is_dir())
    for entry in (robot_dir / 'model_info').iterdir():
        if entry.name != 'scene.xml':
            (model / entry.name).symlink_to(os.path.relpath(entry, model), target_is_directory=entry.is_dir())
    tree = ET.parse(robot_dir / 'model_info' / 'scene.xml')
    worldbody = tree.getroot().find('worldbody')
    half_h = WALL_HEIGHT_M / 2.0
    for i, (x0, y0, x1, y1) in enumerate(boxes):
        gx, gy = (x0 + x1) * res / 2.0, (y0 + y1) * res / 2.0                 # box centre in the map's own grid frame
        wx, wy = map_to_sim((ox + c * gx - s * gy, oy + s * gx + c * gy), start)
        ET.SubElement(worldbody, 'geom', name=f'map_wall_{i}', type='box', pos=f'{wx:.9f} {wy:.9f} {half_h}',
                      size=f'{(x1 - x0) * res / 2:.9f} {(y1 - y0) * res / 2:.9f} {half_h}',
                      euler=f'0 0 {angle - start[2]:.12f}', rgba='0.65 0.68 0.72 1', contype='1', conaffinity='1')
    tree.write(model / 'scene.xml', encoding='utf-8', xml_declaration=True)
    image = (map_yaml.parent / meta['image']).resolve()
    manifest = dict(
        map_yaml=str(map_yaml), image=str(image), image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
        yaml_sha256=hashlib.sha256(map_yaml.read_bytes()).hexdigest(), start_map=list(start), shape=list(blocked.shape),
        resolution=res, origin=origin, rectangles=len(boxes), blocked_cells=int(blocked.sum()), spawn_clearance_m=clearance,
        coverage_exact=True, unknown_is_solid=False, wall_height_m=WALL_HEIGHT_M, robot=str(robot_dir),
        transform='map_xy = R(start_yaw) @ sim_xy + start_xy; sim initial pelvis XY = (0, 0)')
    (out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', required=True, help='Nav2 map.yaml')
    ap.add_argument('--start', nargs=3, type=float, default=[-1.275, -4.825, 0.0], metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--robot', required=True, help='vendor configuration/robot/<name> directory')
    ap.add_argument('--out', required=True, help='new directory for the generated configuration (must not exist)')
    a = ap.parse_args()
    print(json.dumps(build_world(a.map, a.start, a.robot, a.out), indent=2))


if __name__ == '__main__':
    main()
