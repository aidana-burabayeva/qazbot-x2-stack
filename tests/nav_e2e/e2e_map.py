"""Map and fixture poses shared by the nav_e2e suites (real chain, legacy) and their model fixtures.

Map: ws/src/maps/first_map_nav2_v2 (derived from the D1 first_map saved 2026-10-21T18:07:30+05:00 by
tools/derive_nav2_map.py; see its manifest.json). EXPECTED_PGM_SHA256 is read from that manifest.

START is an explicit FIXTURE pose, not a physical home pose (the new D1 map has no home.yaml). It was selected
offline on the v2 grid (report 20261021-x2-nav2-e2e-dryrun-impl2b, section 4):
- every cell of the +x corridor x in [START_X - 0.1, START_X + 1.8] at y = START_Y is free (254) with a distance
  to the nearest non-free (occupied or unknown) cell >= 0.65 m (old map corridor: 0.61 m); at START 0.92 m,
  > robot_radius 0.454 + inflation, so rotation in place is collision free in the costmap;
- the fixture's 360-beam D1 ray-cast scan at START is not ambiguous for the trust node (L.ambiguity: nominal 1.0,
  best competitor beyond 0.5 m 0.34);
- the box obstacle of sim_robot/loc_model_fixture is the line x = wall_x (wall_x < 0 disables it), so the
  obstacle at START_X + 1.3 = 0.025 must stay at x >= 0: this keeps the x offsets of the old suite unchanged.
Out-of-map goals are just outside the grid bounds; OCCUPIED is the centre of a 3x3 block of occupied (0) pixels.
"""
import json
from pathlib import Path

import yaml

MAP_DIR = Path(__file__).resolve().parents[2] / 'ws/src/maps/first_map_nav2_v2'
MAP_YAML = MAP_DIR / 'map.yaml'
EXPECTED_PGM_SHA256 = json.loads((MAP_DIR / 'manifest.json').read_text())['pgm_sha256']

START = (-1.275, -4.825, 0.0)
X0, Y0 = START[0], START[1]


def along(dx):
    """Goal on the START corridor, dx metres ahead along +x."""
    return (round(X0 + dx, 6), Y0, 0.0)


STRAIGHT = along(1.5)          # straight 1.5 m
REPEAT = along(0.9)
FAR = along(1.7)               # cancel / obstacle / faults / takeover goal A
NEAR = along(0.5)              # goal B after takeover / recovery, RViz goal
WALL_X = round(X0 + 1.3, 6)    # box obstacle 1.3 m ahead (local costmap only, not in the map)
OCCUPIED = (-1.225, -3.575)    # occupied 3x3 block, 1.25 m from START


def _bounds():
    y = yaml.safe_load(MAP_YAML.read_text())
    import cv2
    h, w = cv2.imread(str(MAP_DIR / y['image']), 0).shape
    ox, oy, r = float(y['origin'][0]), float(y['origin'][1]), float(y['resolution'])
    return ox, oy, ox + w * r, oy + h * r


_OX, _OY, _XMAX, _YMAX = _bounds()
EPS = 1e-6                     # strictly outside the grid (cells cover [origin, origin + size * res))
OUT_OF_MAP = [('left', _OX - EPS, Y0), ('right', _XMAX + EPS, Y0), ('bottom', 0., _OY - EPS), ('top', 0., _YMAX + EPS)]
