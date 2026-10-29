#!/usr/bin/env python3
"""Phase B: static analysis of the main_office occupancy map (offline, no ROS).

unknown is NEVER treated as free: both unknown and occupied are obstacles for clearance and connectivity.
Door candidates are geometric bottlenecks between watershed regions and stay UNVALIDATED until checked in the
simulator world / on the real building. Coordinates are measured from the map, not invented.
"""
import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage as ndi
from skimage.segmentation import watershed

ROOT = Path(__file__).resolve().parents[2]
MAP = Path(os.environ.get('MAP_DIR', ROOT / 'ws/src/maps/main_office'))
OUT = Path(os.environ.get('MAP_OUT', ROOT / 'docs/nightly/map'))
# 8-point X2 footprint in base_footprint (nav_control.launch.py): width (y) and length (x), circumscribed radius
FP = np.array([[0.17, 0.229], [0.008, 0.34], [-0.182, 0.34], [-0.305, 0.229], [-0.305, -0.261], [-0.182, -0.372], [0.008, -0.372], [0.17, -0.261]])
ROBOT_W = float(FP[:, 1].max() - FP[:, 1].min())
ROBOT_R = float(np.hypot(FP[:, 0], FP[:, 1]).max())
CORE_M = float(os.environ.get('CORE_M', 1.0))


def main():
    man = json.loads((MAP / 'manifest.json').read_text())
    pgm = MAP / 'map.pgm'
    sha = hashlib.sha256(pgm.read_bytes()).hexdigest()
    yml = dict(ln.split(':', 1) for ln in (MAP / 'map.yaml').read_text().splitlines() if ':' in ln and not ln.startswith(' '))
    res = float(yml['resolution']); occ_t = float(yml['occupied_thresh']); free_t = float(yml['free_thresh'])
    lines = (MAP / 'map.yaml').read_text().splitlines()
    i = lines.index('origin:')
    origin = [float(lines[i + k].strip()[2:]) for k in (1, 2)]
    img = np.array(Image.open(pgm).convert('L'))
    p = (255 - img.astype(float)) / 255.0  # negate: 0
    free = p < free_t
    occ = p > occ_t
    unknown = ~free & ~occ
    # row 0 of the image is the top = max y
    h, w = img.shape

    def to_world(r, c):
        return (origin[0] + (c + 0.5) * res, origin[1] + (h - r - 0.5) * res)

    lab, n = ndi.label(free, structure=np.ones((3, 3)))
    sizes = ndi.sum(free, lab, range(1, n + 1))
    main_id = int(np.argmax(sizes)) + 1
    mainfree = lab == main_id
    edt = ndi.distance_transform_edt(mainfree) * res  # distance to nearest non-free (occupied OR unknown)
    # room cores: free space deeper than CORE_M from any obstacle/unknown
    core, nc = ndi.label(edt > CORE_M)
    cs = ndi.sum(np.ones_like(core), core, range(1, nc + 1))
    keep = [i + 1 for i, s in enumerate(cs) if s * res * res > 1.5]
    markers = np.zeros_like(core)
    for k, i in enumerate(keep, 1):
        markers[core == i] = k
    ws = watershed(-edt, markers, mask=mainfree)
    # bottlenecks: boundary pixels between two regions, one candidate per connected boundary cluster (minimum EDT)
    cands = []
    pair = {}
    for dr, dc in ((0, 1), (1, 0)):
        a = ws[:h - dr, :w - dc]; b = ws[dr:, dc:]
        m = (a != b) & (a > 0) & (b > 0)
        for r, c in zip(*np.nonzero(m)):
            key = tuple(sorted((int(a[r, c]), int(b[r, c]))))
            pair.setdefault(key, []).append((r, c))
    for (ra, rb), pts in pair.items():
        mask = np.zeros((h, w), bool)
        for r, c in pts:
            mask[r, c] = True
        cl, ncl = ndi.label(ndi.binary_dilation(mask, iterations=3), structure=np.ones((3, 3)))
        for j in range(1, ncl + 1):
            idx = [(r, c) for r, c in pts if cl[r, c] == j]
            r, c = max(idx, key=lambda t: edt[t])  # passage centre: deepest point of the watershed line
            width = 2 * edt[r, c]
            x, y = to_world(r, c)
            if width > 3.0:  # wide opening between two cores = open space, not a doorway
                continue
            cls = 'impossible' if width < ROBOT_W else 'narrow' if width < ROBOT_W + 0.2 else 'passable'
            cands.append({'id': f'door_cand_{len(cands):03d}', 'x': round(x, 3), 'y': round(y, 3), 'width_m': round(float(width), 3),
                          'regions': [ra, rb], 'class': cls, 'status': 'UNVALIDATED'})
    nclu = {}
    for d in cands:
        nclu[tuple(d['regions'])] = nclu.get(tuple(d['regions']), 0) + 1
    for d in cands:  # one single neck between two regions = door-like; many necks = clutter splitting one room
        d['kind'] = 'door_like' if nclu[tuple(d['regions'])] == 1 else 'clutter_split'
    cands.sort(key=lambda d: d['id'])
    region_area = {int(k): round(float((ws == k).sum() * res * res), 1) for k in range(1, len(keep) + 1)}
    narrow_free = float(((edt > 0) & (edt * 2 < ROBOT_W)).sum() * res * res)
    result = {
        'map_dir': str(MAP), 'pgm_sha256': sha, 'manifest_sha_match': man.get('sha256', man.get('pgm_sha256', sha)) == sha,
        'resolution': res, 'origin': origin, 'shape_hw': [h, w],
        'cells': {'free': int(free.sum()), 'occupied': int(occ.sum()), 'unknown': int(unknown.sum())},
        'unknown_treated_as': 'obstacle (never free)',
        'free_components': {'count': int(n), 'largest_m2': round(float(sizes.max() * res * res), 1),
                            'small_islands_m2_total': round(float((sizes.sum() - sizes.max()) * res * res), 1)},
        'main_free_m2': round(float(mainfree.sum() * res * res), 1),
        'robot': {'width_m': round(ROBOT_W, 3), 'circumscribed_radius_m': round(ROBOT_R, 3)},
        'regions': region_area, 'door_candidates': cands, 'door_class_counts': {k: sum(c['class'] == k and c['kind'] == 'door_like' for c in cands) for k in ('impossible', 'narrow', 'passable')},
        'caveat': 'regions/doors are geometric bottlenecks of the 2D map, not semantic rooms; UNVALIDATED until checked against the sim world (separate from the Nav2 map)',
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'main_office_analysis.json').write_text(json.dumps(result, indent=1))
    ov = np.stack([np.where(occ, 0, np.where(unknown, 150, 255))] * 3, -1).astype(np.uint8)
    rng = np.random.default_rng(1)
    for k in range(1, len(keep) + 1):
        col = rng.integers(120, 230, 3)
        ov[(ws == k)] = (ov[(ws == k)] * 0.4 + col * 0.6).astype(np.uint8)
    for d in cands:
        r = int(h - 1 - (d['y'] - origin[1]) / res); c = int((d['x'] - origin[0]) / res)
        colr = {'impossible': (255, 0, 0), 'narrow': (255, 140, 0), 'passable': (0, 160, 0)}[d['class']]
        ov[max(r - 3, 0):r + 4, max(c - 3, 0):c + 4] = colr
    Image.fromarray(ov).save(OUT / 'main_office_overlay.png')
    print(json.dumps({k: result[k] for k in ('pgm_sha256', 'cells', 'free_components', 'main_free_m2', 'door_class_counts')}))
    print('regions', len(keep), 'candidates', len(cands))


if __name__ == '__main__':
    sys.exit(main())
