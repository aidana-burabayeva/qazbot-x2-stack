#!/usr/bin/env python3
"""Swept-footprint clearance of an L2/L3 pose trace against the Nav2 map (unknown = obstacle). Offline.
usage: clearance_from_trace.py <trace.json> [map_dir] -> prints JSON {min_clearance_m, collisions, left_right}"""
import json, sys
from pathlib import Path
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

FP = np.array([[0.17, 0.229], [0.008, 0.34], [-0.182, 0.34], [-0.305, 0.229], [-0.305, -0.261], [-0.182, -0.372], [0.008, -0.372], [0.17, -0.261]])


def main(trace, mapdir):
    mapdir = Path(mapdir)
    yml = (mapdir / 'map.yaml').read_text().splitlines(); res = float([l for l in yml if l.startswith('resolution')][0].split(':')[1])
    i = yml.index('origin:'); ox, oy = float(yml[i + 1].strip()[2:]), float(yml[i + 2].strip()[2:])
    img = np.array(Image.open(mapdir / 'map.pgm').convert('L')); free = (255 - img.astype(float)) / 255 < 0.196
    h = img.shape[0]; edt = ndi.distance_transform_edt(free) * res - res / 2
    pts = []
    for k in range(8):  # outline samples
        a, b = FP[k], FP[(k + 1) % 8]
        pts += [a + (b - a) * t for t in np.linspace(0, 1, 6)[:-1]]
    pts = np.array(pts)
    tr = np.array(json.loads(Path(trace).read_text()))
    best, minleft, minright, coll = 1e9, 1e9, 1e9, 0
    for t, x, y, th, *_ in tr:
        c, s = np.cos(th), np.sin(th)
        wx = x + c * pts[:, 0] - s * pts[:, 1]; wy = y + s * pts[:, 0] + c * pts[:, 1]
        cc = ((wx - ox) / res).astype(int); rr = (h - 1 - ((wy - oy) / res)).astype(int)
        v = edt[np.clip(rr, 0, h - 1), np.clip(cc, 0, edt.shape[1] - 1)]
        m = float(v.min()); best = min(best, m); coll += m < 0
        left = pts[:, 1] > 0
        minleft = min(minleft, float(v[left].min())); minright = min(minright, float(v[~left].min()))
    print(json.dumps({'min_clearance_m': round(best, 3), 'min_left_m': round(minleft, 3), 'min_right_m': round(minright, 3),
                      'samples_in_collision': int(coll), 'poses': len(tr), 'unknown_is_obstacle': True}))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else str(Path(__file__).resolve().parents[2] / 'ws/src/maps/main_office'))
