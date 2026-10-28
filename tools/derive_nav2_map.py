#!/usr/bin/env python3
"""Derive a Nav2 map from a D1 map directory (procedure of first_map_nav2_v1, docs/NAVIGATION_STATUS.md §5,
docs/DECISIONS.md §1): the PGM is copied byte-identically; the YAML is the source YAML with only
free_thresh 0.25 -> 0.196, so that standard nav2_map_server (trinary) reads pixel 205 as unknown like the
D1 QORGAN loader (254 free, 0 occupied); origin, resolution and pixels are unchanged; a manifest records hashes.
Re-running on the 20.10 source reproduces first_map_nav2_v1 byte-identically (map.pgm, map.yaml, manifest.json).

Usage: derive_nav2_map.py SRC_DIR NEW_DST_DIR "SOURCE LABEL" [--extra]
           [--erase-imprint X Y R --erase-reason "TEXT"]
--extra adds provenance fields (source meta, pixel histogram, tool) on top of the v1 manifest fields.
--erase-imprint (opt-in; without it output is byte-identical to before): frees the imprint of the robot body.
  Seeds = occupied (0) cells within R m of (X,Y) (map frame, cell centres); each 8-connected occupied component
  containing a seed is set to 254 (free), UNLESS any of its cells lies farther than R+0.15 m from (X,Y)
  (then it is a wall/structure and is kept untouched). Unknown (205) cells within R of (X,Y) are also set to 254.
  The erase record (center, radius, cells, bbox, reason, skipped components) goes into manifest.json.
"""
import collections
import hashlib
import json
import shutil
import sys
from pathlib import Path

import yaml


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def pgm_histogram(p):
    d = p.read_bytes()
    fields, i = [], 0
    while len(fields) < 4:
        while d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b'#':
            while d[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not d[j:j + 1].isspace():
            j += 1
        fields.append(d[i:j])
        i = j
    assert fields[0] == b'P5' and int(fields[3]) == 255, fields
    w, h = int(fields[1]), int(fields[2])
    px = d[i + 1:]
    assert len(px) == w * h, (len(px), w, h)
    return w, h, dict(sorted(collections.Counter(px).items()))


def erase_imprint(px, w, h, res, origin, cx, cy, r, keep_margin=0.15):
    """Returns (new bytearray, record). Pixel row 0 = top; cell centre of (row, col) is
    (ox + (col+.5)*res, oy + (h-1-row+.5)*res)."""
    ox, oy = origin[0], origin[1]
    px = bytearray(px)

    def centre(i):
        row, col = divmod(i, w)
        return ox + (col + .5) * res, oy + (h - 1 - row + .5) * res

    def dist(i):
        x, y = centre(i)
        return ((x - cx) ** 2 + (y - cy) ** 2) ** .5

    seeds = [i for i in range(w * h) if px[i] == 0 and dist(i) <= r]
    seen, erase, skipped, ncomp = set(), set(), [], 0
    for s0 in seeds:
        if s0 in seen:
            continue
        comp, stack = [], [s0]
        seen.add(s0)
        while stack:
            i = stack.pop()
            comp.append(i)
            row, col = divmod(i, w)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = row + dr, col + dc
                    j = rr * w + cc
                    if 0 <= rr < h and 0 <= cc < w and j not in seen and px[j] == 0:
                        seen.add(j)
                        stack.append(j)
        far = max(dist(i) for i in comp)
        if far > r + keep_margin:
            skipped.append({'cells': len(comp), 'max_dist_m': round(far, 3)})
        else:
            ncomp += 1
            erase.update(comp)
    n_occ = len(erase)
    erase.update(i for i in range(w * h) if px[i] == 205 and dist(i) <= r)
    for i in erase:
        px[i] = 254
    rc = [divmod(i, w) for i in erase]
    xs = [centre(i)[0] for i in erase]
    ys = [centre(i)[1] for i in erase]
    rec = {'center': [cx, cy], 'radius_m': r, 'keep_margin_m': keep_margin, 'components_erased': ncomp,
           'components_kept_as_structure': skipped, 'cells_erased': len(erase), 'cells_occupied_erased': n_occ,
           'cells_unknown_erased': len(erase) - n_occ,
           'bbox_px_rowcol': [min(r_ for r_, _ in rc), min(c for _, c in rc), max(r_ for r_, _ in rc), max(c for _, c in rc)],
           'bbox_map_m': [round(min(xs), 3), round(min(ys), 3), round(max(xs), 3), round(max(ys), 3)]}
    return px, rec


def main():
    src, dst, label = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    extra = '--extra' in sys.argv[4:]
    y = yaml.safe_load((src / 'map.yaml').read_text())
    assert y['free_thresh'] == 0.25 and y['mode'] == 'trinary' and y['negate'] == 0 and y['occupied_thresh'] == 0.65, y
    w, h, hist = pgm_histogram(src / y['image'])
    # the 0.196 threshold is only meaningful for D1 trinary images {0, 205, 254}
    assert set(hist) <= {0, 205, 254}, hist
    dst.mkdir(parents=True, exist_ok=False)
    y['free_thresh'] = 0.196
    shutil.copyfile(src / y['image'], dst / y['image'])
    erase_rec = None
    if '--erase-imprint' in sys.argv:
        a = sys.argv
        k = a.index('--erase-imprint')
        cx, cy, r = float(a[k + 1]), float(a[k + 2]), float(a[k + 3])
        reason = a[a.index('--erase-reason') + 1]
        raw = (dst / y['image']).read_bytes()
        newpx, erase_rec = erase_imprint(raw[-w * h:], w, h, y['resolution'], y['origin'], cx, cy, r)
        erase_rec['reason'] = reason
        (dst / y['image']).write_bytes(raw[:-w * h] + bytes(newpx))
        _, _, hist = pgm_histogram(dst / y['image'])
    (dst / 'map.yaml').write_text(yaml.safe_dump(y, sort_keys=False))
    man = {'source': label, 'source_yaml_sha256': sha(src / 'map.yaml'), 'pgm_sha256': sha(dst / y['image']),
           'change': 'free_thresh 0.25 -> 0.196: 205 unknown, 254 free, 0 occupied; no pixel or geometry changes',
           'status': 'localization candidate, not accepted for movement'}
    if extra:
        meta = src / 'meta.yaml'
        man.update(source_meta_sha256=sha(meta) if meta.exists() else None,
                   source_meta_saved=str(yaml.safe_load(meta.read_text()).get('saved')) if meta.exists() else None,
                   map_yaml_sha256=sha(dst / 'map.yaml'), size_px=[w, h], resolution=y['resolution'],
                   origin=y['origin'], pixel_histogram={str(k): v for k, v in hist.items()},
                   derived_by='tools/derive_nav2_map.py (procedure of first_map_nav2_v1)')
    if erase_rec:
        man['source_pgm_sha256'] = sha(src / y['image'])
        man['derivation_params'] = {'argv': sys.argv[1:], 'free_thresh': '0.25 -> 0.196'}
        man['erase_record'] = erase_rec
        man['change'] += '; PLUS explicit erase of robot imprint (see erase_record)'
        man['pgm_sha256'] = sha(dst / y['image'])
        if extra:
            man['pixel_histogram'] = {str(k): v for k, v in hist.items()}
    (dst / 'manifest.json').write_text(json.dumps(man, indent=2) + '\n')


if __name__ == '__main__':
    main()
