#!/usr/bin/env python3
"""Restore editor-lost unknown cells from a geometrically identical reference map."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def derive(edited_yaml, reference_yaml, output):
    edited_yaml, reference_yaml, output = map(Path, (edited_yaml, reference_yaml, output))
    edited_meta, reference_meta = [yaml.safe_load(p.read_text()) for p in (edited_yaml, reference_yaml)]
    for key in ('origin', 'resolution', 'negate'):
        if edited_meta[key] != reference_meta[key]:
            raise ValueError(f'mismatched {key}: reference mask cannot be aligned safely')
    if edited_meta['negate'] != 0:
        raise ValueError('only non-negated maps are supported')
    paths = [p.parent / m['image'] for p, m in ((edited_yaml, edited_meta), (reference_yaml, reference_meta))]
    edited, reference = [np.asarray(Image.open(p).convert('L')) for p in paths]
    if edited.shape != reference.shape:
        raise ValueError('map dimensions differ')
    if not set(np.unique(edited)) <= {0, 254} or not set(np.unique(reference)) <= {0, 205, 254}:
        raise ValueError('expected binary edited map and trinary reference map')
    restored = (reference == 205) & (edited == 254)
    result = edited.copy()
    result[restored] = 205
    assert np.array_equal(result[edited == 0], edited[edited == 0])
    assert np.array_equal(result[reference != 205], edited[reference != 205])
    output.mkdir(parents=True, exist_ok=False)
    Image.fromarray(result).save(output / 'map.pgm')
    Image.fromarray(result).save(output / 'preview.png')
    meta = dict(edited_meta, image='map.pgm', mode='trinary', free_thresh=0.196)
    (output / 'map.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = dict(source='Edited t2 with unknown mask restored from aligned D1 export',
                    edited_image=str(paths[0].resolve()), edited_sha256=digest(paths[0]),
                    reference_image=str(paths[1].resolve()), reference_sha256=digest(paths[1]),
                    source_yaml_sha256=digest(edited_yaml), reference_yaml_sha256=digest(reference_yaml),
                    pgm_sha256=digest(output / 'map.pgm'), map_yaml_sha256=digest(output / 'map.yaml'),
                    restored_unknown_cells=int(restored.sum()),
                    pixel_histogram={str(v): int((result == v).sum()) for v in np.unique(result)},
                    rule='Only edited white cells that were reference unknown become 205; all edited obstacles preserved',
                    status='Digital twin only; not a physical-robot qualification')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--edited', required=True)
    ap.add_argument('--reference', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    derive(args.edited, args.reference, args.out)
