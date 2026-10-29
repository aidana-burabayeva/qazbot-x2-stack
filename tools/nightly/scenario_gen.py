#!/usr/bin/env python3
"""Phase C: deterministic scenario generator for main_office (offline).

python3 tools/nightly/scenario_gen.py --seed 1 --per-category 20 --out run/nightly/scenarios/seed1.jsonl
Same seed + same map + same config => byte-identical output. Start and goal yaw are independent.
Geometric feasibility (disc of the footprint half-width must fit) is separate from kinodynamic feasibility (L2/L3 decide);
invalid / impossible goals go to NEGATIVE scenarios with the expected result `safe_reject`, never into the positive set.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

ROOT = Path(__file__).resolve().parents[2]
MAP = Path(os.environ.get('MAP_DIR', ROOT / 'ws/src/maps/main_office'))
FP = np.array([[0.17, 0.229], [0.008, 0.34], [-0.182, 0.34], [-0.305, 0.229], [-0.305, -0.261], [-0.182, -0.372], [0.008, -0.372], [0.17, -0.261]])
HALF_W = float((FP[:, 1].max() - FP[:, 1].min()) / 2)   # 0.356: a disc this wide must fit through any gap
R_CIRC = float(np.hypot(FP[:, 0], FP[:, 1]).max())      # 0.372
SAFETY = 0.10
CATEGORIES = ['corridor_to_office', 'office_to_corridor', 'corridor_to_wing', 'wing_to_corridor', 'office_internal', 'corridor_internal',
              'doorway_crossing_in', 'doorway_crossing_out', 'near_obstacle_goal', 'yaw_only_far']
NEGATIVE = ['goal_in_obstacle', 'goal_in_unknown', 'goal_too_close_to_wall', 'goal_in_isolated_island']


class World:
    def __init__(self):
        yml = (MAP / 'map.yaml').read_text().splitlines()
        self.res = float([ln for ln in yml if ln.startswith('resolution')][0].split(':')[1])
        i = yml.index('origin:'); self.origin = [float(yml[i + k].strip()[2:]) for k in (1, 2)]
        self.sha = hashlib.sha256((MAP / 'map.pgm').read_bytes()).hexdigest()
        img = np.array(Image.open(MAP / 'map.pgm').convert('L'))
        p = (255 - img.astype(float)) / 255
        self.free, self.occ = p < 0.196, p > 0.65
        self.h, self.w = img.shape
        lab, n = ndi.label(self.free, structure=np.ones((3, 3)))
        sizes = ndi.sum(self.free, lab, range(1, n + 1)); self.lab = lab; self.main = lab == (int(np.argmax(sizes)) + 1)
        self.edt = ndi.distance_transform_edt(self.free) * self.res  # unknown and occupied are both obstacles
        self.edt_main = ndi.distance_transform_edt(self.main) * self.res
        # regions from the Phase B analysis (same parameters)
        from importlib import util
        a = json.loads((ROOT / 'docs/nightly/map/main_office_analysis.json').read_text())
        assert a['pgm_sha256'] == self.sha, 'analysis is for a different map'
        self.doors = [d for d in a['door_candidates'] if d['kind'] == 'door_like']
        from skimage.segmentation import watershed
        core, nc = ndi.label(self.edt_main > 1.0)
        cs = ndi.sum(np.ones_like(core), core, range(1, nc + 1)); keep = [i + 1 for i, s in enumerate(cs) if s * self.res ** 2 > 1.5]
        mk = np.zeros_like(core)
        for k, i in enumerate(keep, 1):
            mk[core == i] = k
        self.region = watershed(-self.edt_main, mk, mask=self.main)
        # room names: corridor = biggest region; wing/office = the two regions behind the door_like candidates; 4 merges into its clutter partner
        ra = {int(k): v for k, v in a['regions'].items()}
        self.corr = max(ra, key=ra.get)
        behind = [r for d in self.doors for r in d['regions'] if r != self.corr]
        w = {d['id']: [r for r in d['regions'] if r != self.corr][0] for d in self.doors}
        self.door_room = {d['id']: ('wing' if d['width_m'] > 1.2 else 'office') for d in self.doors}
        self.room_regions = {}
        for d in self.doors:
            self.room_regions.setdefault(self.door_room[d['id']], set()).add(w[d['id']])
        for r in ra:   # leftover regions (clutter splits) join the room that has the most shared clutter necks
            if r != self.corr and all(r not in s for s in self.room_regions.values()):
                self.room_regions.setdefault('office', set()).add(r)

    def w2c(self, x, y):
        return int(self.h - 1 - (y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res)

    def c2w(self, r, c):
        return self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (self.h - r - 0.5) * self.res

    def reachable(self, a, b):
        """Geometric reachability for a disc of the footprint half-width: same component after erosion by HALF_W."""
        m = self.edt_main >= HALF_W
        lab, _ = ndi.label(m, structure=np.ones((3, 3)))
        ra, ca = self.w2c(*a[:2]); rb, cb = self.w2c(*b[:2])
        return bool(lab[ra, ca] and lab[ra, ca] == lab[rb, cb])


def sample(world, rng, mask):
    rr, cc = np.nonzero(mask)
    i = int(rng.integers(len(rr)))
    x, y = world.c2w(rr[i], cc[i])
    return [round(float(x), 3), round(float(y), 3), round(float(rng.uniform(-np.pi, np.pi)), 3)]


def region_mask(world, room):
    ids = [world.corr] if room == 'corridor' else sorted(world.room_regions.get(room, []))
    return np.isin(world.region, ids) & (world.edt_main >= R_CIRC + SAFETY)


def gen(seed, per, cfg):
    W = World(); rng = np.random.default_rng(seed); out = []
    near = lambda d: np.isin(W.region, [W.corr] + [r for s in W.room_regions.values() for r in s]) & (W.edt_main >= R_CIRC + SAFETY) & (W.edt_main < R_CIRC + SAFETY + d)
    pairs = {'corridor_to_office': ('corridor', 'office'), 'office_to_corridor': ('office', 'corridor'), 'corridor_to_wing': ('corridor', 'wing'),
             'wing_to_corridor': ('wing', 'corridor'), 'office_internal': ('office', 'office'), 'corridor_internal': ('corridor', 'corridor')}
    for cat in CATEGORIES:
        for k in range(per):
            if cat in pairs:
                a, b = pairs[cat]; s, g = sample(W, rng, region_mask(W, a)), sample(W, rng, region_mask(W, b))
            elif cat.startswith('doorway_crossing'):
                d = W.doors[k % len(W.doors)]; room = W.door_room[d['id']]
                # start/goal 1.5-3 m either side of the door centre (door_like candidate, UNVALIDATED), lateral offset random
                def side(mask_room):
                    m = region_mask(W, mask_room); rr, cc = np.nonzero(m)
                    xs, ys = np.array([W.c2w(r, c) for r, c in zip(rr, cc)]).T
                    dist = np.hypot(xs - d['x'], ys - d['y']); sel = np.nonzero((dist > 1.5) & (dist < 3.0))[0]
                    if not len(sel): return None
                    j = int(sel[int(rng.integers(len(sel)))]); return [round(float(xs[j]), 3), round(float(ys[j]), 3), round(float(rng.uniform(-np.pi, np.pi)), 3)]
                p, q = side('corridor'), side(room)
                if p is None or q is None: continue
                s, g = (p, q) if cat.endswith('_in') else (q, p)
            elif cat == 'near_obstacle_goal':
                s, g = sample(W, rng, region_mask(W, 'corridor')), sample(W, rng, near(0.15))
            else:  # yaw_only_far: same spot, opposite yaw (rotate in place)
                s = sample(W, rng, region_mask(W, 'corridor')); g = [s[0], s[1], round(float((s[2] + np.pi * rng.uniform(.5, 1)) % (2 * np.pi) - np.pi), 3)]
            ok = W.reachable(s, g)
            sc = {'category': cat, 'start': s, 'goal': g, 'geometric_reachable': ok, 'expected': 'reach' if ok else 'safe_reject', 'door': None,
                  'positive': ok, 'kinodynamic': 'UNKNOWN (L2/L3 decide)'}
            out.append(sc)
    for cat in NEGATIVE:
        for k in range(max(1, per // 4)):
            s = sample(W, rng, region_mask(W, 'corridor'))
            if cat == 'goal_in_obstacle': rr, cc = np.nonzero(W.occ); i = int(rng.integers(len(rr))); gx, gy = W.c2w(rr[i], cc[i])
            elif cat == 'goal_in_unknown': rr, cc = np.nonzero(~W.free & ~W.occ & (ndi.distance_transform_edt(~(W.free | W.occ)) > 20)); i = int(rng.integers(len(rr))); gx, gy = W.c2w(rr[i], cc[i])
            elif cat == 'goal_too_close_to_wall': m = W.main & (W.edt_main > 0.1) & (W.edt_main < HALF_W - 0.05); rr, cc = np.nonzero(m); i = int(rng.integers(len(rr))); gx, gy = W.c2w(rr[i], cc[i])
            else:
                isl = W.free & ~W.main; rr, cc = np.nonzero(isl)
                if not len(rr): continue
                i = int(rng.integers(len(rr))); gx, gy = W.c2w(rr[i], cc[i])
            out.append({'category': 'NEG_' + cat, 'start': s, 'goal': [round(float(gx), 3), round(float(gy), 3), round(float(rng.uniform(-np.pi, np.pi)), 3)],
                        'geometric_reachable': False, 'expected': 'safe_reject', 'positive': False, 'door': None, 'kinodynamic': 'n/a'})
    meta = {'seed': seed, 'map_sha256': W.sha, 'config': cfg, 'generator': 'tools/nightly/scenario_gen.py v1', 'robot_half_width_m': round(HALF_W, 3),
            'safety_margin_m': SAFETY, 'doors_unvalidated': [d['id'] for d in W.doors]}
    for n, sc in enumerate(out):
        sc['id'] = f"s{seed}_{n:04d}"; sc['meta_seed'] = seed
    return meta, out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--seed', type=int, required=True); ap.add_argument('--per-category', type=int, default=20); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    meta, sc = gen(a.seed, a.per_category, {'per_category': a.per_category})
    p = Path(a.out); p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'w') as f:
        f.write(json.dumps({'meta': meta}) + '\n')
        for s in sc: f.write(json.dumps(s, sort_keys=True) + '\n')
    cnt = {}
    for s in sc: cnt[(s['category'], s['expected'])] = cnt.get((s['category'], s['expected']), 0) + 1
    print(len(sc), 'scenarios'); [print(' ', k, v) for k, v in sorted(cnt.items())]


if __name__ == '__main__':
    main()
