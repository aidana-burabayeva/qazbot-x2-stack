#!/usr/bin/env python3
"""Stage 1: foreground segmentation timeline (registered + raw cloud) -> seg.pkl + printed 1 Hz timeline."""
import datetime as dt
import os
import pickle
import sys

import numpy as np
from scipy import ndimage

D = sys.argv[1]
small = pickle.load(open(os.path.join(D, 'small.pkl'), 'rb'))
frames = pickle.load(open(os.path.join(D, 'frames.pkl'), 'rb'))
raws = pickle.load(open(os.path.join(D, 'raw.pkl'), 'rb'))


def tl(ts):
    return dt.datetime.fromtimestamp(ts / 1e9).strftime('%H:%M:%S.%f')[:-4]


def sec_of_day(ts):
    d = dt.datetime.fromtimestamp(ts / 1e9)
    return d.hour * 3600 + d.minute * 60 + d.second + d.microsecond * 1e-6


def hms(s):
    return '%02d:%02d:%02d' % (s // 3600, (s % 3600) // 60, s % 60)


BG0, BG1 = 18 * 3600 + 30 * 60 + 20, 18 * 3600 + 30 * 60 + 48   # step 0 empty baseline (operator log)
VOX = 0.05


def enc(k):
    k = k.astype(np.int64) + 1000
    return (k[:, 0] * 4096 + k[:, 1]) * 4096 + k[:, 2]


def bg_set(items):
    keys = []
    for it in items:
        q = it['reg']
        if len(q):
            k = np.unique(np.floor(q / VOX).astype(np.int32), axis=0)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        keys.append(enc(k + np.array([dx, dy, dz], np.int32)))
    return np.unique(np.concatenate(keys))


def fg(q, bg):
    if not len(q):
        return q
    m = ~np.isin(enc(np.floor(q / VOX).astype(np.int32)), bg) & (q[:, 2] >= 0.05)
    return q[m]


def clusters(q, minpts=8):
    if len(q) < minpts:
        return []
    g = 0.08
    ij = np.floor(q[:, :2] / g).astype(int) + 60
    ij = np.clip(ij, 0, 119)
    occ = np.zeros((120, 120), bool)
    occ[ij[:, 0], ij[:, 1]] = True
    lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
    L = lab[ij[:, 0], ij[:, 1]]
    out = []
    for c in range(1, n + 1):
        p = q[L == c]
        if len(p) < minpts:
            continue
        r = np.hypot(p[:, 0], p[:, 1])
        b = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
        cx, cy = p[:, 0].mean(), p[:, 1].mean()
        out.append(dict(n=len(p), b=float(np.degrees(np.arctan2(cy, cx))), r=float(np.hypot(cx, cy)),
                        rmin=float(r.min()), bmin=float(b.min()), bmax=float(b.max()),
                        hmax=float(np.percentile(p[:, 2], 98)), hmin=float(p[:, 2].min()),
                        x=float(cx), y=float(cy), pts=p))
    return out


def run(items, label):
    bgi = [it for it in items if BG0 <= sec_of_day(it['ts']) <= BG1]
    bg = bg_set(bgi)
    res = []
    for it in items:
        cs = clusters(fg(it['reg'], bg))
        res.append(dict(ts=it['ts'], s=sec_of_day(it['ts']), cl=cs))
    print(f'{label}: bg frames {len(bgi)}, bg voxels {len(bg)}, items {len(items)}')
    return res


segR = run(frames, 'registered')
segW = run(raws, 'raw')
pickle.dump(dict(reg=segR, raw=segW), open(os.path.join(D, 'seg.pkl'), 'wb'))


def line(seg, s):
    its = [x for x in seg if s <= x['s'] < s + 1]
    if not its:
        return '-'
    x = its[len(its) // 2]
    cs = sorted([c for c in x['cl'] if c['n'] >= 15 and c['rmin'] < 2.6], key=lambda c: c['rmin'])
    return ' '.join(f"[{c['b']:+.0f}deg r{c['rmin']:.2f} h{c['hmax']:.2f} n{c['n']}]" for c in cs[:4]) or '.'


s0 = int(min(x['s'] for x in segR))
s1 = int(max(x['s'] for x in segR))
for s in range(s0, s1 + 1):
    print(hms(s), 'REG', line(segR, s), '| RAW', line(segW, s))
