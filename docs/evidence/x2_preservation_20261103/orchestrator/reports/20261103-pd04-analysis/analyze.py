#!/usr/bin/env python3
"""Stage 2: costmap metrics per observed event for A_live, A_emu, C_emu, C_nav2 (and A_nav2 if present)."""
import json
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cmlib import D, T, hms, live_A, nav2, pose_at, sod, to_body, zones, FOV  # noqa: E402

seg = pickle.load(open(D + '/seg.pkl', 'rb'))
emu = pickle.load(open(D + '/emu.pkl', 'rb'))
END = None

# observed events (from raw-cloud segmentation, seg.log / detail.py)
EV = [
    dict(id='O1', what='person F', on=T(18, 34, 6), off=T(18, 34, 51.5), bsec=(-40, 12)),
    dict(id='O2', what='person L-edge', on=T(18, 34, 52.5), off=T(18, 35, 19), bsec=(30, 60)),
    dict(id='O4', what='low object R-edge (box?)', on=T(18, 35, 44), off=T(18, 36, 26.5), bsec=(-60, -50)),
    dict(id='O6', what='low object L-edge (box?)', on=T(18, 36, 44.5), off=T(18, 36, 54.5), bsec=(44, 56)),
]
WALK = [(T(18, 31, 6), T(18, 31, 8.5)), (T(18, 31, 35.5), T(18, 31, 40)), (T(18, 35, 19), T(18, 35, 25)),
        (T(18, 35, 37), T(18, 35, 44)), (T(18, 35, 52), T(18, 35, 55)), (T(18, 36, 26.5), T(18, 36, 30)),
        (T(18, 36, 54.5), T(18, 36, 58)), (T(18, 38, 34), T(18, 38, 42))]
EMPTY = [(T(18, 30, 16), T(18, 31, 5.5)), (T(18, 31, 41), T(18, 34, 5)), (T(18, 36, 59), T(18, 38, 33)),
         (T(18, 38, 43), T(18, 42, 16))]


def region_cells(ev, src='raw', dil=2):
    pts = []
    for x in seg[src]:
        if ev['on'] + 0.5 <= x['s'] <= ev['off'] - 0.5:
            for c in x['cl']:
                p = c['pts']
                b = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
                p = p[(b >= ev['bsec'][0]) & (b <= ev['bsec'][1]) & (np.hypot(p[:, 0], p[:, 1]) < 2.0)]
                if len(p):
                    pts.append(p[:, :2])
    P = np.concatenate(pts)
    k = np.unique(np.floor(P / 0.05).astype(int), axis=0)
    cells = set()
    for dx in range(-dil, dil + 1):
        for dy in range(-dil, dil + 1):
            cells.update((a + dx, b + dy) for a, b in k)
    return cells, len(P)


def count_in(B, cells):
    if not len(B):
        return 0, 0
    k = np.floor(B / 0.05).astype(int)
    m = np.array([(a, b) in cells for a, b in k])
    bb = np.degrees(np.arctan2(B[:, 1], B[:, 0]))
    edge = m & ((bb > FOV[1]) | (bb < FOV[0]))
    return int(m.sum()), int(edge.sum())


def emu_src(v):
    out = []
    for k, xy, pose in zip(emu['keys'], emu['out'][v], emu['poses']):
        xy = np.asarray(xy, float)
        out.append(dict(s=sod(k), ns=k, B=to_body(xy, pose[:3]) if len(xy) else np.empty((0, 2)), xy=xy))
    return out


def main():
    srcs = {'A_live': live_A(), 'A_emu': emu_src('A_rec'), 'C_emu': emu_src('C')}
    for v in ('C', 'A_rec'):
        if os.path.exists(D + f'/nav2_{v}.pkl'):
            srcs['C_nav2' if v == 'C' else 'A_nav2'] = nav2(v)
    end_s = srcs['A_live'][-1]['s']
    regs = {e['id']: region_cells(e) for e in EV}
    res = dict(events={}, sources=list(srcs), zones_final={}, empty={}, walk={})
    for e in EV:
        cells, npts = regs[e['id']]
        ne = [x for x in EV if x['on'] > e['on']]
        nxt = min([x['on'] for x in ne] + [end_s])
        r = dict(what=e['what'], on=hms(e['on']), off=hms(e['off']), region_cells=len(cells), region_pts=npts)
        for name, S in srcs.items():
            pres = [count_in(p['B'], cells) for p in S if e['on'] + 1.0 <= p['s'] <= e['off']]
            after = [(p['s'], *count_in(p['B'], cells)) for p in S if e['off'] + 0.5 <= p['s'] <= end_s]
            pc = [c for c, _ in pres]
            first0 = next((s for s, c, _ in after if c == 0), None)
            first0e = next((s for s, _, ce in after if ce == 0), None)
            r[name] = dict(
                present_min=min(pc) if pc else None, present_med=float(np.median(pc)) if pc else None,
                present_max=max(pc) if pc else None, present_zero_pubs=sum(c == 0 for c in pc), n_present_pubs=len(pc),
                present_edge_max=max((ce for _, ce in pres), default=None),
                after_first=after[0][1:] if after else None,
                after_at_next_event=next(((c, ce) for s, c, ce in after if s >= nxt), None),
                after_final=after[-1][1:] if after else None,
                t_to_zero_s=None if first0 is None else round(first0 - e['off'], 1),
                t_to_zero_edge_s=None if first0e is None else round(first0e - e['off'], 1),
                residual_duration_s=round(end_s - e['off'], 1) if first0 is None else None)
        res['events'][e['id']] = r
    for name, S in srcs.items():
        res['zones_final'][name] = dict(first=zones(S[0]['B']), last=zones(S[-1]['B']),
                                        t_first=hms(S[0]['s']), t_last=hms(S[-1]['s']))
        # lethal r<1.5 |b|<=60 during empty windows (no object present)
        em = []
        for a, b in EMPTY:
            ps = [p for p in S if a <= p['s'] <= b]
            if not ps:
                continue
            z0, z1 = zones(ps[0]['B']), zones(ps[-1]['B'])
            em.append(dict(win=f'{hms(a)}-{hms(b)}', first=z0, last=z1))
        res['empty'][name] = em
        # walkers: edge cells before / after each transient pass
        wk = []
        for a, b in WALK:
            pre = [p for p in S if p['s'] < a]
            post = [p for p in S if p['s'] >= b + 1.0]
            if pre and post:
                z0, z1 = zones(pre[-1]['B']), zones(post[0]['B'])
                wk.append(dict(win=f'{hms(a)}-{hms(b)}', edge_before=z0['edgeL'] + z0['edgeR'],
                               edge_after=z1['edgeL'] + z1['edgeR'], inner_after=z1['inner']))
        res['walk'][name] = wk
    json.dump(res, open(D + '/analysis.json', 'w'), indent=1, default=str)
    # brief print
    for eid, r in res['events'].items():
        print(eid, r['what'], r['on'], r['off'], 'cells', r['region_cells'])
        for name in srcs:
            x = r[name]
            print(f"   {name:7s} present min/med/max {x['present_min']}/{x['present_med']}/{x['present_max']} zero_pubs {x['present_zero_pubs']}/{x['n_present_pubs']} edge_max {x['present_edge_max']} | after first {x['after_first']} at_next {x['after_at_next_event']} final {x['after_final']} t0 {x['t_to_zero_s']} t0edge {x['t_to_zero_edge_s']}")
    for name in srcs:
        print(name, 'zones first', res['zones_final'][name]['first'], 'last', res['zones_final'][name]['last'])
    for name in srcs:
        print(name, 'walk', [(w['win'][:8], w['edge_before'], w['edge_after'], w['inner_after']) for w in res['walk'][name]])
    for name in srcs:
        print(name, 'empty', [(w['win'][:8], w['first']['inner'], w['last']['inner'], w['last']['edgeL'] + w['last']['edgeR']) for w in res['empty'][name]])


if __name__ == '__main__':
    main()
