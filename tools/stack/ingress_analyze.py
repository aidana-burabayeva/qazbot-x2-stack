#!/usr/bin/env python3
"""Offline analysis for ingress_trace.py (+ trust_trace_copy.py) outputs. Read-only. Usage: ingress_analyze.py DIR TAG"""
import bisect, collections, json, sys
D, TAG = sys.argv[1], sys.argv[2]
rows = collections.defaultdict(list)
for ln in open(f'{D}/ingress_{TAG}.csv').read().splitlines()[1:]:
    t, s, m, r = ln.split(','); rows[t].append((int(s), int(m), int(r)))
st = [json.loads(l) for l in open(f'{D}/status_{TAG}.jsonl')]
meta = json.load(open(f'{D}/meta_{TAG}.json'))
def P(v):
    if not v: return None
    s = sorted(v); g = lambda p: round(s[min(len(s)-1, int(p*(len(s)-1)+.5))], 4)
    return dict(n=len(s), p50=g(.5), p95=g(.95), p99=g(.99), max=round(s[-1], 4))
off = [r - m for _, m, r in rows['scan']]; OFF = sorted(off)[len(off)//2] if off else 0   # real - mono (ns)
out = dict(tag=TAG, load=[meta['load0'].split()[:3], meta['load1'].split()[:3]], ros_minus_real_ns=meta['ros_minus_real_ns'],
           use_sim_time=meta['use_sim_time'], topics={})
first = {}
for t, v in rows.items():
    v.sort(key=lambda x: x[1])
    age = [(r - s) / 1e9 for s, m, r in v]
    sg = [(b[0] - a[0]) / 1e9 for a, b in zip(v, v[1:])]; rg = [(b[1] - a[1]) / 1e9 for a, b in zip(v, v[1:])]
    out['topics'][t] = dict(n=len(v), rate=round((len(v)-1)/((v[-1][1]-v[0][1])/1e9), 2) if len(v) > 1 else None,
                            recv_age=P(age), stamp_gap=P(sg), recv_gap=P(rg),
                            nonincreasing=sum(1 for g in sg if g <= 0))
    d = {}
    for s, m, r in v: d.setdefault(s, r)
    first[t] = d
def seg(a, b, shift=None):
    """b_recv - a_recv for same stamp (or nearest a stamp below b stamp within shift ns)"""
    A, B = first.get(a, {}), first.get(b, {}); res = []
    if shift is None:
        res = [(B[s] - A[s]) / 1e9 for s in B if s in A]
    else:
        ks = sorted(A)
        for s in B:
            i = bisect.bisect_left(ks, s) - 1
            if i >= 0 and 0 < s - ks[i] <= shift: res.append((B[s] - A[ks[i]]) / 1e9)
    return P(res)
lo = sorted(first.get('raw_lidar', {})); cl = sorted(first.get('lio_cloud', {}))
offs = []
for s in cl[:200]:
    i = bisect.bisect_left(lo, s) - 1
    if i >= 0: offs.append((s - lo[i]) / 1e9)
out['lio_stamp_minus_raw_stamp'] = P(offs)
out['segments'] = {
 'raw_lidar_recv -> lio_cloud_recv (FAST-LIO, stamp end-of-scan matched)': seg('raw_lidar', 'lio_cloud', 150_000_000),
 'raw_lidar_recv -> lio_odom_recv': seg('raw_lidar', 'lio_odom', 150_000_000),
 'lio_odom_recv -> lio_cloud_recv (same stamp)': seg('lio_odom', 'lio_cloud'),
 'lio_cloud_recv -> scan_recv (level_bridge, same stamp)': seg('lio_cloud', 'scan'),
 'lio_odom_recv -> odom_level_recv (level_bridge)': seg('lio_odom', 'odom_level'),
 'lio_odom_recv -> tf odom>base recv (level_bridge)': seg('lio_odom', 'tf:odom>base_footprint'),
}
sc = first.get('scan', {}); scs = sorted(sc)
out['scan_drops'] = dict(lio_clouds=len(cl), scans=len(scs),
                         scan_stamp_gaps_gt_0p25=sum(1 for a, b in zip(scs, scs[1:]) if b - a > 250_000_000),
                         clouds_without_scan_in_gap=[round((b - a) / 1e9, 3) for a, b in zip(scs, scs[1:]) if b - a > 250_000_000][:20])
# live trust: decompose each tick
def trust_ticks(src):
    res = []; obs = {k: sorted((r, s) for s, r in first.get(k, {}).items()) for k in ('scan', 'odom_level', 'tf:odom>base_footprint')}
    for x in st:
        if x['src'] != src: continue
        ia = x.get('input_ages') or {}; cb = x.get('cb') or {}
        tick = dict(recv_real=x['real_ns'], reasons=[r for r in x['reasons'] or [] if r.startswith('T1_scan') or r.startswith('T1_odom')])
        for key, cbn, top in (('scan', '_scan', 'scan'), ('odom_msg', '_odom', 'odom_level'), ('odom', '_tf', 'tf:odom>base_footprint')):
            A = ia.get(key); R = (cb.get(cbn) or {}).get('received')
            if A is None or R is None: continue
            Rr = int(R * 1e9) + OFF
            arr = obs[top]; i = bisect.bisect_right(arr, (Rr + 30_000_000, 0)) - 1   # newest stamp the observer had by trust T3 + 30 ms
            cand = [arr[j] for j in range(max(0, i - 3), i + 1) if arr[j][1] < Rr]
            if not cand: continue
            o_r, s = min(cand, key=lambda c: abs(c[0] - Rr))
            eval_real = s + int(A * 1e9)
            # newest stamp already received by the observer before the evaluation
            k = bisect.bisect_right(arr, (eval_real, 0)) - 1
            newest_obs = arr[k][1] if k >= 0 else None
            tick[key] = dict(age=A, stamp=s, src_to_obs=(o_r - s)/1e9, obs_to_T3=(Rr - o_r)/1e9, src_to_T3=(Rr - s)/1e9,
                             T3_to_eval=(eval_real - Rr)/1e9, eval_to_status_recv=(x['real_ns'] - eval_real)/1e9,
                             newer_at_observer=(newest_obs is not None and newest_obs > s),
                             newest_obs_age=None if newest_obs is None else (eval_real - newest_obs)/1e9)
        res.append(tick)
    return res
for src in ('live', 'copy'):
    tk = trust_ticks(src)
    if not tk: continue
    r = dict(ticks=len(tk), status_recv_gap=P([(b['recv_real'] - a['recv_real'])/1e9 for a, b in zip(tk, tk[1:])]))
    for key in ('scan', 'odom_msg', 'odom'):
        v = [t[key] for t in tk if key in t]
        r[key] = {f: P([x[f] for x in v]) for f in ('age', 'src_to_obs', 'obs_to_T3', 'src_to_T3', 'T3_to_eval', 'eval_to_status_recv')}
        r[key]['ticks_newer_msg_at_observer_not_in_trust'] = sum(1 for x in v if x['newer_at_observer'])
    ev = [t for t in tk if t['reasons']]
    r['stale_ticks'] = len(ev); r['stale_reason_counts'] = dict(collections.Counter(z for t in ev for z in t['reasons']))
    r['stale_events'] = [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in dict(reasons=t['reasons'], **{
        kk + '.' + f: t[kk][f] for kk in ('scan', 'odom_msg', 'odom') if kk in t for f in
        ('age', 'src_to_obs', 'obs_to_T3', 'T3_to_eval', 'newer_at_observer', 'newest_obs_age')}).items()} for t in ev[:40]]
    out['trust_' + src] = r
out['sched'] = json.load(open(f'{D}/sched_{TAG}.json'))
for k, v in out['sched'].items(): v['threads'] = dict(sorted(v['threads'].items(), key=lambda kv: -kv[1]['cpu_pct'])[:4])
print(json.dumps(out, indent=1, default=str))
