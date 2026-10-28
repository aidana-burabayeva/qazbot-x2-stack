#!/usr/bin/env python3
"""Offline analysis for burst_probe.py outputs (read-only). Usage: burst_analyze.py DIR TAG [--win 1.0]
Bursts = (a) odom_level or scan DDS-receive gap > 0.3 s at the observer, (b) live trust T1_scan*/T1_odom*,
(c) trust age at eval odom_msg > 0.3 s or scan > 0.45 s. For each burst: per-stamp hop latencies from DDS
source timestamps (publisher-side, observer-independent) and 100 ms schedstat bins of the suspect processes."""
import bisect, collections, json, sys
D, TAG = sys.argv[1], sys.argv[2]
WIN = 1.0
rows = collections.defaultdict(list)
for ln in open(f'{D}/bingress_{TAG}.csv').read().splitlines()[1:]:
    t, *v = ln.split(','); rows[t].append(tuple(int(x) for x in v))     # stamp, src, rcv, take_m, take_r
for t in rows: rows[t].sort(key=lambda x: x[1])
st = [json.loads(l) for l in open(f'{D}/bstatus_{TAG}.jsonl')]
meta = json.load(open(f'{D}/bmeta_{TAG}.json'))
R2M = meta['real_minus_mono_ns']
def P(v):
    if not v: return None
    s = sorted(v); g = lambda p: round(s[min(len(s)-1, int(p*(len(s)-1)+.5))], 4)
    return dict(n=len(s), p50=g(.5), p95=g(.95), p99=g(.99), max=round(s[-1], 4))
by = {t: {r[0]: r for r in v} for t, v in rows.items()}
raw_st = sorted(by.get('raw_lidar', {}))
def raw_for(lio_stamp):
    i = bisect.bisect_right(raw_st, lio_stamp) - 1
    if i >= 0 and 80_000_000 <= lio_stamp - raw_st[i] <= 110_000_000: return by['raw_lidar'][raw_st[i]]
def hops(s):
    """per lio stamp: dict of hop latencies (s) from source timestamps"""
    lo = by['lio_odom'].get(s); h = dict(stamp=s)
    rw = raw_for(s)
    if rw: h['hal_pub'] = (rw[1] - rw[0]) / 1e9; h['raw_dds'] = (rw[2] - rw[1]) / 1e9
    if lo:
        h['lio_odom_pub_from_stamp'] = (lo[1] - s) / 1e9
        if rw: h['fastlio'] = (lo[1] - rw[1]) / 1e9
    lc = by.get('lio_cloud', {}).get(s)
    if lc and rw: h['fastlio_cloud'] = (lc[1] - rw[1]) / 1e9
    ol = by.get('odom_level', {}).get(s)
    if ol and lo: h['bridge_odom'] = (ol[1] - lo[1]) / 1e9
    if ol: h['odom_level_dds'] = (ol[2] - ol[1]) / 1e9; h['odom_level_age_at_rcv'] = (ol[2] - s) / 1e9
    sc = by.get('scan', {}).get(s)
    if sc and lc: h['bridge_scan'] = (sc[1] - lc[1]) / 1e9
    if sc: h['scan_age_at_rcv'] = (sc[2] - s) / 1e9
    return h
H = [hops(s) for s in sorted(by.get('lio_odom', {}))]
out = dict(tag=TAG, load=[meta['load0'].split()[:3], meta['load1'].split()[:3]], topics={}, hops={})
for t, v in rows.items():
    sg = [(b[1] - a[1]) / 1e9 for a, b in zip(v, v[1:])]
    rg = sorted(((b[2] - a[2]) / 1e9) for a, b in zip(sorted(v, key=lambda x: x[2]), sorted(v, key=lambda x: x[2])[1:]))
    out['topics'][t] = dict(n=len(v), src_minus_stamp=P([(r[1] - r[0]) / 1e9 for r in v]), src_gap=P(sg),
                            rcv_gap_max=round(rg[-1], 4) if rg else None, take_minus_rcv=P([(r[4] - r[2]) / 1e9 for r in v]))
for k in ('hal_pub', 'raw_dds', 'fastlio', 'fastlio_cloud', 'bridge_odom', 'bridge_scan', 'odom_level_dds', 'odom_level_age_at_rcv', 'scan_age_at_rcv'):
    out['hops'][k] = P([h[k] for h in H if k in h])
NOM = {k: (v['p50'] if v else 0) for k, v in out['hops'].items()}
out['scan_coverage'] = dict(lio_clouds=len(by.get('lio_cloud', {})), scans=len(by.get('scan', {})))
# ---- burst candidates (real ns) ----
ev = []
for t, thr in (('odom_level', .3), ('scan', .3), ('lio_odom', .3), ('raw_lidar', .3), ('imu', .1)):
    v = sorted(rows.get(t, []), key=lambda x: x[2])
    for a, b in zip(v, v[1:]):
        if (b[2] - a[2]) / 1e9 > thr: ev.append((b[2], f'{t}_rcv_gap', round((b[2] - a[2]) / 1e9, 3)))
    v = rows.get(t, [])
    for a, b in zip(v, v[1:]):
        if (b[1] - a[1]) / 1e9 > thr: ev.append((b[1], f'{t}_src_gap', round((b[1] - a[1]) / 1e9, 3)))
stale = 0
for x in st:
    rs = [r for r in (x['reasons'] or []) if r.startswith('T1_scan') or r.startswith('T1_odom')]
    ia = x.get('input_ages') or {}
    if rs: ev.append((x['src'], 'trust:' + '+'.join(rs), dict(scan=ia.get('scan'), odom_msg=ia.get('odom_msg'), odom=ia.get('odom')))); stale += 1
    elif (ia.get('odom_msg') or 0) > .3 or (ia.get('scan') or 0) > .45:
        ev.append((x['src'], 'trust_age', dict(scan=ia.get('scan'), odom_msg=ia.get('odom_msg'))))
ev.sort(key=lambda e: e[0])
bursts = []
for e in ev:
    if bursts and e[0] - bursts[-1]['t_last'] < 1.5e9: bursts[-1]['ev'].append(e); bursts[-1]['t_last'] = e[0]
    else: bursts.append(dict(t0=e[0], t_last=e[0], ev=[e]))
out['trust_ticks'] = len(st); out['trust_stale_ticks'] = stale
# ---- sched ----
S = [json.loads(l) for l in open(f'{D}/bsched_{TAG}.jsonl')]
SM = json.load(open(f'{D}/bsched_meta_{TAG}.json')); names = SM['names']
Sr = [s['r'] for s in S]
def sched_window(r0, r1):
    i0 = max(0, bisect.bisect_left(Sr, r0) - 1); i1 = min(len(S) - 1, bisect.bisect_left(Sr, r1))
    a, b = S[i0], S[i1]; dt = (b['r'] - a['r']) / 1e9
    res = dict(dt=round(dt, 3))
    proc = collections.defaultdict(lambda: [0, 0, 0, 0]); thr = {}
    for tid, v in b['t'].items():
        u = a['t'].get(tid)
        if not u: continue
        k, comm = names[tid]; d = [v[0] - u[0], v[1] - u[1], v[3] - u[3], v[4] - u[4]]
        for j in range(4): proc[k][j] += d[j]
        if d[0] + d[1] > 0.05 * dt * 1e9: thr[f'{k}:{tid}:{comm}'] = dict(run=round(d[0]/1e9, 3), wait=round(d[1]/1e9, 3), vol=d[2], invol=d[3])
    res['proc'] = {k: dict(run=round(v[0]/1e9, 3), wait=round(v[1]/1e9, 3), vol=v[2], invol=v[3]) for k, v in proc.items()}
    res['threads'] = dict(sorted(thr.items(), key=lambda kv: -kv[1]['wait'])[:10])
    if 'cpu' in a and 'cpu' in b:
        idle = sum((cb[3] - ca[3]) for ca, cb in zip(a['cpu'], b['cpu'])); tot = sum(sum(cb) - sum(ca) for ca, cb in zip(a['cpu'], b['cpu']))
        res['host_idle_pct'] = round(100 * idle / max(tot, 1), 1)
    res['procs_running_max'] = max(s.get('procs_running', 0) for s in S[i0:i1 + 1])
    return res
out['sched_run'] = sched_window(Sr[0], Sr[-1])
for k, v in out['sched_run']['proc'].items():
    v['cpu_pct'] = round(100 * v['run'] / out['sched_run']['dt'], 1); v['wait_pct'] = round(100 * v['wait'] / out['sched_run']['dt'], 1)
out['nominal_hops_p50'] = NOM
out['bursts'] = []
for b in bursts:
    t0 = b['t0']
    stamps = [s for s in sorted(by.get('lio_odom', {})) if t0 - 1.2e9 <= s <= t0 + 0.3e9]
    # also stamps of raw lidar in the window lacking FAST-LIO output
    chain = []
    for s in stamps:
        h = hops(s); chain.append({k: (round(v, 3) if isinstance(v, float) else (round((v - t0) / 1e9, 3) if k == 'stamp' else v)) for k, v in h.items()})
    raw_missing = [round((r - t0) / 1e9, 3) for r in raw_st if t0 - 1.2e9 <= r <= t0 + .3e9 and (r + 95_200_000) not in by.get('lio_odom', {})
                   and not any(abs(r + 95_200_000 - s) < 5_000_000 for s in stamps)]
    # first delayed segment: for each chain element find the largest hop excess over nominal
    worst = None
    for h in chain:
        for k in ('hal_pub', 'raw_dds', 'fastlio', 'bridge_odom', 'odom_level_dds', 'bridge_scan'):
            if k in h:
                ex = h[k] - NOM.get(k, 0)
                if worst is None or ex > worst[1]: worst = (k, round(ex, 3), h['stamp'])
    bins = []
    for j in range(-8, 4):
        r0 = t0 + j * 100_000_000; w = sched_window(r0, r0 + 100_000_000)
        bins.append(dict(t=round(j * .1, 1), idle=w.get('host_idle_pct'), pr=w['procs_running_max'],
                         **{k: [v['run'], v['wait'], v['vol'], v['invol']] for k, v in w['proc'].items()}))
    out['bursts'].append(dict(t0_real=t0 / 1e9, events=[(round((e[0] - t0) / 1e9, 3), e[1], e[2]) for e in b['ev']][:20],
                              chain_rel_t0=chain, raw_without_lio_output=raw_missing, worst_hop_excess=worst,
                              sched_window=sched_window(t0 - 800_000_000, t0 + 100_000_000), sched_bins_100ms=bins))
print(json.dumps(out, indent=1, default=str))
