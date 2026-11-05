#!/usr/bin/env python3
"""Analyse trust_trace_copy.py TSV (T3/T3L/T4/T5) joined with ingress_trace observer CSV. Usage: copy_analyze.py DIR TAG"""
import collections, json, sys
D, TAG = sys.argv[1], sys.argv[2]
def P(v):
    if not v: return None
    s = sorted(v); g = lambda p: round(s[min(len(s)-1, int(p*(len(s)-1)+.5))], 4)
    return dict(n=len(s), p50=g(.5), p95=g(.95), p99=g(.99), max=round(s[-1], 4))
obs = collections.defaultdict(dict)
for ln in open(f'{D}/ingress_{TAG}.csv').read().splitlines()[1:]:
    t, s, m, r = ln.split(','); obs[t].setdefault(int(s), int(r))
key_topic = {'/qazbot_lio/scan': ('scan', 'scan'), '/qazbot_lio/odom_level': ('odom_msg', 'odom_level')}
t3 = collections.defaultdict(dict); seg = collections.defaultdict(lambda: collections.defaultdict(list)); evs = []; rej = collections.Counter()
pending = {}
for ln in open(f'{D}/copy_{TAG}.tsv').read().splitlines()[1:]:
    ev, key, mono, rr, src, extra = (ln.split('\t') + [''])[:6]
    mono, rr, src = int(mono), int(rr), int(src)
    if ev == 'T3' and key in key_topic:
        k, top = key_topic[key]; pending[k] = (mono, rr, src)
        seg[k]['src_to_T3'].append((rr - src) / 1e9)
        if src in obs[top]: seg[k]['obs_to_T3'].append((rr - obs[top][src]) / 1e9); seg[k]['src_to_obs'].append((obs[top][src] - src) / 1e9)
    elif ev == 'T3' and key == '/qazbot_d1_map/tf':
        pending['tf'] = (mono, rr, src)
    elif ev == 'T3L' and key in ('_scan', '_odom', '_tf'):
        k = {'_scan': 'scan', '_odom': 'odom_msg', '_tf': 'tf'}[key]
        if k in pending: seg[k]['T3_to_lock'].append((mono - pending[k][0]) / 1e9)
    elif ev == 'T4':
        ok = extra.startswith('ok'); 
        if not ok: rej[(key, extra.split(';')[0])] += 1
        k = key if key in ('scan', 'odom_msg') else ('tf' if key == 'odom' else None)
        if k and k in pending:
            seg[k]['T3_to_T4'].append((mono - pending[k][0]) / 1e9)
            t3[k][src] = pending[k][0]
            if key == 'odom': t3['odom'][src] = pending[k][0]
    elif ev == 'T5in':
        tin = mono
    elif ev == 'T5eval':
        x = json.loads(extra); x['eval_enter'] = mono; x['eval_exit'] = rr; x['tick_in'] = tin; evs.append(x)
out = dict(tag=TAG, rejections=dict((f'{a}:{b}', c) for (a, b), c in rej.items()), evals=len(evs))
for k, v in seg.items(): out[k] = {f: P(x) for f, x in v.items()}
ticks = [e['eval_enter'] for e in evs]
out['tick_gap'] = P([(b - a) / 1e9 for a, b in zip(ticks, ticks[1:])])
out['tick_in_to_eval'] = P([(e['eval_enter'] - e['tick_in']) / 1e9 for e in evs])
out['eval_duration'] = P([e['eval_ns'] / 1e9 for e in evs])
for k, src_f, t_f in (('scan', 'scan_src', 'scan_t'), ('odom_msg', 'odom_msg_src', 'odom_msg_t'), ('odom', 'odom_src', 'odom_t')):
    ages = [e['now'] - e[t_f] for e in evs if e[t_f] is not None and e[t_f] > -1e300]
    T3e = []
    for e in evs:
        s = e[src_f]
        if s is None: continue
        sn = int(round(s * 1e9)); m3 = t3[k].get(sn) or t3[k].get(sn + 1) or t3[k].get(sn - 1)
        if m3: T3e.append(e['now'] - m3 / 1e9)
    out[k + '_age_at_eval'] = P(ages); out[k + '_T3_to_eval'] = P(T3e)
stale = [e for e in evs if any(r.startswith('T1_scan') or r.startswith('T1_odom') for r in e['reasons'] or [])]
out['stale_ticks'] = len(stale)
out['stale_examples'] = [dict(reasons=e['reasons'], scan_age=round(e['now'] - e['scan_t'], 3), odom_age=round(e['now'] - e['odom_t'], 3)) for e in stale[:20]]
out['reason_counts'] = dict(collections.Counter(r for e in evs for r in e['reasons'] or []))
print(json.dumps(out, indent=1))
