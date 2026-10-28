import json, sys, bisect, glob, os, statistics
EV = sys.argv[1]
def pct(v, p):
    if not v: return None
    s = sorted(v); return round(s[min(len(s)-1, int(p*(len(s)-1)+0.5))], 4)
def st(v): return dict(n=len(v), p50=pct(v,.5), p95=pct(v,.95), p99=pct(v,.99), max=pct(v,1)) if v else None
res = {}
for f in sorted(glob.glob(EV + '/probe_*.jsonl')):
    name = os.path.basename(f)[6:-6]
    lines = open(f).read().splitlines(); hdr = json.loads(lines[0]); topics = hdr['topics']
    rows = [json.loads(l) for l in lines[1:]]
    by = {}
    for r in rows: by.setdefault(r[0], []).append(r)
    out = {}
    short = {t: t.split('/')[-1] if 'qazbot' not in t else t for t in topics}
    for i, t in enumerate(topics):
        rs = by.get(i, [])
        if t == 'trust_status':
            reasons = {}; ages = {'scan': [], 'odom': [], 'odom_msg': []}
            for r in rs:
                j = json.loads(r[4])
                for x in j['reasons'] or []: reasons[x] = reasons.get(x, 0) + 1
                for k in ages:
                    a = (j.get('input_ages') or {}).get(k)
                    if a is not None: ages[k].append(a)
            out['trust'] = dict(n=len(rs), reasons=reasons, input_ages={k: st(v) for k, v in ages.items()},
                                input_callbacks_last=json.loads(rs[-1][4]).get('input_callbacks') if rs else None)
            continue
        if t == '/qazbot_d1_map/tf':
            sub = {}
            for r in rs: sub.setdefault(r[4], []).append(r)
            for k, v in sub.items():
                ages = [r[2]-r[1] for r in v]; gaps = [b[3]-a[3] for a, b in zip(v, v[1:])]
                out['tf ' + k] = dict(age=st(ages), gap=st(gaps))
            continue
        if not rs: out[t] = None; continue
        ages = [r[2]-r[1] for r in rs]; gaps = [b[3]-a[3] for a, b in zip(rs, rs[1:])]
        sgaps = [b[1]-a[1] for a, b in zip(rs, rs[1:])]
        out[t] = dict(n=len(rs), rate=round((len(rs)-1)/(rs[-1][3]-rs[0][3]), 2), size_p50=pct([r[4] for r in rs], .5),
                      age=st(ages), recv_gap=st(gaps), stamp_gap=st(sgaps), nonincreasing=sum(1 for g in sgaps if g <= 0))
    # same-stamp correlation
    def idx(i): return {round(r[1], 6): r for r in by.get(i, [])}
    T = {t: i for i, t in enumerate(topics)}
    creg = by.get(T['/qazbot_lio/cloud_registered'], []); odo = idx(T['/qazbot_lio/odometry']); scan = idx(T['/qazbot_lio/scan'])
    imu = by.get(T['/aima/hal/sensor/lidar_chest_front/imu'], []); imu_st = [r[1] for r in imu]
    raw = by.get(T.get('/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', -1), []); raw_st = [r[1] for r in raw]
    corr = dict(creg_age=[], odo_minus_creg=[], scan_minus_creg=[], raw_age=[], lio_stage=[], stamp_offset=[], imu_gate_wait=[], lio_after_inputs=[], imu_cover_age=[])
    for c in creg:
        k = round(c[1], 6); corr['creg_age'].append(c[2]-c[1])
        if k in odo: corr['odo_minus_creg'].append(odo[k][2]-c[2])
        if k in scan: corr['scan_minus_creg'].append(scan[k][2]-c[2])
        j = bisect.bisect_right(raw_st, c[1]) - 1
        if raw and j >= 0 and 0 <= c[1]-raw_st[j] < 0.2:
            rr = raw[j]; corr['raw_age'].append(rr[2]-rr[1]); corr['lio_stage'].append(c[2]-rr[2]); corr['stamp_offset'].append(c[1]-rr[1])
            m = bisect.bisect_left(imu_st, c[1])
            if m < len(imu):
                gate = imu[m][2]; corr['imu_cover_age'].append(gate - c[1])
                corr['imu_gate_wait'].append(max(0, gate - rr[2]))
                corr['lio_after_inputs'].append(c[2] - max(gate, rr[2]))
    out['_same_stamp'] = {k: st(v) for k, v in corr.items()}
    # Trust-like age at 5 Hz ticks (now - latest stamp received), probe-side
    def tick_age(i):
        rs = by.get(i, []);
        if len(rs) < 3: return None
        t0, t1 = rs[0][2], rs[-1][2]; ages = []; j = 0; last = None; t = t0 + 1
        while t < t1:
            while j < len(rs) and rs[j][2] <= t: last = rs[j][1] if last is None else max(last, rs[j][1]); j += 1
            ages.append(t - last); t += 0.2
        return dict(st(ages), over_0_5=sum(1 for a in ages if a > 0.5))
    out['_tick_age_5Hz'] = {t: tick_age(T[t]) for t in ['/qazbot_lio/scan', '/qazbot_native/odom', '/qazbot_lio/cloud_registered']}
    # level_bridge backlog: does scan receipt lag grow relative to cloud receipt in bursts?
    sc = [scan[round(c[1], 6)][2]-c[2] for c in creg if round(c[1], 6) in scan]
    out['_level_bridge_backlog'] = dict(scan_minus_creg_series_max_run_over_50ms=max((sum(1 for _ in g) for k, g in __import__('itertools').groupby(sc, key=lambda x: x > .05) if k), default=0),
                                        n_scan_per_creg=round(len(sc)/max(1, len(creg)), 3))
    res[name] = out
json.dump(res, open(EV + '/analysis.json', 'w'), indent=1)
for name, out in res.items():
    print('#####', name)
    for k, v in out.items():
        if k.startswith('tf') or k in ('trust',): print(k, json.dumps(v)[:600]); continue
        print(k, json.dumps(v)[:700])
