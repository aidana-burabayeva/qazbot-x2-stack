import pickle, numpy as np
D = pickle.load(open('win.pkl', 'rb'))
d_ob_reg, d_reg_raw, raw_log_to_ob_log, rawstamp_to_ob_log, dt_ob = [], [], [], [], []
for w in D['wins']:
    rs = np.array([s for _, s, _ in w['raw']]); rl = np.array([t for t, _, _ in w['raw']])
    gs = np.array([s for _, s, _ in w['reg']]); gl = np.array([t for t, _, _ in w['reg']])
    for t, s, f, P in w['ob']:
        j = np.argmin(np.abs(gs - s)); d_ob_reg.append((s - gs[j]) / 1e6)
        i = np.searchsorted(rs, s, 'right') - 1   # last raw scan stamped <= ob stamp
        d_reg_raw.append((s - rs[i]) / 1e6); rawstamp_to_ob_log.append((t - rs[i]) / 1e6); raw_log_to_ob_log.append((t - rl[i]) / 1e6)
    obl = np.array([t for t, *_ in w['ob']]); dt_ob += list(np.diff(obl) / 1e6)
for n, v in [('ob_stamp-reg_stamp(nearest)', d_ob_reg), ('ob_stamp-raw_stamp(prev)', d_reg_raw), ('ob_log-raw_stamp', rawstamp_to_ob_log),
             ('ob_log-raw_log', raw_log_to_ob_log), ('ob period', dt_ob)]:
    v = np.array(v); print(f'{n}: p50 {np.percentile(v,50):.1f} p99 {np.percentile(v,99):.1f} min {v.min():.1f} max {v.max():.1f} ms')
