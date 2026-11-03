import pickle, numpy as np
D = pickle.load(open('win.pkl', 'rb')); T0 = D['T0']
lat = dict(raw=[], reg=[], ob=[], cm=[]); match = []
for w in D['wins']:
    print('=== win', w['a'], 'frames', w['ob'][0][2], w['raw'][0][2], w['reg'][0][2])
    tfp = set((t.header.frame_id, t.child_frame_id) for _, m in w['tf'] for t in m.transforms); print(' tf pairs', tfp)
    rs = np.array([s for _, s, _ in w['raw']]);
    for t, s, *_ in w['raw']: lat['raw'].append((t - s) / 1e6)
    for t, s, *_ in w['reg']: lat['reg'].append((t - s) / 1e6)
    for t, s in w['cm']: lat['cm'].append((t - s) / 1e6)
    for t, s, f, P in w['ob']:
        lat['ob'].append((t - s) / 1e6); match.append(np.min(np.abs(rs - s)) / 1e6)
        C = P[(np.abs(P[:, 1]) < 0.40) & (P[:, 0] > 0) & (P[:, 0] < 1.2)] if len(P) else P
        print(f"  t={(t-T0)/1e9:7.2f} n={len(P):4d} corr<1.2:{len(C):3d} xmin={C[:,0].min() if len(C) else float('nan'):.2f} "
              f"in0.65:{int((C[:,0]<0.65).sum()) if len(C) else 0}")
for k, v in lat.items():
    v = np.array(v); print(k, 'log-stamp ms p50 %.1f p99 %.1f max %.1f min %.1f n %d' % (np.percentile(v, 50), np.percentile(v, 99), v.max(), v.min(), len(v)))
m = np.array(match); print('ob stamp vs nearest raw stamp ms p50 %.2f max %.2f' % (np.median(m), m.max()))
