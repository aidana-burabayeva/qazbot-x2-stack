import csv, glob, math, os, statistics as st, sys
def run(path):
    r=[{k:float(v) for k,v in x.items()} for x in csv.DictReader(open(path))]
    if not r: return None
    # anchor offset = error while the robot is (nearly) static at the start: median of the first 3 seconds
    head=[x for x in r if x['t']<=4.0] or r[:20]
    pe=[x['pos_err'] for x in r]; ye=[abs(x['yaw_err']) for x in r]
    # worst-case error of a point 7 m ahead of the robot caused by the yaw offset alone
    yoff=st.median(x['yaw_err'] for x in head)
    return dict(n=len(r), pos_static=round(st.median(x['pos_err'] for x in head),3), yaw_static=round(yoff,3),
                pos_max=round(max(pe),3), pos_p95=round(sorted(pe)[int(.95*len(pe))],3), yaw_max=round(max(ye),3), yaw_p95=round(sorted(ye)[int(.95*len(ye))],3),
                err7m=round(abs(yoff)*7,2), moved_m=round(math.hypot(r[-1]['truth_x']-r[0]['truth_x'], r[-1]['truth_y']-r[0]['truth_y']),2))
out={}
for p in sorted(glob.glob('*.csv')):
    k=os.path.basename(p)[:-4]; out[k]=run(p)
print('%-18s %5s %9s %9s %8s %8s %8s %8s %8s' % ('run','n','pos_stat','yaw_stat','pos_max','pos_p95','yaw_max','yaw_p95','7m_eff'))
for k,v in out.items():
    if v: print('%-18s %5d %9.3f %9.3f %8.3f %8.3f %8.3f %8.3f %8.2f' % (k,v['n'],v['pos_static'],v['yaw_static'],v['pos_max'],v['pos_p95'],v['yaw_max'],v['yaw_p95'],v['err7m']))
import json; json.dump(out,open('../ab_summary.json','w'),indent=1)
