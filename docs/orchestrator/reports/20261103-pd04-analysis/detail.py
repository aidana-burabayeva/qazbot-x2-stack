import pickle,numpy as np,sys
D=sys.argv[1]; seg=pickle.load(open(D+'/seg.pkl','rb'))
def hms(s): s=int(s); return '%02d:%02d:%02d'%(s//3600,(s%3600)//60,s%60)
src=sys.argv[2]; a,b=[int(x.split(':')[0])*3600+int(x.split(':')[1])*60+int(x.split(':')[2]) for x in sys.argv[3:5]]
step=float(sys.argv[5]) if len(sys.argv)>5 else 1
last=-1e9
for x in seg[src]:
    if a<=x['s']<=b and x['s']-last>=step:
        last=x['s']
        cs=[c for c in x['cl'] if c['n']>=int(sys.argv[6] if len(sys.argv)>6 else 8)]
        print(hms(x['s']), ' '.join(f"[c({c['x']:.2f},{c['y']:.2f}) b{c['bmin']:+.0f}..{c['bmax']:+.0f} rmin{c['rmin']:.2f} h{c['hmin']:.2f}-{c['hmax']:.2f} n{c['n']}]" for c in cs))
