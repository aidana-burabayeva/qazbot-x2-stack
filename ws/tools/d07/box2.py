import pickle, numpy as np
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']; full=D['full']
n0=D['plane0'][:3]; xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
EMPTY=[(66616,66665),(66701,66845),(67019,67113),(67123,67336)]
sod=np.array([m['sod'] for m in meta])
def fr(i,P=None):
    m=meta[i]; P=pts[i] if P is None else P; n=np.asarray(m['nf']); h=P[:,:3]@n+m['df']; x=P[:,:3]@f; y=P[:,:3]@L; return x,y,h
bg={}; nbg=0
for i,m in enumerate(meta):
    if any(a<=m['sod']<=b for a,b in EMPTY):
        nbg+=1; x,y,h=fr(i); s=(h>0.04)&(h<0.6)
        for t in set(map(tuple,np.floor(np.column_stack((x[s],y[s]))/0.05).astype(int))): bg[t]=bg.get(t,0)+1
# win21: weak clusters
idx=[i for i in range(len(meta)) if 66990<=sod[i]<=67004.5]
occ={}
for i in idx:
    x,y,h=fr(i); s=(h>0.04)&(h<0.6)
    for t in set(map(tuple,np.floor(np.column_stack((x[s],y[s]))/0.05).astype(int))): occ[t]=occ.get(t,0)+1
new=sorted([(c,t) for t,c in occ.items() if bg.get(t,0)<0.02*nbg],reverse=True)[:8]
print('win21 frames',len(idx),'non-bg cells (frames, cell, r, b):',[(c,t,round(np.hypot(*(np.array(t)*0.05+0.025)),2),round(np.degrees(np.arctan2(t[1]+.5,t[0]+.5)),1)) for c,t in new])
# full frames r<3.5 in win21: any non-floor points 3.0-3.5?
fk=[k for k in full if 66990<=meta[k-1]['sod']<=67004.5]
print('full frames in win21',len(fk))
# per-frame box top & counts for +48 box and R box, incl. decimation estimate and floor pts in footprint
for name,(a,b),bb,rr in [('box_p48',(67005,67014.5),(44,56),(0.9,1.6)),('box45_R',(66955,66986),(-59,-50),(0.95,1.25))]:
    tops=[];cnt=[];flo=[];band=[]
    for i in range(len(meta)):
        if not a<=sod[i]<=b: continue
        x,y,h=fr(i); r=np.hypot(x,y); be=np.degrees(np.arctan2(y,x))
        s=(be>=bb[0])&(be<=bb[1])&(r>=rr[0])&(r<=rr[1])
        o=s&(h>0.04)&(h<0.6)
        cnt.append(o.sum()); tops.append(h[o].max() if o.any() else 0); band.append((o&(h>0.329+0.035*r)).sum())
    cnt=np.array(cnt); tops=np.array(tops); band=np.array(band)
    print(name,'frames',len(cnt),'raw obj pts/frame p10/p50/p90 %d/%d/%d'%tuple(np.percentile(cnt,[10,50,90])),'top p10/p50/p90/max %.3f/%.3f/%.3f/%.3f'%(*np.percentile(tops,[10,50,90]),tops.max()),
          'frames with >=1 above live band %d (%.0f%%)'%((band>0).sum(),100*(band>0).mean()), 'est. after 1/3 decimation pts/frame %.1f'%(np.median(cnt)/3))
    # elevation of top: visible?
