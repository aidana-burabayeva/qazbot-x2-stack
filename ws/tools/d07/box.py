import pickle, numpy as np
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']; full=D['full']
n0=D['plane0'][:3]; d0=D['plane0'][3]
xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
EMPTY=[(66616,66665),(66701,66845),(67019,67113),(67123,67336)]
WIN={'box45_R':(66944,66948.5),'box45_R_b':(66955,66986),'win21':(66990,67004.5),'box_p48':(67005,67014.5),'person_Ledge':(66892,66919)}
def inwin(s,W): return any(a<=s<=b for a,b in W)
HT=0.04; RES=0.05
def frame(m,P):
    n,d=np.asarray(m['nf']),m['df']
    h=P[:,:3]@n+d; x=P[:,:3]@f; y=P[:,:3]@L
    return x,y,h
bg={}; nbg=0
for m,P in zip(meta,pts):
    if 'nf' not in m or not inwin(m['sod'],EMPTY): continue
    nbg+=1; x,y,h=frame(m,P); s=(h>HT)&(h<0.6)&(np.hypot(x,y)<3)
    for t in set(map(tuple,np.floor(np.column_stack((x[s],y[s]))/RES).astype(int))): bg[t]=bg.get(t,0)+1
print('bg frames',nbg,'bg cells',len(bg))
for name,(a,b) in WIN.items():
    idx=[i for i,m in enumerate(meta) if a<=m['sod']<=b and 'nf' in m]
    occ={}; 
    for i in idx:
        x,y,h=frame(meta[i],pts[i]); s=(h>HT)&(h<0.6)&(np.hypot(x,y)<3)
        for t in set(map(tuple,np.floor(np.column_stack((x[s],y[s]))/RES).astype(int))): occ[t]=occ.get(t,0)+1
    new=[t for t,c in occ.items() if c>=0.3*len(idx) and bg.get(t,0)<0.02*nbg]
    print('\n==',name,'frames',len(idx),'new cells',len(new))
    if not new: 
        # report top transient cells
        top=sorted(occ.items(),key=lambda kv:-kv[1])[:5]
        print(' top occ cells (cell, frames, bg):',[(t,c,bg.get(t,0)) for t,c in top]); continue
    # cluster by connectivity
    S=set(new); cl=[]
    while S:
        st=[S.pop()]; c=[st[0]]
        while st:
            u=st.pop()
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    v=(u[0]+dx,u[1]+dy)
                    if v in S: S.remove(v); st.append(v); c.append(v)
        cl.append(c)
    for c in sorted(cl,key=len,reverse=True)[:4]:
        C=set(c); per=[]; band=[]; flo=[]; hs=[]
        for i in idx:
            x,y,h=frame(meta[i],pts[i]); ij=np.floor(np.column_stack((x,y))/RES).astype(int)
            inC=np.array([tuple(t) in C for t in ij]) if len(ij) else np.zeros(0,bool)
            s=inC&(h>HT)&(h<0.6); r=np.hypot(x,y)
            per.append(s.sum()); band.append((s&(h>0.329+0.035*r)).sum()); hs.append(h[s])
            k=list(full.keys())
        hh=np.concatenate(hs) if hs else np.zeros(0)
        cx=np.mean([t[0] for t in c])*RES+RES/2; cy=np.mean([t[1] for t in c])*RES+RES/2
        print(' cluster cells %d centre r=%.2f b=%+.1f  rawpts/frame med %.0f min %d  h p50 %.3f p95 %.3f max %.3f  above-live-band pts/frame med %.1f max %d'%(
            len(c),np.hypot(cx,cy),np.degrees(np.arctan2(cy,cx)),np.median(per),min(per),np.median(hh),np.percentile(hh,95),hh.max(),np.median(band),max(band)))
