import pickle, numpy as np, json
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']
n0=D['plane0'][:3]; d0=D['plane0'][3]
xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
EMPTY=[(66616,66665),(66701,66845),(67019,67113),(67123,67336)]
def inwin(s,W): return any(a<=s<=b for a,b in W)
TH=[0.03,0.05,0.07,0.10,0.15,0.20]
RB=[(0.45,1.0),(1.0,1.5),(1.5,2.0),(2.0,2.5)]
res={}
cells={}
nfr=0
for m,P in zip(meta,pts):
    if not inwin(m['sod'],EMPTY) or 'nf' not in m: continue
    nfr+=1
    for mode in ('pf','glob'):
        n,d=(np.asarray(m['nf']),m['df']) if mode=='pf' else (n0,d0)
        h=P[:,:3]@n+d; x=P[:,:3]@f; y=P[:,:3]@L; r=np.hypot(x,y); b=np.degrees(np.arctan2(y,x))
        for T in TH:
            for (a0,a1) in RB:
                s=(h>T)&(h<0.5)&(r>=a0)&(r<a1)&(np.abs(b)<=56)
                k=(mode,T,a0); c=res.setdefault(k,[0,0])
                c[0]+=int(s.sum()); c[1]+=int(s.sum()>0)
                if mode=='pf' and s.any():
                    ij=np.floor(np.column_stack((x[s],y[s]))/0.05).astype(int)
                    for t in set(map(tuple,ij)): cells.setdefault((T,a0),{}).setdefault(t,0); cells[(T,a0)][t]+=1
print('empty frames',nfr)
for k,v in sorted(res.items()):
    pc=cells.get((k[1],k[2]),{}) if k[0]=='pf' else {}
    pers=sum(1 for c in pc.values() if c>0.5*nfr);
    print(k,'pts/frame %.3f frames_with_pts %.4f'%(v[0]/nfr,v[1]/nfr), ('cells_ever %d persistent>50%% %d max_cell_rate %.3f'%(len(pc),pers,max(pc.values())/nfr) if pc else ''))
pickle.dump((res,cells,nfr),open('fp.pkl','wb'))
