import pickle, numpy as np
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']
n0=D['plane0'][:3]; xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
EMPTY=[(66616,66665),(66701,66845),(67019,67113),(67123,67336)]
def inwin(s): return any(a<=s<=b for a,b in EMPTY)
TH=[0.05,0.07,0.10]
st={T:dict(fr=0,k2=0,k3=0,consec=0,dec=0) for T in TH}; prev={T:set() for T in TH}; nfr=0; lastsod=None
for m,P in zip(meta,pts):
    if 'nf' not in m or not inwin(m['sod']): continue
    nfr+=1; n=np.asarray(m['nf']); h=P[:,:3]@n+m['df']; x=P[:,:3]@f; y=P[:,:3]@L; r=np.hypot(x,y); b=np.degrees(np.arctan2(y,x))
    contig = lastsod is not None and m['sod']-lastsod<0.15; lastsod=m['sod']
    for T in TH:
        s=(h>T)&(h<0.5)&(r>=0.45)&(r<2.0)&(np.abs(b)<=56)
        ij=np.floor(np.column_stack((x[s],y[s]))/0.05).astype(int)
        u,c=np.unique(ij,axis=0,return_counts=True) if len(ij) else (np.zeros((0,2),int),np.zeros(0,int))
        cs=set(map(tuple,u))
        st[T]['fr']+=len(cs)>0; st[T]['k2']+=(c>=2).any(); st[T]['k3']+=(c>=3).any()
        # dilate prev by 1 cell for consecutive-frame persistence
        if contig and cs:
            pd={(a+dx,bb+dy) for (a,bb) in prev[T] for dx in (-1,0,1) for dy in (-1,0,1)}
            st[T]['consec']+=len(cs&pd)>0
        prev[T]=cs
        st[T]['dec']+= (np.random.default_rng(len(meta)).random(s.sum())<1/3).any() if s.any() else 0
print('empty frames',nfr,'(r 0.45-2.0, |b|<=56, h<0.5, per-frame plane)')
for T,v in st.items(): print('T=%.2f'%T, {k:('%d (%.4f)'%(c,c/nfr)) for k,c in v.items()})
