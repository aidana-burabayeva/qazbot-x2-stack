import pickle, numpy as np
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']
n0=D['plane0'][:3]; xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
sod=np.array([m['sod'] for m in meta])
def stats(a,b,bb,rr,lab):
    H=[];n=[];per=[]
    for i in range(len(meta)):
        if not a<=sod[i]<=b: continue
        m=meta[i];P=pts[i]; h=P[:,:3]@np.asarray(m['nf'])+m['df']; x=P[:,:3]@f; y=P[:,:3]@L; r=np.hypot(x,y); be=np.degrees(np.arctan2(y,x))
        s=(be>=bb[0])&(be<=bb[1])&(r>=rr[0])&(r<=rr[1])&(h>0.04)&(h<0.6)
        H.append(h[s]); per.append(s.sum())
    H=np.concatenate(H); per=np.array(per)
    print(lab,'frames',len(per),'frames with pts',(per>0).sum(),'pts/frame mean %.2f max %d'%(per.mean(),per.max()),'h p50/p95/max %.3f/%.3f/%.3f'%((*np.percentile(H,[50,95]),H.max()) if len(H) else (0,0,0)))
stats(66990,67004.5,(51,58),(0.9,1.3),'win21 L-edge +51..58 0.9-1.3')
stats(66701,66845,(51,58),(0.9,1.3),'EMPTY2 same region')
stats(67019,67113,(51,58),(0.9,1.3),'EMPTY3 same region')
stats(67005,67014.5,(44,56),(0.9,1.3),'box_p48 +44..56 0.9-1.3')
