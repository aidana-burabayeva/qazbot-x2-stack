import pickle,numpy as np,sys
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
D=sys.argv[1]; seg=pickle.load(open(D+'/seg.pkl','rb'))
t0=18*3600+33*60; t1=18*3600+39*60
fig,ax=plt.subplots(3,1,figsize=(22,13),sharex=True)
for src,mk in (('raw',1),):
    T=[];B=[];H=[];Rr=[]
    for x in seg[src]:
        if not (t0<=x['s']<=t1): continue
        for c in x['cl']:
            p=c['pts']; 
            if len(p)>400: p=p[::len(p)//400]
            T.append(np.full(len(p),x['s']-t0)); B.append(np.degrees(np.arctan2(p[:,1],p[:,0]))); H.append(p[:,2]); Rr.append(np.hypot(p[:,0],p[:,1]))
    T=np.concatenate(T);B=np.concatenate(B);H=np.concatenate(H);Rr=np.concatenate(Rr)
    sc=ax[0].scatter(T,B,c=H,s=1,cmap='jet',vmin=0,vmax=1.8); ax[0].set_ylabel('bearing deg (+left)'); plt.colorbar(sc,ax=ax[0])
    ax[1].scatter(T,Rr,c=H,s=1,cmap='jet',vmin=0,vmax=1.8); ax[1].set_ylabel('range m')
    lo=H<0.6
    ax[2].scatter(T[lo],B[lo],c=Rr[lo],s=2,cmap='viridis',vmin=0.3,vmax=2.0); ax[2].set_ylabel('bearing, pts h<0.6 (color=range)')
for a in ax:
    a.grid(True); a.set_xticks(range(0,t1-t0+1,10))
ax[2].set_xticklabels([f'{(t0+v)//60%60:02d}:{(t0+v)%60:02d}' for v in range(0,t1-t0+1,10)],rotation=90)
plt.tight_layout(); plt.savefig(D+'/timeline.png',dpi=60)
