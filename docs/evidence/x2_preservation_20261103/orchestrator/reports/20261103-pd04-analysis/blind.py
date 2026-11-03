import sys,pickle,numpy as np,os
sys.path.insert(0,'.'); sys.path.insert(0,'/home/agi/control_backend/tests/d04')
from cmlib import *
import analyze
srcs={'A_emu':analyze.emu_src('A_rec'),'C_emu':analyze.emu_src('C'),'A_live':live_A()}
if os.path.exists(D+'/nav2_C.pkl'): srcs['C_nav2']=nav2('C')
if os.path.exists(D+'/nav2_A_rec.pkl'): srcs['A_nav2']=nav2('A_rec')
CMIN,CMAX=-57.7,56.2
def keys(p):
    return set(map(tuple,np.round(p['xy'][:,:2]/0.05).astype(int))) if len(p['xy']) else set()
for nm,S in srcs.items():
    clr_blind=0; clr_cov=0; max_blind=0; blind_cells_end=0; ev=[]
    prev=None; prevp=None
    for p in S:
        k=keys(p)
        if prev is not None:
            gone=prev-k
            if gone:
                G=np.array(list(gone))*0.05
                B=to_body(G,pose_at(p['ns']))
                b=np.degrees(np.arctan2(B[:,1],B[:,0])); r=np.hypot(B[:,0],B[:,1])
                inwin=(np.abs(B[:,0])<1.9)&(np.abs(B[:,1])<1.9)   # not a rolling-window drop
                bl=inwin&((b>CMAX+1)|(b<CMIN-1))
                clr_blind+=int(bl.sum()); clr_cov+=int((inwin&~bl).sum())
                if bl.sum(): ev.append((hms(p['s']),int(bl.sum()), np.round(b[bl][:3],0).tolist()))
        if len(p['B']):
            b=np.degrees(np.arctan2(p['B'][:,1],p['B'][:,0]))
            nb=int(((b>CMAX+1)|(b<CMIN-1)).sum()); max_blind=max(max_blind,nb)
        prev=k
    b=np.degrees(np.arctan2(S[-1]['B'][:,1],S[-1]['B'][:,0])) if len(S[-1]['B']) else np.array([])
    print(nm,'clears in blind bearings (beyond C ray coverage +-1deg):',clr_blind,'clears in covered:',clr_cov,'max lethal blind cells',max_blind,'blind at end',int(((b>CMAX+1)|(b<CMIN-1)).sum()), ev[:5])
