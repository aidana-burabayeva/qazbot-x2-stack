import sys,pickle,collections,numpy as np
from combo import *
tot=collections.defaultdict(lambda:[0,[],[]])
for b in open('bags.txt').read().split():
    R,L,S,ts,x,y,yaw=load(b); V=R.get('/aima/mc/locomotion/velocity',[])
    if len(V)<10: continue
    v,wz,j=winstats(ts,x,y,yaw,0.1)
    vt=np.array([r[0] for r in V]); sh=np.array([s[1] for s in S]); k=np.clip(np.searchsorted(sh,ts)-1,0,len(S)-1)
    c=np.clip(np.searchsorted(vt,ts+0*ts)-1,0,len(V)-1)  # latest cmd by recv time
    rt=np.array([r[1] for r in L])
    c=np.clip(np.searchsorted(vt,rt)-1,0,len(V)-1)
    fresh=(rt-vt[c])<0.3
    for i in np.where(fresh)[0][::5]:
        f,l,a=V[c[i]][3:6]
        cls='turn_in_place' if abs(a)>0.05 and abs(f)<0.02 and abs(l)<0.02 else 'zero_cmd' if max(abs(f),abs(l),abs(a))<0.01 else 'translating'
        sf=S[k[i]][2]; key=(cls,sf['fsm_state.current_state'],sf['action_info.action_desc'],sf['action_info.status.value'])
        tot[key][0]+=1; tot[key][1].append(v[i]); tot[key][2].append(wz[i])
for key,(n,vs,ws) in sorted(tot.items()):
    print(key,n,'v p50 %.4f p99 %.3f  wz p50 %.3f p99 %.3f'%(np.median(vs),np.percentile(vs,99),np.median(ws),np.percentile(ws,99)))
