import sys,numpy as np
from combo import *
W=0.5
allq={w:[] for w in (0.25,0.5,1.0,2.0)}; ally={w:[] for w in allq}
for b in sys.argv[1:]:
    R,L,S,ts,x,y,yaw=load(b)
    sh=np.array([s[1] for s in S]); k=np.clip(np.searchsorted(sh,ts)-1,0,len(S)-1)
    st=np.array([(S[i][2]['fsm_state.current_state']==4 and S[i][2]['action_info.action_desc']=='STAND_DEFAULT' and S[i][2]['action_info.status.value']==100) for i in k])
    # event mask: any window of 1s with disp>1.5mm or dyaw>1.5mrad -> mark +-4s
    j1=np.searchsorted(ts,ts-1.0); mv=(np.hypot(x-x[j1],y-y[j1])>0.0015)|(np.abs(yaw-yaw[j1])>0.0015)
    ev=np.where(mv)[0]; bad=~st.copy()
    for i in ev:
        a=np.searchsorted(ts,ts[i]-5.0); bad[a:np.searchsorted(ts,ts[i]+4.0)]=True
    # also 5 s after nonstanding
    ok=np.zeros(len(ts),bool); lb=ts[0]-1e9
    for i in range(len(ts)):
        if not st[i]: lb=ts[i]
        ok[i]=ts[i]-lb>5.0
    q=ok&~bad
    for w in allq:
        j=np.searchsorted(ts,ts-w); g=q&q[j]&(ts-ts[j]>0.9*w)
        allq[w].append(np.hypot(x-x[j],y-y[j])[g]); ally[w].append(np.abs(yaw-yaw[j])[g])
    print(b,'quiet samples',q.sum(),'of',len(ts),flush=True)
for w in allq:
    d=np.concatenate(allq[w]); dy=np.concatenate(ally[w])
    print('W=%.2f n=%d disp mm p50 %.3f p99 %.3f p99.99 %.3f max %.3f | yaw urad p99 %.1f max %.1f'%(w,len(d),np.median(d)*1e3,np.percentile(d,99)*1e3,np.percentile(d,99.99)*1e3,d.max()*1e3,np.percentile(dy,99)*1e6,dy.max()*1e6))
