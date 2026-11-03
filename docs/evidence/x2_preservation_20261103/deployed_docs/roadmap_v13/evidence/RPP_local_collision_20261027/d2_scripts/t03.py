import replay, numpy as np, math, copy
E=replay.EV+'Static_map_trial_1230/'
snap=np.load(E+'local_costmap_before_clear.npy').reshape(80,80); SO=(-5.0,-2.0)
scans,tf=replay.load('Static_map_trial_1230'); T=replay.TF(tf)
print('first scan',scans[0][0],'tf range',tf[0][0],tf[-1][0])
for t in [4290,4301.5,4301.79,4301.84,4302.2,4349.0]: print(t, np.round(T.at(1793100000+t),4))
TS=[1793104301.60,1793104301.70,1793104301.79,1793104301.84,1793104301.89,1793104301.99,1793104302.09]
def lethal_set(g,ox,oy):
    jj,ii=np.nonzero(g==254); return set(zip(np.round(ox+(ii+.5)*.05,3),np.round(oy+(jj+.5)*.05,3)))
for mode in ['baseline','inf_fov','inf_guard2']:
    # phase to align with snapshot grid
    x0,y0,_=T.at(scans[0][0]); ox=SO[0]+round((x0-2-SO[0])/.05)*.05; oy=SO[1]+round((y0-2-SO[1])/.05)*.05
    L=replay.Layer(ox,oy); states={}
    def hook(sc,L):
        for t in TS:
            if sc[0]<=t-0.03: states[t]=(sc[0],L.g.copy(),L.ox,L.oy)
    replay.run(scans,tf,mode,L,None,1793104349.006,hook)
    A=lethal_set(L.g,L.ox,L.oy); Bs=lethal_set(snap,*SO)
    print(f'[{mode}] at 4349: replay lethal {len(A)} snapshot {len(Bs)} common {len(A&Bs)} origin {L.ox:.2f},{L.oy:.2f}')
    for t in TS:
        st,g,ox_,oy_=states[t]; LL=replay.Layer(ox_,oy_,g); p=T.at(t)
        out=[]
        for v,w in [(0.3,-0.129),(0.3,0.0)]:
            c,d=replay.check(LL,p,v,w); out.append(('COLL '+str(d[0][:2])+' cells '+str(d[0][5][:2])) if c else 'clear')
        # nearest lethal to robot and its bearing
        jj,ii=np.nonzero(g==254); wx=ox_+(ii+.5)*.05; wy=oy_+(jj+.5)*.05; dd=np.hypot(wx-p[0],wy-p[1])
        k=np.argsort(dd)[:3]; br=[round(math.degrees(math.atan2(wy[i]-p[1],wx[i]-p[0])-p[2])%360,0) for i in k]
        print(f'   t={t-1793104000:.2f} costmap@scan {st-1793104000:.2f} pose yaw {p[2]:.4f} near lethal d={np.round(dd[k],2)} brg={br} : {out}')
    if mode=='baseline': np.save('t03_replay_4349.npy',L.g)
