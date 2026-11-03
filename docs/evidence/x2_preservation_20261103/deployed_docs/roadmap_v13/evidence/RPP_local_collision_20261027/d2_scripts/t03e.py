import replay, numpy as np, math
scans,tf=replay.load('Static_map_trial_1230'); T=replay.TF(tf); T0=1793104000; SO=(-5.0,-2.0)
TS=[301.60,301.65,301.70,301.75,301.79,301.84,301.89,301.94,301.99,302.04,302.09,302.14]
x0,y0,_=T.at(scans[0][0])
for mode in ['baseline','inf_fov']:
  L=replay.Layer(SO[0]+round((x0-2-SO[0])/.05)*.05,SO[1]+round((y0-2-SO[1])/.05)*.05); states={}
  def hook(sc,L):
    for t in TS:
        if sc[0]<=T0+t-0.03: states[t]=(L.g.copy(),L.ox,L.oy)
  replay.run(scans,tf,mode,L,None,T0+302.2,hook)
  for inj in [[],[(51,36),(51,37)],[(52,34)],[(51,36)]]:
    res=''
    for t in TS:
        g,ox,oy=states[t]; g=g.copy()
        for i,j in inj:
            mi=int(round((SO[0]+i*.05-ox)/.05)); mj=int(round((SO[1]+j*.05-oy)/.05)); g[mj,mi]=254
        c,d=replay.check(replay.Layer(ox,oy,g),T.at(T0+t),0.3,-0.129); res+='C' if c else '.'
    print(f'{mode:8s} inject {str(inj):22s} cycles {TS[0]}..{TS[-1]} (live: .....CCCCCCC): {res}')
