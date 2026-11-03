import replay, numpy as np, math
E=replay.EV+'First_RViz_goal_C3_1205/'
snap=np.load(E+'local_costmap_before_clear.npy').reshape(80,80)
grid=np.where(snap==254,254,0).astype(np.uint8)
scans,tf=replay.load('First_RViz_goal_C3_1205',1793103100,None)
T=replay.TF(tf); pose=T.at(1793103181.43); print('pose',pose)
def lethal_near(L,pose,R=1.6):
    jj,ii=np.nonzero(L.g==254); wx=L.ox+(ii+.5)*.05; wy=L.oy+(jj+.5)*.05
    d=np.hypot(wx-pose[0],wy-pose[1]); return int((L.g==254).sum()), int((d<R).sum())
for mode in ['baseline','inf_fov','inf_guard2','inf_guard4','inf_all']:
    L=replay.Layer(-3.85,-3.6,grid)
    replay.run(scans,tf,mode,L,1793103176.0,1793103181.43)
    res=[]
    for v,w in [(0.3,0),(0.3,0.05),(0.3,-0.05),(0.3,0.1),(0.3,-0.1),(0.0,0.6),(0.0,-0.6)]:
        c,d=replay.check(L,pose,v,w); res.append(('C' if c else '-')+(f"@{d[0][0]}{d[0][1]}" if c else ''))
    print(f'{mode:9s} lethal total/within1.6m {lethal_near(L,pose)} checks v0.3 w0,+.05,-.05,+.1,-.1, rot+,rot-: {res}')
L=replay.Layer(-3.85,-3.6,grid); print('snapshot itself', lethal_near(L,pose), replay.check(L,pose,0.3,0)[0])
