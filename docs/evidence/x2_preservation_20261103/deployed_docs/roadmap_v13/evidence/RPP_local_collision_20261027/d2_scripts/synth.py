import replay, numpy as np, math
E=replay.EV+'First_RViz_goal_C3_1205/'
snap=np.load(E+'local_costmap_before_clear.npy').reshape(80,80)
grid=np.where(snap==254,254,0).astype(np.uint8)
scans,tf=replay.load('First_RViz_goal_C3_1205',1793103100,None)
T=replay.TF(tf)
win=[s for s in scans if 1793103171.4<=s[0]<=1793103181.43]
rng=np.random.default_rng(0)
def inject(sc,d,wid,p,bear=0.0):
    st,rt,amin,inc,rmin,rmax,R=sc; R=R.copy()
    ang=amin+inc*np.arange(len(R))
    # flat face perpendicular to bearing `bear`, half width wid/2, at distance d
    rel=ang-math.radians(bear); y=d*np.tan(rel)
    hit=(np.abs(rel)<math.pi/2-0.01)&(np.abs(y)<=wid/2)&(rng.random(len(R))<p)
    r=d/np.cos(rel); R[hit]=np.minimum(R[hit],r[hit]); return (st,rt,amin,inc,rmin,rmax,R)
print('scans in window',len(win))
for d in [0.6,0.8,1.0,1.5]:
  for p in [0.5,0.2,0.05]:
    row=[]
    for mode in ['baseline','inf_fov','inf_guard2']:
        L=replay.Layer(-3.85,-3.6,np.zeros_like(grid)); cnt=[];col=[]
        for k,sc in enumerate(win):
            s2=inject(sc,d,0.4,p)
            ang,rg,fin,clr=replay.scan_rays(s2,mode); pose=T.at(sc[0]); L.update(pose,ang,rg,fin,clr)
            # obstacle cells: within the face
            fx=pose[0]+d*math.cos(pose[2]); fy=pose[1]+d*math.sin(pose[2])
            jj,ii=np.nonzero(L.g==254); wx=L.ox+(ii+.5)*.05; wy=L.oy+(jj+.5)*.05
            near=np.hypot(wx-fx,wy-fy)<0.25; cnt.append(int(near.sum()))
            if k>=2: col.append(replay.check(L,pose,0.3,0.0)[0])
        row.append(f'{mode}: face lethal min/mean/last {min(cnt[2:])}/{np.mean(cnt[2:]):.1f}/{cnt[-1]} RPP-collision {sum(col)}/{len(col)}')
    print(f'd={d} p={p}: '+' | '.join(row))
