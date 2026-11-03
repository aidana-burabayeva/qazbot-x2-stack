import pickle,numpy as np,math,sys
from replay import *
D=pickle.load(open('trial.pkl','rb')); tf=D['tf']; scans=D['scans']
E='/home/agi/control_backend/docs/roadmap_v13/evidence/Static_map_trial_1230/'
snap=np.load(E+'local_costmap_before_clear.npy').reshape(80,80)
def run(tend, mode='drop', mask=None, start=None, g=None, rmax=3.5, keep=None):
    g=g or ObstacleGrid(rmax=rmax)
    hist=[]
    for sc in scans:
        rt=sc[1]
        if start and rt<start: continue
        if rt>tend: break
        rp=interp_pose(tf,rt)
        g.update_origin(rp[0]-2.0,rp[1]-2.0)
        sp=interp_pose(tf,sc[0])
        cl,mk=scan_points(sc,sp,mode,mask)
        g.scan_update(sp[0],sp[1],cl,mk)
        g.footprint_clear(rp[0],rp[1])
        if keep: keep(g,sc)
    return g
if __name__=='__main__':
    T=1793104349.006
    g=run(T)
    print('replay origin',g.ox,g.oy,'snap origin -5,-2')
    rep=(g.g==254); sn=(snap==254)
    print('replay lethal',rep.sum(),'snap lethal',sn.sum(),'both',(rep&sn).sum(),'snap only',(sn&~rep).sum(),'replay only',(rep&~sn).sum())
    rp=interp_pose(tf,T); print('robot',rp)
    for name,M in [('snap_only',sn&~rep),('replay_only',rep&~sn),('both',rep&sn)]:
        ys,xs=np.nonzero(M)
        wx=g.ox+(xs+.5)*.05; wy=g.oy+(ys+.5)*.05
        d=np.hypot(wx-rp[0],wy-rp[1]); b=np.degrees(np.arctan2(wy-rp[1],wx-rp[0])-rp[2]); b=(b+180)%360-180
        if len(d): print(name,'dist min/med',d.min().round(2),np.median(d).round(2),'bearing hist',np.histogram(b,bins=[-180,-90,-46,0,46,90,180])[0])
    np.save('rep4349.npy',g.g)
