import pickle,numpy as np,math,sys,importlib.util
from replay import *
from run1 import run, tf, scans, snap, D
spec=importlib.util.spec_from_file_location('rpp','/home/agi/.orchestrator/reports/20261027-rpp-local-collision-rpp_offline.py'); rpp=importlib.util.module_from_spec(spec); spec.loader.exec_module(rpp)
fp=rpp.footprint()
rep=np.load('rep4349.npy'); sn=snap==254
# pre-bag stale cells (lethal in snapshot, never marked in replay) in odom coords
ys,xs=np.nonzero(sn&(rep!=254)); prebag=[(-5.0+(x+.5)*.05,-2.0+(y+.5)*.05) for x,y in zip(xs,ys)]
def checks(g,t,extra=None,label=''):
    G=g.g.copy()
    if extra:
        for wx,wy in extra:
            m=g.w2m(wx,wy)
            if m: G[m[1],m[0]]=254
    cm=rpp.Costmap(G.ravel(),g.ox,g.oy)
    p=interp_pose(tf,t); out=[]
    for v,w in [(0.3,-0.129),(0.3,0.0),(0.3,-0.3),(0.3,0.129),(0.0,-0.6),(0.0,0.6),(0.15,-0.129)]:
        det=[]; c=rpp.collision_imminent(cm,fp,p[0],p[1],p[2],v,w,0.5,detail=det)
        out.append(('C' if c else '.')+('' if not det else '@%s%d'%(det[0][0][0],det[0][1])))
    ys,xs=np.nonzero(G==254); wx=g.ox+(xs+.5)*.05; wy=g.oy+(ys+.5)*.05
    d=np.hypot(wx-p[0],wy-p[1]); b=(np.degrees(np.arctan2(wy-p[1],wx-p[0])-p[2])+180)%360-180
    near=sorted([(round(dd,2),round(bb)) for dd,bb in zip(d,b) if dd<1.0])
    print(label,round(t-1793104000,3),'pose',np.round(p,3),'lethal',int((G==254).sum()),'<1m:',len(near),near[:8],'checks',out)
g=None
for T in [1793104290.0,1793104301.58,1793104301.79,1793104301.84,1793104301.94,1793104302.04,1793104302.14,1793104302.3]:
    g=run(T)
    checks(g,T,label='replay')
    checks(g,T,extra=prebag,label='replay+prebag')
