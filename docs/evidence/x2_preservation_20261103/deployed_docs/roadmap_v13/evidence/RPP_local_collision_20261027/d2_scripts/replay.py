import sys, math, numpy as np, bisect, pickle, os
sys.path.insert(0,'/home/agi/.orchestrator/reports')
import importlib.util
spec=importlib.util.spec_from_file_location('rpp','/home/agi/.orchestrator/reports/20261027-rpp-local-collision-rpp_offline.py'); rpp=importlib.util.module_from_spec(spec); spec.loader.exec_module(rpp)
EV='/home/agi/control_backend/docs/roadmap_v13/evidence/'
RES=0.05; N=80
def load(bagdir, t0=None, t1=None):
    cache=f'/tmp/claude-1001/-home-agi/85355f79-5e00-4825-8c75-b2bc85299e5c/scratchpad/d2/{bagdir}_{t0}_{t1}.pkl'
    if os.path.exists(cache): return pickle.load(open(cache,'rb'))
    import bagio
    B=EV+bagdir+'/bag/bag_0.db3'
    scans=[]; tf=[]
    for n,t,m in bagio.read(B,['/qazbot_lio/scan','/qazbot_d1_map/tf'],t0,t1):
        if n=='/qazbot_lio/scan':
            scans.append((bagio.st(m.header),t,m.angle_min,m.angle_increment,m.range_min,m.range_max,np.array(m.ranges,dtype=np.float32)))
        else:
            for tr in m.transforms:
                if tr.header.frame_id=='odom' and tr.child_frame_id=='base_footprint':
                    q=tr.transform.rotation; tf.append((bagio.st(tr.header),tr.transform.translation.x,tr.transform.translation.y,bagio.yaw(q),t))
    tf.sort(); out=(scans,tf); pickle.dump(out,open(cache,'wb')); return out
class TF:
    def __init__(s,tf): s.t=[a[0] for a in tf]; s.d=tf
    def at(s,t):
        i=bisect.bisect_left(s.t,t)
        if i<=0: a=s.d[0]; return a[1],a[2],a[3]
        if i>=len(s.t): a=s.d[-1]; return a[1],a[2],a[3]
        a,b=s.d[i-1],s.d[i]; f=(t-a[0])/(b[0]-a[0]) if b[0]>a[0] else 0
        dy=math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
        return a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]),a[3]+f*dy
class Layer:
    """Emulation of Nav2 Humble ObstacleLayer (rolling, track_unknown false): clear (raytrace) then mark, footprint clearing."""
    def __init__(s,ox,oy,grid=None):
        s.ox,s.oy=ox,oy; s.g=np.zeros((N,N),np.uint8) if grid is None else grid.copy()
    def roll(s,rx,ry):
        nx,ny=rx-N*RES/2,ry-N*RES/2
        cx=int((nx-s.ox)/RES); cy=int((ny-s.oy)/RES)
        if cx==0 and cy==0: return
        g=np.zeros_like(s.g)
        sx0,sx1=max(0,cx),min(N,N+cx); sy0,sy1=max(0,cy),min(N,N+cy)
        if sx0<sx1 and sy0<sy1: g[sy0-cy:sy1-cy,sx0-cx:sx1-cx]=s.g[sy0:sy1,sx0:sx1]
        s.g=g; s.ox+=cx*RES; s.oy+=cy*RES
    def cells(s,wx,wy):
        mx=np.floor((wx-s.ox)/RES).astype(int); my=np.floor((wy-s.oy)/RES).astype(int)
        ok=(mx>=0)&(my>=0)&(mx<N)&(my<N); return mx,my,ok
    def update(s,pose,ang,rng,mark_mask,clear_mask,obst_max=3.0,ray_max=3.5,fp_r=0.464):
        x,y,th=pose
        s.roll(x,y)
        # clearing
        a=ang[clear_mask]; r=rng[clear_mask]
        if len(a):
            L=np.minimum(r,ray_max)
            steps=np.arange(0,1.0001,0.005)[None,:]
            d=L[:,None]*steps
            wx=x+d*np.cos(th+a)[:,None]; wy=y+d*np.sin(th+a)[:,None]
            mx,my,ok=s.cells(wx.ravel(),wy.ravel()); s.g[my[ok],mx[ok]]=0
        # marking
        a=ang[mark_mask]; r=rng[mark_mask]; k=r<obst_max; a=a[k]; r=r[k]
        mx,my,ok=s.cells(x+r*np.cos(th+a),y+r*np.sin(th+a)); s.g[my[ok],mx[ok]]=254
        # footprint clearing (circle approx of padded 16-gon)
        ii,jj=np.meshgrid(np.arange(N),np.arange(N))
        cxw=s.ox+(ii+0.5)*RES; cyw=s.oy+(jj+0.5)*RES
        s.g[np.hypot(cxw-x,cyw-y)<fp_r]=0
def scan_rays(sc, mode, fov=(-46.0,44.5)):
    st,rt,amin,inc,rmin,rmax,R=sc
    ang=amin+inc*np.arange(len(R))
    fin=np.isfinite(R)&(R>rmin)&(R<rmax)
    rng=np.where(fin,R,rmax-1e-4).astype(float)
    deg=np.degrees(ang)
    if mode=='baseline': clr=fin.copy()
    elif mode=='inf_all': clr=fin|np.isinf(R)
    elif mode=='inf_fov': clr=fin|(np.isinf(R)&(deg>=fov[0])&(deg<=fov[1]))
    elif mode.startswith('inf_guard'):
        k=int(mode[9:] or 2)
        near=np.convolve(fin.astype(int),np.ones(2*k+1,int),'same')>0
        clr=fin|(np.isinf(R)&~near&(deg>=fov[0])&(deg<=fov[1]))
    return ang,rng,fin,clr
def run(scans,tf,mode,layer,t_from=None,t_to=None,hook=None):
    T=TF(tf)
    for sc in scans:
        if t_from is not None and sc[0]<t_from: continue
        if t_to is not None and sc[0]>t_to: break
        ang,rng,fin,clr=scan_rays(sc,mode)
        layer.update(T.at(sc[0]),ang,rng,fin,clr)
        if hook: hook(sc,layer)
    return layer
def check(layer,pose,v,w,carrot=0.5):
    cm=rpp.Costmap(layer.g.ravel(),layer.ox,layer.oy,RES,N,N); det=[]
    fp=rpp.footprint(0.454,0.01)
    c=rpp.collision_imminent(cm,fp,pose[0],pose[1],pose[2],v,w,carrot,detail=det)
    return c,det
