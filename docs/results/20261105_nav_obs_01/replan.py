"""Offline emulation of the global plan (inflation re-computation + Smac2D-like grid search) on the real saved costmap.
No ROS, no robot. Validation first against the recorded Smac plan; then one-parameter variants (global inflation radius)."""
import json,math,heapq,sys,numpy as np
from scipy import ndimage
Z=np.load('costmap_box.npz');A=Z['global_data'];R0=float(Z['global_res']);ORG=Z['global_org']
box=json.load(open('boxcheck_box.json'));x0,y0,yw=box['start'];gx,gy,_=box['goal'];REAL=box['plan']['path']
INS=0.17+0.01+0.05/math.sqrt(2);CS=1.5
FOOT=[(0.17,0.229),(0.008,0.34),(-0.182,0.34),(-0.305,0.229),(-0.305,-0.261),(-0.182,-0.372),(0.008,-0.372),(0.17,-0.261)]
lethal=(A==254);unknown=(A==255)
D=ndimage.distance_transform_edt(~lethal)*R0
def inflate(R,cs=CS):
    c=np.zeros(A.shape,dtype=np.float32)
    m=(D>INS)&(D<=R);c[m]=252*np.exp(-cs*(D[m]-INS));c[D<=INS]=253;c[lethal]=254;c[unknown]=255
    return c
def idx(x,y): return int((y-ORG[1])/R0),int((x-ORG[0])/R0)
def plan(C,mult=4.0):
    i0=idx(x0,y0);i1=idx(gx,gy);H,W=C.shape
    # crop around start-goal
    r0=max(0,min(i0[0],i1[0])-60);r1=min(H,max(i0[0],i1[0])+60);c0=max(0,min(i0[1],i1[1])-60);c1=min(W,max(i0[1],i1[1])+60)
    dist={};prev={};pq=[(0.0,i0)];dist[i0]=0.0
    nb=[(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,1.4142),(-1,1,1.4142),(1,-1,1.4142),(1,1,1.4142)]
    while pq:
        d,u=heapq.heappop(pq)
        if d>dist.get(u,1e18): continue
        if u==i1: break
        for dr,dc,l in nb:
            v=(u[0]+dr,u[1]+dc)
            if not(r0<=v[0]<r1 and c0<=v[1]<c1): continue
            cv=C[v]
            if cv>=253: continue
            nd=d+l*(1.0+mult*cv/252.0)
            if nd<dist.get(v,1e18): dist[v]=nd;prev[v]=u;heapq.heappush(pq,(nd,v))
    p=[];u=i1
    while u in prev or u==i0:
        p.append((ORG[0]+(u[1]+0.5)*R0,ORG[1]+(u[0]+0.5)*R0))
        if u==i0: break
        u=prev[u]
    return p[::-1]
ux,uy=math.cos(yw),math.sin(yw);lx,ly=-uy,ux
def loc(p): dx,dy=p[0]-x0,p[1]-y0;return dx*ux+dy*uy,dx*lx+dy*ly
def smooth(p,it=30):  # light smoothing like Smac's: moves points toward neighbours' midpoint (does not cut obstacles because cost is checked)
    q=[list(a) for a in p]
    for _ in range(it):
        for k in range(1,len(q)-1):
            q[k][0]=0.5*q[k][0]+0.25*(q[k-1][0]+q[k+1][0]);q[k][1]=0.5*q[k][1]+0.25*(q[k-1][1]+q[k+1][1])
    return [tuple(a) for a in q]
def metrics(path,boxpoly=None,label=''):
    L=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))
    lat=[loc(p)[1] for p in path];dev=max(lat,key=abs)
    # swept footprint clearance to ALL lethal cells (distance transform) and to a given box polygon
    dd=lambda x,y:D[idx(x,y)]
    mn=9.9;mn_box=9.9
    for k,p in enumerate(path):
        nxt=path[min(k+1,len(path)-1)];th=math.atan2(nxt[1]-p[1],nxt[0]-p[0]) if k<len(path)-1 else yw
        c,s=math.cos(th),math.sin(th)
        pts=[]
        for q in range(len(FOOT)):
            a=FOOT[q];b=FOOT[(q+1)%len(FOOT)]
            for t in (0,0.5):pts.append((a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t))
        for fx,fy in pts:
            X=p[0]+c*fx-s*fy;Y=p[1]+s*fx+c*fy
            mn=min(mn,dd(X,Y)-R0/2)
            if boxpoly:
                dx=max(boxpoly[0]-loc((X,Y))[0],0,loc((X,Y))[0]-boxpoly[1]);dy=max(boxpoly[2]-loc((X,Y))[1],0,loc((X,Y))[1]-boxpoly[3]);mn_box=min(mn_box,math.hypot(dx,dy))
    # max heading change per metre (curvature proxy)
    th=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(path,path[1:])]
    kmax=max(abs(math.remainder(t2-t1,2*math.pi))/R0 for t1,t2 in zip(th,th[1:]))
    return dict(len=L,lateral=dev,foot_to_lethal=mn,foot_to_box=mn_box,kmax=kmax)
if __name__=='__main__':
    C=inflate(0.6)
    # validation of the inflation model near the box: compare re-computed costs with the real ones
    i=idx(-8.62,-4.19);sl=(slice(i[0]-25,i[0]+25),slice(i[1]-25,i[1]+25))
    diff=np.abs(C[sl]-A[sl].astype(np.float32));print('inflation model vs real costmap near the box: mean abs diff %.2f, 95%% %.1f (cost units)'%(diff.mean(),np.percentile(diff,95)))
    pr=plan(C);ps=smooth(pr)
    print('emulated plan R=0.60: lateral %+.2f len %.2f ; recorded Smac: lateral %+.2f len %.2f'%(max((loc(p)[1] for p in pr),key=abs),sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pr,pr[1:])),max((loc(p)[1] for p in REAL),key=abs),box['plan']['length']))
    d=[min(abs(loc(p)[1]-loc(q)[1]) for q in REAL if abs(loc(q)[0]-loc(p)[0])<0.06) for p in ps if 0.2<loc(p)[0]<3.8 and any(abs(loc(q)[0]-loc(p)[0])<0.06 for q in REAL)]
    print('smoothed emulation vs recorded plan (lateral difference at equal along): mean %.3f max %.3f m'%(np.mean(d),np.max(d)))
