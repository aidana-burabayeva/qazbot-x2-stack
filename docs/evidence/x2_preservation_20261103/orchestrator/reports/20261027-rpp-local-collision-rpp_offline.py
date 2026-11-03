# Offline re-implementation of Nav2 Humble 1.1.18 RPP collision check
# (RegulatedPurePursuitController::isCollisionImminent / inCollision and
#  FootprintCollisionChecker::footprintCostAtPose / lineCost / nav2_util::LineIterator)
import math, numpy as np
LETHAL=254; INSCRIBED=253; NOINFO=255
def sign0(x): return -1.0 if x<0 else (1.0 if x>0 else 0.0)
def footprint(radius=0.454, padding=0.01):
    pts=[]
    for i in range(16):
        a=i*2*math.pi/16
        x=math.cos(a)*radius; y=math.sin(a)*radius
        pts.append((x+sign0(x)*padding, y+sign0(y)*padding))
    return pts
def circumscribed(fp): return max(math.hypot(*p) for p in fp)
def inscribed(fp):
    # nav2 calculateMinAndMaxDistances: min over vertices and edge distances
    mn=1e9
    for i in range(len(fp)):
        p=fp[i]; q=fp[(i+1)%len(fp)]
        mn=min(mn, math.hypot(*p))
        # dist from origin to segment
        dx,dy=q[0]-p[0],q[1]-p[1]; t=max(0,min(1,-(p[0]*dx+p[1]*dy)/(dx*dx+dy*dy)))
        mn=min(mn, math.hypot(p[0]+t*dx,p[1]+t*dy))
    return mn
def line_iter(x0,y0,x1,y1):
    # nav2_util::LineIterator (Bresenham)
    dx=abs(x1-x0); dy=abs(y1-y0)
    xinc1=xinc2=1 if x1>=x0 else -1
    yinc1=yinc2=1 if y1>=y0 else -1
    if dx>=dy:
        xinc1=0; yinc2=0; den=dx; num=dx/2; numadd=dy; numpixels=dx
    else:
        xinc2=0; yinc1=0; den=dy; num=dy/2; numadd=dx; numpixels=dy
    x,y=x0,y0
    for _ in range(numpixels+1):
        yield x,y
        num+=numadd
        if num>=den:
            num-=den; x+=xinc1; y+=yinc1
        x+=xinc2; y+=yinc2
class Costmap:
    def __init__(self,data,ox,oy,res=0.05,nx=80,ny=80):
        self.d=np.asarray(data,dtype=np.uint8).reshape(ny,nx); self.ox=ox; self.oy=oy; self.res=res; self.nx=nx; self.ny=ny
    def w2m(self,wx,wy):
        if wx<self.ox or wy<self.oy: return None
        mx=int((wx-self.ox)/self.res); my=int((wy-self.oy)/self.res)
        if mx<self.nx and my<self.ny: return mx,my
        return None
    def m2w(self,mx,my): return self.ox+(mx+0.5)*self.res, self.oy+(my+0.5)*self.res
    def cost(self,mx,my): return int(self.d[my,mx])
def footprint_cost(cm,fp,x,y,th,hits=None):
    c,s=math.cos(th),math.sin(th)
    pts=[(x+c*px-s*py, y+s*px+c*py) for px,py in fp]
    cells=[]
    for p in pts:
        m=cm.w2m(*p)
        if m is None: return LETHAL
        cells.append(m)
    best=0
    n=len(cells)
    for i in range(n):
        (x0,y0),(x1,y1)=cells[i],cells[(i+1)%n]
        for mx,my in line_iter(x0,y0,x1,y1):
            v=cm.cost(mx,my)
            if v==LETHAL:
                if hits is not None: hits.append((mx,my))
                return LETHAL
            best=max(best,v)
    return best
def in_collision(cm,fp,x,y,th,track_unknown=False,hits=None):
    if cm.w2m(x,y) is None: return False
    fc=footprint_cost(cm,fp,x,y,th,hits)
    if fc==NOINFO and track_unknown: return False
    return fc>=LETHAL
def collision_imminent(cm,fp,x,y,th,v,w,carrot_dist,tmax=1.5,res=0.05,detail=None):
    hits=[]
    if in_collision(cm,fp,x,y,th,hits=hits):
        if detail is not None: detail.append(('current',0,x,y,th,hits))
        return True
    if abs(v)<0.01 and abs(w)>0.01:
        pt=2.0*math.sin((res/2)/circumscribed(fp))/abs(w)
    else:
        pt=res/abs(v) if abs(v)>0 else 1e9
    cx,cy,ct=x,y,th; i=1
    while i*pt<tmax:
        i+=1
        cx+=pt*v*math.cos(ct); cy+=pt*v*math.sin(ct); ct+=pt*w
        if math.hypot(cx-x,cy-y)>carrot_dist: break
        hits=[]
        if in_collision(cm,fp,cx,cy,ct,hits=hits):
            if detail is not None: detail.append(('proj',i-1,cx,cy,ct,hits))
            return True
    return False
