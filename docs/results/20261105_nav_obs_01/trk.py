"""Indicative tracking simulation (NOT a measurement): pure-pursuit on the planned path with D-05 plant (cmd 0.30 -> real 0.50 m/s,
yaw rate = commanded wz, 0.4 s dead time) + random heading drift calibrated on the straight runs (max cross-track 0.18-0.36 m)."""
import json,math,random,sys,numpy as np
FOOT=None
src=open('/home/agi/control_backend/ws/src/navigation/nav_control.launch.py').read()
import re
m=re.search(r"X2_FOOTPRINT = (\[.*?\])\n",src,re.S);FOOT=eval(m.group(1))
d=json.load(open(sys.argv[1]));x0,y0,yw=d['start'];P=d['plan']['path'] if 'plan' in d else None
ux,uy=math.cos(yw),math.sin(yw);lx,ly=-uy,ux
def to_local(p): dx,dy=p[0]-x0,p[1]-y0;return (dx*ux+dy*uy,dx*lx+dy*ly)
if P is None: P=[(x0+s*ux,y0+s*uy) for s in np.arange(0,4.01,0.05)]
PL=[to_local(p) for p in P]   # (along, lateral) in the start frame
def run(seed,drift,box,lag=0.4,vreal=0.5,vcmd=0.3,L=0.5,dt=0.05,T=14):
    r=random.Random(seed);x,y,th=0.0,0.0,0.0;buf=[(0.0,0.0)]*int(lag/dt);bias=r.gauss(0,0.05);minc=9;maxcross=0
    for k in range(int(T/dt)):
        # nearest path index, lookahead point
        i=min(range(len(PL)),key=lambda j:(PL[j][0]-x)**2+(PL[j][1]-y)**2)
        s=i;acc=0
        while s+1<len(PL) and acc<L: acc+=math.hypot(PL[s+1][0]-PL[s][0],PL[s+1][1]-PL[s][1]);s+=1
        cx,cy=PL[s];dxl=(cx-x)*math.cos(th)+(cy-y)*math.sin(th);dyl=-(cx-x)*math.sin(th)+(cy-y)*math.cos(th)
        Ld=max(math.hypot(dxl,dyl),1e-3);kappa=2*dyl/(Ld*Ld)
        remaining=PL[-1][0]-x
        v=vcmd if remaining>0.1 else 0.0
        wz=max(-0.6,min(0.6,v*kappa))
        buf.append((v,wz));vc,wc=buf.pop(0)
        va=vreal if vc>0.1 else 0.0
        bias+=r.gauss(0,drift*math.sqrt(dt))   # heading drift (rad), random walk
        th+=(wc+0.0)*dt+bias*dt*0.0+r.gauss(0,drift*math.sqrt(dt))
        x+=va*math.cos(th)*dt;y+=va*math.sin(th)*dt
        # cross-track to path
        cross=min(abs(PL[j][1]-y) for j in range(max(0,i-5),min(len(PL),i+6)))
        maxcross=max(maxcross,cross)
        if box:
            c,s_=math.cos(th),math.sin(th)
            for fx,fy in FOOT:
                px,py=x+c*fx-s_*fy,y+s_*fx+c*fy
                # signed clearance to rectangle box [a0,a1]x[l0,l1]
                dx=max(box[0]-px,0,px-box[1]);dy=max(box[2]-py,0,py-box[3]);dd=math.hypot(dx,dy)
                inside=(box[0]<=px<=box[1] and box[2]<=py<=box[3])
                minc=min(minc,-0.01 if inside else dd)
            # also edge crossing test (polygon edge vs box): sample edges
            for q in range(len(FOOT)):
                a=FOOT[q];b=FOOT[(q+1)%len(FOOT)]
                for t in (0.25,0.5,0.75):
                    fx=a[0]+(b[0]-a[0])*t;fy=a[1]+(b[1]-a[1])*t;px,py=x+c*fx-s_*fy,y+s_*fx+c*fy
                    dx=max(box[0]-px,0,px-box[1]);dy=max(box[2]-py,0,py-box[3]);inside=(box[0]<=px<=box[1] and box[2]<=py<=box[3])
                    minc=min(minc,-0.01 if inside else math.hypot(dx,dy))
        if x>PL[-1][0]+0.5: break
    return minc,maxcross
mode=sys.argv[2]
if mode=='calib':
    for drift in (0.2,0.3,0.4,0.55):
        mc=[run(s,drift,None)[1] for s in range(100)]
        print('drift %.2f: max cross-track mean %.2f p90 %.2f max %.2f (target 0.18-0.36)'%(drift,np.mean(mc),np.percentile(mc,90),max(mc)))
else:
    drift=float(sys.argv[3])
    # box in the start frame: observed face centre along 2.04, lateral -0.09; width W (lateral), depth D (along, behind the face)
    for name,(W,Dd) in {'observed 0.20 wide x 0.40 deep':(0.20,0.40),'0.40 x 0.40':(0.40,0.40)}.items():
        box=(2.04,2.04+Dd,-0.09-W/2,-0.09+W/2)
        res=[run(s,drift,box) for s in range(300)]
        mcs=[r[0] for r in res]
        print('%-32s min footprint clearance: min %.2f p5 %.2f median %.2f ; contact in %d/300 runs ; max cross-track mean %.2f'%(name,min(mcs),np.percentile(mcs,5),np.median(mcs),sum(1 for c in mcs if c<=0),np.mean([r[1] for r in res])))
