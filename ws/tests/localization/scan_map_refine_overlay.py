import numpy as np, math
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
m=np.load('cap.npz'); g=m['grid']; res=float(m['res']); ox=float(m['ox']); oy=float(m['oy']); H,W=g.shape
occ=(g==100); dist=distance_transform_edt(~occ)*res
S=np.load('cap_standing.npz'); R=S['scans']; a=S['amin']+np.arange(R.shape[1])*S['ainc']
r=np.array([np.nanmedian(np.where(np.isfinite(R[:,i]),R[:,i],np.nan)) if np.isfinite(R[:,i]).any() else np.nan for i in range(R.shape[1])])
ok=np.isfinite(r)&(r>0.5)&(r<8.0); r,a=r[ok],a[ok]; bx,by=r*np.cos(a),r*np.sin(a)
def sco(x,y,yaw,sig=0.15):
    c,s=math.cos(yaw),math.sin(yaw); wx=x+c*bx-s*by; wy=y+s*bx+c*by
    ix=((wx-ox)/res).astype(int); iy=((wy-oy)/res).astype(int); inb=(ix>=0)&(ix<W)&(iy>=0)&(iy<H)
    d=np.full(bx.size,2.0); d[inb]=dist[iy[inb],ix[inb]]; return float(np.exp(-d**2/(2*sig**2)).mean()), d
def refine(x0,y0,w0):
    best=(0,0,0,0)
    for dx in np.arange(-.3,.3001,.02):
        for dy in np.arange(-.3,.3001,.02):
            for dw in np.arange(-6,6.01,.5):
                s,_=sco(x0+dx,y0+dy,math.radians(w0+dw))
                if s>best[0]: best=(s,x0+dx,y0+dy,w0+dw)
    return best
res_={}
for name,(x,y,w) in {'A':(0.18,-7.37,357),'B':(-0.02,0.53,186),'C':(-0.52,-0.07,183)}.items():
    b=refine(x,y,w); res_[name]=b; s,d=sco(b[1],b[2],math.radians(b[3]))
    print(name,'refined score %.3f x %.2f y %.2f yaw %.1f'%b,' beams within 0.10 m of an occupied cell: %.0f%%'%(100*(d<0.10).mean()),' within 0.25: %.0f%%'%(100*(d<0.25).mean()))
# sensitivity: score drop for 0.2 m / 5 deg shift at best
s0,x0,y0,w0=res_['A']
for dx,dy,dw in [(0.2,0,0),(0,0.2,0),(0,0,5),(0.5,0,0),(0,0.5,0)]:
    print('shift',dx,dy,dw,'-> score %.3f'%sco(x0+dx,y0+dy,math.radians(w0+dw))[0])
fig,ax=plt.subplots(1,2,figsize=(15,7))
for k,(name,ttl) in enumerate([('A','candidate A'),('B','candidate B')]):
    s,x,y,w=res_[name]; c,sn=math.cos(math.radians(w)),math.sin(math.radians(w))
    ex=(ox+np.arange(W+1)*res); ey=(oy+np.arange(H+1)*res)
    img=np.where(g==100,0,np.where(g==0,1,0.6)).astype(float)
    ax[k].imshow(img,origin='lower',extent=[ox,ox+W*res,oy,oy+H*res],cmap='gray',vmin=0,vmax=1)
    wx=x+c*bx-sn*by; wy=y+sn*bx+c*by; ax[k].scatter(wx,wy,s=4,c='red')
    ax[k].arrow(x,y,0.8*c,0.8*sn,head_width=0.2,color='lime'); ax[k].plot(x,y,'go')
    ax[k].set_xlim(x-6,x+6); ax[k].set_ylim(y-6,y+6); ax[k].set_title('%s  x=%.2f y=%.2f yaw=%.0f score=%.3f'%(name,x,y,w,s)); ax[k].grid(alpha=.3)
plt.tight_layout(); plt.savefig('overlay.png',dpi=80)
