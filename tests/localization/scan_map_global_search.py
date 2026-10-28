import numpy as np, math
from scipy.ndimage import distance_transform_edt
m=np.load('cap.npz'); g=m['grid']; res=float(m['res']); ox=float(m['ox']); oy=float(m['oy'])
S=np.load('cap_standing.npz'); R=S['scans']; a=S['amin']+np.arange(R.shape[1])*S['ainc']
occ=(g==100); dist=distance_transform_edt(~occ)*res; H,W=g.shape
# union of rays over the 10 stationary scans, one per bearing bin (median of finite)
r=np.array([np.nanmedian(np.where(np.isfinite(R[:,i]),R[:,i],np.nan)) if np.isfinite(R[:,i]).any() else np.nan for i in range(R.shape[1])])
ok=np.isfinite(r)&(r>0.5)&(r<8.0); r,a=r[ok],a[ok]; bx,by=r*np.cos(a),r*np.sin(a); print('beams',r.size,'range med',np.median(r).round(2),'max',r.max().round(2))
sig=0.15
yaws=np.radians(np.arange(0,360,3)); C,Sn=np.cos(yaws),np.sin(yaws)
free_clear=(g==0)&(dist>0.3)   # robot stands in known free space (g==0 free)
rows,cols=np.nonzero(free_clear[::2,::2]); rows*=2; cols*=2   # 0.1 m stride
print('candidate positions',rows.size)
best=np.zeros(rows.size); besty=np.zeros(rows.size)
for s in range(0,rows.size,300):
    px=ox+(cols[s:s+300]+.5)*res; py=oy+(rows[s:s+300]+.5)*res
    wx=px[:,None,None]+C[None,:,None]*bx[None,None,:]-Sn[None,:,None]*by[None,None,:]
    wy=py[:,None,None]+Sn[None,:,None]*bx[None,None,:]+C[None,:,None]*by[None,None,:]
    ix=((wx-ox)/res).astype(int); iy=((wy-oy)/res).astype(int)
    inb=(ix>=0)&(ix<W)&(iy>=0)&(iy<H); d=np.full(ix.shape,2.0,dtype=np.float32); d[inb]=dist[iy[inb],ix[inb]]
    sc=np.exp(-d**2/(2*sig**2)).mean(axis=2); k=sc.argmax(axis=1); best[s:s+300]=sc[np.arange(sc.shape[0]),k]; besty[s:s+300]=k
order=np.argsort(-best); peaks=[]
for i in order:
    x=ox+(cols[i]+.5)*res; y=oy+(rows[i]+.5)*res; w=math.degrees(yaws[int(besty[i])])
    if all(math.hypot(x-px,y-py)>0.6 for _,px,py,_ in peaks): peaks.append((best[i],x,y,w))
    if len(peaks)>=6: break
for p in peaks: print('peak score %.3f  x %.2f  y %.2f  yaw %.0f'%p)
def sco(x,y,yaw):
    c,s_=math.cos(yaw),math.sin(yaw); wx=x+c*bx-s_*by; wy=y+s_*bx+c*by
    ix=((wx-ox)/res).astype(int); iy=((wy-oy)/res).astype(int); inb=(ix>=0)&(ix<W)&(iy>=0)&(iy<H)
    d=np.full(bx.size,2.0); d[inb]=dist[iy[inb],ix[inb]]; return float(np.exp(-d**2/(2*sig**2)).mean())
print('operator hypothesis (1,-8,52.5):',round(sco(1,-8,math.radians(52.5)),3))
