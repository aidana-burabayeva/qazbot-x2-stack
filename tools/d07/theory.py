import pickle, numpy as np, sys
D=pickle.load(open('b183.pkl','rb')); dirs=D['dirs']
n0=D['plane0'][:3]; d0=D['plane0'][3]
xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
# beam set: directions of first empty-window frame (returns only); check repetition
U0=dirs[0][1].astype(float); U1=dirs[1][1].astype(float)
print('dir frames sods',[round(x[0]) for x in dirs], 'n', len(U0), len(U1))
def lv(U): return np.column_stack((U@f,U@L,U@n0))
V=lv(U0)
el=np.degrees(np.arcsin(V[:,2])); bb=np.degrees(np.arctan2(V[:,1],V[:,0]))
for b0 in (0,20,30,40,45,50,55,58):
    s=np.abs(np.abs(bb)-b0)<1.0 if b0 else np.abs(bb)<1.0
    if s.sum(): print('bearing ±%d: el %.1f..%.1f n=%d'%(b0,el[s].min(),el[s].max(),s.sum()))
# vertical sampling at bearing 0: elevation spacing
s=np.abs(bb)<0.5; e=np.sort(el[s]); print('el spacing at b0 median %.2f deg, p90 %.2f'%(np.median(np.diff(e)),np.percentile(np.diff(e),90)))
W=float(sys.argv[1]) if len(sys.argv)>1 else 0.30
def cast(V,H,r,b,w=W,dep=0.30):
    c,sn=np.cos(np.radians(-b)),np.sin(np.radians(-b))
    ux=c*V[:,0]-sn*V[:,1]; uy=sn*V[:,0]+c*V[:,1]; uz=V[:,2]
    lo=np.array([r,-w/2,0.0]); hi=np.array([r+dep,w/2,H]); o=np.array([0,0,d0])
    with np.errstate(divide='ignore',invalid='ignore'):
        t1=(lo-o)[None,:]/np.column_stack((ux,uy,uz)); t2=(hi-o)[None,:]/np.column_stack((ux,uy,uz))
    tmin=np.nanmax(np.minimum(t1,t2),axis=1); tmax=np.nanmin(np.maximum(t1,t2),axis=1)
    hit=(tmax>=tmin)&(tmin>0)
    tf=np.where(uz<0,-d0/uz,np.inf); hit&=tmin<tf
    hz=d0+tmin*uz; rh=tmin*np.hypot(ux,uy)
    return hz[hit], rh[hit]
rows=[]
for b in (0,30,45,50,55):
    for H in (0.10,0.15,0.21,0.30,0.45):
        line=[]
        for r in (0.3,0.4,0.5,0.6,0.7,0.8,1.0,1.2,1.5,2.0,2.5,3.0):
            hz,rh=cast(V,H,r,b)
            n=len(hz); top=hz.max() if n else 0
            n05=int((hz>0.05).sum()); n10=int((hz>0.10).sum())
            band=int((hz>0.329+0.035*rh).sum())
            rows.append(dict(b=b,H=H,r=r,n=n,top=float(top),n05=n05,n10=n10,band=band))
            line.append('%d/%d/%d/%d'%(n,n05,n10,band))
        print('b=%2d H=%.2f '%(b,H)+' '.join(line))
print('cols r=',(0.3,0.4,0.5,0.6,0.7,0.8,1.0,1.2,1.5,2.0,2.5,3.0),' cell = raw hits / h>0.05 / h>0.10 / above live band (raw, before 1/3 decimation)')
pickle.dump(rows,open('theory_%g.pkl'%W,'wb'))
