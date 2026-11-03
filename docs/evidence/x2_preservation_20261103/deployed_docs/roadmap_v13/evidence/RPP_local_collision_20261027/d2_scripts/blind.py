import bagio, glob, numpy as np, math, sys
from sensor_msgs_py import point_cloud2 as pc2
B=glob.glob('/home/agi/control_backend/logs/raw_comparison_input_01/*.db3')[0]
R_imu_lidar=np.array([[-0.003843965609489519,-0.9999545839883389,0.008720888091315271],[0.0008810920694298785,0.008717562353234953,0.9999616131549165],[-0.9999922237723444,0.0038515019571126463,0.0008475420438577297]])
acc=[];clouds=[];fields=None
for n,t,m in bagio.read(B,['/aima/hal/sensor/lidar_chest_front/imu','/aima/hal/sensor/lidar_chest_front/lidar_pointcloud','/tf_static']):
    if n.endswith('imu'): acc.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
    elif n=='/tf_static':
        for tr in m.transforms:
            if 'lidar' in tr.child_frame_id or 'lidar' in tr.header.frame_id or tr.child_frame_id in('base_link','torso_link'):
                print('tfs',tr.header.frame_id,tr.child_frame_id,round(tr.transform.translation.x,3),round(tr.transform.translation.y,3),round(tr.transform.translation.z,3))
    else:
        if fields is None: fields=[f.name for f in m.fields]; print('fields',fields,m.header.frame_id,m.width,m.height)
        if len(clouds)<40: clouds.append(m)
acc=np.array(acc); a=acc.mean(0); print('acc mean',a,'std',acc.std(0),'|a|',np.linalg.norm(a))
u=R_imu_lidar.T@a; u/=np.linalg.norm(u); print('up in lidar frame',u)
fwd=np.array([1,0,0.]); fwd=fwd-(fwd@u)*u; fwd/=np.linalg.norm(fwd); left=np.cross(u,fwd)
print('lidar x pitch vs horizontal (deg)', math.degrees(math.asin(np.array([1,0,0])@u)))
P=[]
for m in clouds:
    names=['x','y','z']+(['ring'] if 'ring' in fields else [])
    a2=pc2.read_points(m,field_names=tuple(names),skip_nans=True); P.append(np.stack([a2[k].astype(float) for k in names],1))
P=np.vstack(P).astype(float)
X=P[:,:3]; ok=np.linalg.norm(X,axis=1)>0.1; P=P[ok]; X=X[ok]
h=X@u; xf=X@fwd; yl=X@left; r=np.hypot(xf,yl); el=np.degrees(np.arctan2(h,r)); az=np.degrees(np.arctan2(yl,xf))
print('N',len(X),'az range',np.percentile(az,[0.5,99.5]),'el range',np.percentile(el,[0.1,1,50,99,99.9]))
hist,edges=np.histogram(h,bins=np.arange(-2,0.5,0.02)); i=np.argmax(hist[:80]); print('floor h peak',edges[i],edges[i+1],hist[i])
H=-(edges[i]+0.01)
fl=np.abs(h+H)<0.06
print('floor pts',fl.sum(),'nearest floor horiz range pct',np.percentile(r[fl],[0,1,5,50]))
# lowest elevation per azimuth band
for a0 in range(-50,51,10):
    s=(az>=a0-5)&(az<a0+5)
    if s.sum(): print('az',a0,'min el %.2f'%np.percentile(el[s],0.2),'nearest floor r %.2f'%(np.percentile(r[s&fl],0.5) if (s&fl).sum() else -1))
if 'ring' in fields:
    rings=np.unique(P[:,3]); print('rings',len(rings))
    els=[np.median(el[P[:,3]==k]) for k in rings]; print('ring el median sorted',np.round(sorted(els)[:6],2),'...',np.round(sorted(els)[-3:],2))
np.save(sys.argv[1] if len(sys.argv)>1 else 'geo.npy',np.array([H,np.percentile(el,0.2)]))
# density of beams in front (|az|<20) by elevation band, per cloud
nc=len(clouds); s=np.abs(az)<20
for e0 in range(-64,16,8):
    k=s&(el>=e0)&(el<e0+8); print('el [%d,%d) pts/cloud %.0f  ring count %d'%(e0,e0+8,k.sum()/nc,len(np.unique(P[k,3])) if k.sum() else 0))
# lowest-beam height at horizontal range r (front): H - r*tan(depression)
d=-np.percentile(el[s],0.2)
for rr in [0.45,0.5,0.6,0.8,1.0,1.5]: print('r=%.2f lowest beam height above floor %.2f ; slice lower bound %.2f'%(rr,H-rr*math.tan(math.radians(d)),H-0.65+0.035*rr))
# self points: anything with r<0.6 above floor+0.1
k=(r<0.6)&(h>-H+0.1); print('near non-floor pts per cloud', k.sum()/nc, 'h pct', np.percentile(h[k]+H,[5,50,95]) if k.sum() else None)
