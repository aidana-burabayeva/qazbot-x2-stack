"""R3 raw E1R extraction (read-only bag reader). Sensor-frame floor plane fit per frame."""
import sys, time, datetime as dt, pickle
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG=sys.argv[1]; OUT=sys.argv[2]; T=sys.argv[3] if len(sys.argv)>3 else '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
FULLWIN=[tuple(map(float,w.split(':'))) for w in (sys.argv[4].split(',') if len(sys.argv)>4 and sys.argv[4] else [])]
pl0=np.load('/tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/r3/plane0.npy')
rd=rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='mcap' if BAG.endswith('mcap') else 'sqlite3'),rosbag2_py.ConverterOptions('',''))
rd.set_filter(rosbag2_py.StorageFilter(topics=[T]))
C=get_message('sensor_msgs/msg/PointCloud2')
DT=np.dtype({'names':['x','y','z','intensity','ring'],'formats':['<f4','<f4','<f4','<f4','<u2'],'offsets':[0,4,8,16,20],'itemsize':32})
def sod(ts):
    d=dt.datetime.fromtimestamp(ts/1e9); return d.hour*3600+d.minute*60+d.second+d.microsecond*1e-6
def fit(P,n,d):
    for it in range(3):
        h=P@n+d; rh=np.linalg.norm(P-np.outer(P@n,n),axis=1)
        s=(np.abs(h)<(0.06 if it==0 else max(0.015,3*sd)))&(rh>0.45)&(rh<3.0)
        Q=P[s]
        if len(Q)<200: return None
        c=Q.mean(0); u,sv,vt=np.linalg.svd(Q-c,full_matrices=False); nn=vt[2]
        if nn@n<0: nn=-nn
        n,d=nn,-nn@c; sd=float(np.std(Q@n+d))
    return n,d,sd,int(s.sum())
meta=[]; pts=[]; full={}; dirs=[]
n_g,d_g=pl0[:3],pl0[3]
t0=time.time(); k=0
RB=[0.45,0.6,0.8,1.0,1.25,1.5,2.0,2.5,3.0]
while rd.has_next():
    name,data,ts=rd.read_next(); m=deserialize_message(data,C); k+=1
    a=np.frombuffer(bytes(m.data),dtype=DT,count=m.width*m.height)
    P=np.column_stack((a['x'],a['y'],a['z'])).astype(np.float64); ring=a['ring']; inten=a['intensity']
    r3=np.linalg.norm(P,axis=1)
    f=fit(P,n_g,d_g)
    hg=P@n_g+d_g; ph=P-np.outer(P@n_g,n_g); rh=np.linalg.norm(ph,axis=1)
    row=dict(ts=ts,sod=sod(ts),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,n=len(P),r3min=float(r3.min()),
             n_r3_lt=[int((r3<v).sum()) for v in (0.3,0.5,0.7,0.8,1.0)])
    if f is not None:
        n,d,sd,ni=f; row.update(n=n.tolist(),d=d,sd=sd,ni=ni) if False else row.update(nf=n,df=d,sdf=sd,nif=ni)
        hf=P@n+d
        fl=(np.abs(hf)<0.08)
        row['fl']=[]
        for i in range(len(RB)-1):
            s=fl&(rh>=RB[i])&(rh<RB[i+1])
            if s.sum()>20:
                q=hf[s]; row['fl'].append((RB[i],int(s.sum()),float(np.median(q)),float(q.std()),float(np.percentile(q,99)),float(np.percentile(q,99.9)),float(q.max())))
    meta.append(row)
    sel=(hg>0.012)&(rh<3.0)&(hg<1.2)
    pts.append(np.column_stack((P[sel],inten[sel],ring[sel])).astype(np.float32))
    if any(a0<=row['sod']<=a1 for a0,a1 in FULLWIN):
        s2=rh<3.5
        full[k]=np.column_stack((P[s2],inten[s2],ring[s2])).astype(np.float32)
    if k%40==3 and len(dirs)<12:
        dirs.append((row['sod'],(P/r3[:,None]).astype(np.float32),ring.copy()))
    if k%500==0: print(k,f'{time.time()-t0:.0f}s',flush=True)
pickle.dump(dict(meta=meta,pts=pts,full=full,dirs=dirs,plane0=pl0),open(OUT,'wb'),protocol=4)
print('done',k,time.time()-t0)
