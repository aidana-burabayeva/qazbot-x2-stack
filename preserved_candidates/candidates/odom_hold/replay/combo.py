import pickle,sys,math,numpy as np,collections
sys.path.insert(0,'/home/agi/control_backend/ws/src/lio')
import native_odom as no
def load(b):
    R=pickle.load(open(b+'.pkl','rb'))
    L=R['/aima/mc/leg_odometry']; S=R['/aima/mc/common/state']
    ts=np.array([r[0] for r in L]); x=np.array([r[2] for r in L]); y=np.array([r[3] for r in L])
    yaw=np.unwrap([no.imu_footprint_yaw(r[5:9])[0] for r in L])
    return R,L,S,ts,x,y,yaw
def winstats(ts,x,y,yaw,w):
    j=np.searchsorted(ts,ts-w); dt=np.maximum(ts-ts[j],1e-3)
    return np.hypot(x-x[j],y-y[j])/dt, np.abs(yaw-yaw[j])/dt, j
if __name__=='__main__':
    agg=collections.defaultdict(list)
    for b in sys.argv[1:]:
        R,L,S,ts,x,y,yaw=load(b)
        v,wz,j=winstats(ts,x,y,yaw,0.1)
        sh=np.array([s[1] for s in S]); k=np.clip(np.searchsorted(sh,ts)-1,0,len(S)-1)
        for i in range(len(ts)):
            if i<15: continue
            f=S[k[i]][2]; key=(f['fsm_state.current_state'],f['action_info.action_desc'],f['action_info.status.value'])
            agg[key].append((v[i],wz[i]))
    for key,a in sorted(agg.items(),key=lambda kv:-len(kv[1])):
        a=np.array(a); print(key,len(a),'v p50/p99/max %.4f %.4f %.3f'%(np.median(a[:,0]),np.percentile(a[:,0],99),a[:,0].max()),'wz p50/p99/max %.4f %.4f %.3f'%(np.median(a[:,1]),np.percentile(a[:,1],99),a[:,1].max()))
