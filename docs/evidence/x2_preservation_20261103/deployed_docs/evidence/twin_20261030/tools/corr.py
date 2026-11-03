import sys, math, bisect
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
am=[];mo=[];wz=[];lag={"/qazbot_raw/scan":[], "/qazbot_lio/scan":[], "/qazbot_raw/scan_low":[]}
with open(sys.argv[1],'rb') as f:
    r=make_reader(f, decoder_factories=[DecoderFactory()])
    T0=r.get_summary().statistics.message_start_time
    for s,c,m,msg in r.iter_decoded_messages(topics=["/qazbot_d1_map/amcl_pose","/qazbot_d1_map/tf","/qazbot_native/odom","/qazbot_raw/scan","/qazbot_lio/scan","/qazbot_raw/scan_low"]):
        t=(m.log_time-T0)/1e9
        if c.topic.endswith("amcl_pose"): am.append(t)
        elif c.topic.endswith("/tf"):
            for tr in msg.transforms:
                if tr.child_frame_id=="odom": x=tr.transform.translation; mo.append((t,x.x,x.y))
        elif c.topic=="/qazbot_native/odom": wz.append((t,abs(msg.twist.twist.angular.z)))
        else:
            st=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            lag[c.topic].append((t,(m.log_time/1e9)-st))
mot=[e[0] for e in mo]; wzt=[e[0] for e in wz]
def near(L,T,t):
    i=bisect.bisect_left(T,t); i=min(max(i,0),len(L)-1); return L[i]
rows=[]
for t in am:
    a=near(mo,mot,t-0.3); b=near(mo,mot,t+0.3)
    corr=math.hypot(b[1]-a[1],b[2]-a[2])
    w=np.mean([e[1] for e in wz if t-0.5<=e[0]<=t+0.5] or [0])
    rows.append((t,corr,w))
R=np.array(rows)
def rank(x): return np.argsort(np.argsort(x))
def sp(x,y): return np.corrcoef(rank(x),rank(y))[0,1]
print(f"AMCL updates {len(R)}; correction median {np.median(R[:,1]):.2f} p90 {np.percentile(R[:,1],90):.2f} max {R[:,1].max():.2f} m")
print(f"Spearman(correction, |yaw rate|) = {sp(R[:,1],R[:,2]):.2f}")
turn=R[:,2]>0.3; still=R[:,2]<0.1
print(f"|wz|>0.3 rad/s: n={turn.sum()} median corr {np.median(R[turn,1]) if turn.any() else float('nan'):.2f} m ; |wz|<0.1: n={still.sum()} median corr {np.median(R[still,1]) if still.any() else float('nan'):.2f} m")
for k,L in lag.items():
    v=np.array([e[1] for e in L]); 
    if len(v): print(f"{k} transport lag (recv-stamp) p50 {np.median(v):.2f} p95 {np.percentile(v,95):.2f} max {v.max():.2f} s")
# predicted lateral error at 5 m = wz*lag*5
wmax=np.percentile(R[turn,2],50) if turn.any() else 0
print(f"median |wz| during turns {wmax:.2f} rad/s -> at lag 0.2 s, 5 m range: {wmax*0.2*5:.2f} m lateral")
