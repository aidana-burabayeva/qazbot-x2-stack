import sys, json, math
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
a,b=float(sys.argv[2]),float(sys.argv[3])
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
with open(sys.argv[1],'rb') as f:
    r=make_reader(f, decoder_factories=[DecoderFactory()])
    T0=r.get_summary().statistics.message_start_time
    n=0; prev=None; amcl=[]; mo=[]; odom=[]
    for s,c,m,msg in r.iter_decoded_messages(topics=["/qazbot_d1_map/localization_trust/status","/qazbot_d1_map/amcl_pose","/qazbot_d1_map/tf","/qazbot_native/odom"]):
        t=(m.log_time-T0)/1e9
        if not(a<=t<=b): continue
        if c.topic.endswith("trust/status"):
            d=json.loads(msg.data)
            if n==0: print("KEYS",list(d.keys())); n=1
            key=json.dumps({k:d.get(k) for k in ("state","ready","t5","reanchor","reason","reasons","paused","trusted") if k in d},sort_keys=True)
            if key!=prev: print(f"{t:7.1f}s TRUST {key}"); prev=key
        elif c.topic.endswith("amcl_pose"):
            p=msg.pose.pose; cv=msg.pose.covariance
            print(f"{t:7.1f}s AMCL ({p.position.x:.2f},{p.position.y:.2f}) yaw={math.degrees(yaw(p.orientation)):.0f} sxy={math.sqrt(max(cv[0],0)):.2f}")
        elif c.topic.endswith("/tf"):
            for tr in msg.transforms:
                if tr.child_frame_id=="odom" or (tr.header.frame_id=="map" and tr.child_frame_id.startswith("odom")):
                    x=tr.transform.translation; mo.append((t,x.x,x.y,yaw(tr.transform.rotation)))
    print("map->odom samples",len(mo))
    for i in range(1,len(mo)):
        d=math.hypot(mo[i][1]-mo[i-1][1],mo[i][2]-mo[i-1][2])
        if d>0.12: print(f"{mo[i][0]:7.1f}s map->odom JUMP {d:.2f} m  dyaw={math.degrees(mo[i][3]-mo[i-1][3]):.1f}deg dt={mo[i][0]-mo[i-1][0]:.2f}s")
