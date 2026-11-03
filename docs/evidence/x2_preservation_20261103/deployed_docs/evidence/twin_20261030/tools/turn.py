import sys, math
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
a,b=float(sys.argv[2]),float(sys.argv[3])
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
od=[];leg=[];am=[]
with open(sys.argv[1],'rb') as f:
    r=make_reader(f, decoder_factories=[DecoderFactory()])
    T0=r.get_summary().statistics.message_start_time
    for s,c,m,msg in r.iter_decoded_messages(topics=["/qazbot_native/odom","/aima/mc/leg_odometry","/qazbot_d1_map/amcl_pose"]):
        t=(m.log_time-T0)/1e9
        if not(a<=t<=b): continue
        if c.topic=="/qazbot_native/odom":
            p=msg.pose.pose; od.append((t,p.position.x,p.position.y,yaw(p.orientation)))
        elif c.topic=="/aima/mc/leg_odometry":
            p=msg.pose.pose; leg.append((t,p.position.x,p.position.y,yaw(p.orientation)))
        else:
            p=msg.pose.pose; am.append((t,p.position.x,p.position.y,yaw(p.orientation)))
def at(L,t): return min(L,key=lambda e:abs(e[0]-t))
print("t      native_odom(x,y,yaw)        leg_odom(x,y,yaw)          AMCL(x,y,yaw)")
for t in [a+i*0.8 for i in range(int((b-a)/0.8)+1)]:
    o=at(od,t); l=at(leg,t); m_=at(am,t)
    print(f"{t:6.1f} ({o[1]:6.2f},{o[2]:6.2f},{math.degrees(o[3]):5.0f})  ({l[1]:6.2f},{l[2]:6.2f},{math.degrees(l[3]):5.0f})  ({m_[1]:6.2f},{m_[2]:6.2f},{math.degrees(m_[3]):5.0f})")
def span(L,a,b):
    xs=[e for e in L if a<=e[0]<=b]; 
    import itertools
    d=sum(math.hypot(xs[i][1]-xs[i-1][1],xs[i][2]-xs[i-1][2]) for i in range(1,len(xs)))
    return math.hypot(xs[-1][1]-xs[0][1],xs[-1][2]-xs[0][2]), d
for n,L in (("native",od),("leg",leg),("amcl",am)):
    net,path=span(L,162.5,171.5); print(f"{n}: net displacement {net:.2f} m, path length {path:.2f} m in 162.5-171.5 s")
