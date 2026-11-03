import pickle, math
d=pickle.load(open('win.pkl','rb'))
def yaw(q): return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
ca=d['/qazbot_d1_map/cmd_vel_authorized']; mc=d['/aima/mc/locomotion/velocity']; od=d['/qazbot_native/odom']
print('mc keys', list(mc[0][1].keys()))
print('srcs', set(m.get('source') for t,m in mc))
ours=[(t,m) for t,m in mc if 'qazbot' in str(m.get('source'))]
print('ours',len(ours), ours[0][0] if ours else None, ours[-1][0] if ours else None)
print('cmd_auth', len(ca), ca[0][0], ca[-1][0])
# cmd_auth summary every msg in window 1793349878..1793349905
import bisect
ot=[t for t,_ in od]; oy=[yaw(m['pose']['pose']['orientation']) for _,m in od]
def oyaw(t):
    i=min(bisect.bisect_left(ot,t),len(ot)-1); return math.degrees(oy[i])
ourt=[t for t,_ in ours]
for t,m in ca:
    if 1793349878<=t<=1793349905:
        i=bisect.bisect_left(ourt,t); o=ours[min(i,len(ours)-1)][1] if ours else {}
        print(f"{t-1793349800:7.2f} vx={m['linear']['x']:+.3f} vy={m['linear']['y']:+.2f} wz={m['angular']['z']:+.3f} | mc fwd={o.get('forward_velocity',0):+.2f} ang={o.get('angular_velocity',0):+.2f} | odomyaw={oyaw(t):+7.1f}")
