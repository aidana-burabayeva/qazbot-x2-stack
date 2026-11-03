"""Offline (pure python, no ROS) slow-displacement check: bag 183504 standing segment, injected world-x ramp (starts 15 s into segment)."""
import sys, pickle, math, numpy as np
sys.path.insert(0,'/home/agi/control_backend/ws/src/lio'); sys.path.insert(0,'/home/agi/control_backend/candidates/odom_hold')
import native_odom as no, odom_hold as oh
B='/tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/hold/nav_bag_183504'
R=pickle.load(open(B+'.pkl','rb')); L=R['/aima/mc/leg_odometry']; S=R['/aima/mc/common/state']
def run(rate, seg0=20.0, dur=60.0, rs=15.0):
    t0=L[0][1]+seg0
    ev=[(r[1],0,i) for i,r in enumerate(L) if t0<=r[1]<=t0+dur]+[(s[0],-1,i) for i,s in enumerate(S) if t0<=s[0]<=t0+dur]
    ev.sort(); g=no.native_guard(); H=oh.StandingHold(); out=[]
    for rt,k,i in ev:
        if k==-1:
            f=S[i][2]; H.set_state(rt,f['fsm_state.current_state'],f['action_info.action_desc'],f['action_info.status.value'],f['body_status.value']); continue
        r=L[i]; st=g.add(r[0],r[15],r[16],(r[2],r[3],r[4],r[5],r[6],r[7],r[8]),rt)
        if st=='stamp_back': H.clear_window()
        if st not in('ok','stamp_back'): continue
        _,x,y,z,yaw=g.pose; rel=rt-t0; x+=rate*1e-3*max(0.0,rel-rs)
        px,py,held=H.update(r[0],rt,x,y,yaw); out.append((rel,x,y,px,py,held,H.capped))
    return np.array(out,float),H
print('rate  | first cap release (s since ep start, since ramp) | hidden at release mm | budget at release mm | hidden max after ramp start mm | hidden end mm | entries/exits | held samples after release | injected at end mm | dur')
for dur in (60.0,150.0):
  for rate in (0,0.5,1,2,5):
    O,H=run(rate,dur=dur); t=O[:,0]; hid=np.hypot(O[:,1]-O[:,3],O[:,2]-O[:,4])*1e3
    cap=np.where(O[:,6]>0)[0]; i0=np.searchsorted(t,15.0)
    if len(cap):
        c=cap[0]; tr=t[c]; hr=hid[c-1]; bud=(oh.DRIFT_RATE_MAX_MPS*(tr-t[0])+oh.DRIFT_SLACK_M)*1e3; after=int(O[c:,5].sum())
        rel=f'{tr-t[0]:.1f}s / ramp+{tr-15:.1f}s'
    else: rel='none'; hr=bud=float('nan'); after=-1
    print(f'{rate} mm/s dur {dur:.0f}: release {rel} | hidden@release {hr:.2f} | budget {bud:.2f} | max hidden (all) {hid.max():.2f} | end {hid[-1]:.2f} | {H.counts} | held after release {after} | injected {rate*(t[-1]-15):.1f}')
print('--- hidden component along the injected direction (x), vs control (drift only)')
for dur in (60.0,150.0):
    C,_=run(0,dur=dur); cx=(C[-1,1]-C[-1,3])*1e3
    for rate in (0.5,1,2,5):
        O,_=run(rate,dur=dur); hx=(O[-1,1]-O[-1,3])*1e3; hy=(O[-1,2]-O[-1,4])*1e3
        print(f'dur {dur:.0f} rate {rate}: hidden x {hx:.2f} mm (control x {cx:.2f}), y {hy:.2f}; injected {rate*(dur-15):.1f} mm; hidden injected-part {hx-cx:.2f} mm = {100*(hx-cx)/(rate*(dur-15)):.1f}%')
