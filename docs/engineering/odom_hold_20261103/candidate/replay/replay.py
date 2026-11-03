import sys,pickle,math,json,numpy as np
sys.path.insert(0,'/home/agi/control_backend/ws/src/lio'); sys.path.insert(0,'/home/agi/control_backend/candidates/odom_hold')
import native_odom as no, odom_hold as oh
def replay(b, hold=True):
    R=pickle.load(open(b+'.pkl','rb')); L=R['/aima/mc/leg_odometry']; S=R['/aima/mc/common/state']
    ev=[(r[1],0,i) for i,r in enumerate(L)]+[(s[0],-1,i) for i,s in enumerate(S)]   # state first on ties
    ev.sort()
    g=no.native_guard(); H=oh.StandingHold() if hold else None
    out=[]; stand=False
    for rt,kind,i in ev:
        if kind==-1:
            f=S[i][2]; a=(f['fsm_state.current_state'],f['action_info.action_desc'],f['action_info.status.value'],f['body_status.value'])
            stand=oh.is_standing(*a)
            if H: H.set_state(rt,*a)
            continue
        r=L[i]; st=g.add(r[0],r[15],r[16],(r[2],r[3],r[4],r[5],r[6],r[7],r[8]),rt)
        if st=='stamp_back' and H: H.clear_window()
        if st not in ('ok','stamp_back'): continue
        _,x,y,z,yaw=g.pose
        if H: px,py,held=H.update(r[0],rt,x,y,yaw)
        else: px,py,held=x,y,False
        out.append((r[0],rt,x,y,yaw,px,py,held,stand,H.ox if H else 0.0,H.oy if H else 0.0))
    return np.array(out,dtype=float), (H.counts if H else None), g.counts
def slope(t,x,y):
    A=np.vstack([t-t[0],np.ones_like(t)]).T
    vx=np.linalg.lstsq(A,x,rcond=None)[0][0]; vy=np.linalg.lstsq(A,y,rcond=None)[0][0]
    return math.hypot(vx,vy)*100*60   # cm/min
def metrics(b):
    O,cnt,gc=replay(b)
    t,x,y,yaw,px,py,held,stand,ox,oy=(O[:,k] for k in (0,2,3,4,5,6,7,8,9,10))
    res=dict(bag=b,samples=len(O),hold_counts=cnt,guard_drops={k:v for k,v in gc.items() if k not in('ok',) and v})
    # standing intervals >=30 s (state), trimmed 3 s at start
    iv=[];i=0;n=len(t)
    while i<n:
        if stand[i]:
            j=i
            while j+1<n and stand[j+1]: j+=1
            if t[j]-t[i]>=30: iv.append((i,j))
            i=j+1
        else: i+=1
    rows=[]
    for i,j in iv:
        m=(t>=t[i]+3)&(np.arange(n)>=i)&(np.arange(n)<=j)
        if m.sum()<100: continue
        dur=t[m][-1]-t[m][0]
        rows.append(dict(t0=round(t[m][0]-t[0],1),dur_min=round(dur/60,2),raw_cm_min=round(slope(t[m],x[m],y[m]),3),pub_cm_min=round(slope(t[m],px[m],py[m]),3),
                         raw_net_cm=round(100*math.hypot(x[m][-1]-x[m][0],y[m][-1]-y[m][0]),2),pub_net_cm=round(100*math.hypot(px[m][-1]-px[m][0],py[m][-1]-py[m][0]),2),
                         held_frac=round(held[m].mean(),3)))
    res['standing']=rows
    # increments
    dr=np.hypot(np.diff(x),np.diff(y)); dp=np.hypot(np.diff(px),np.diff(py))
    ddx=np.diff(px)-np.diff(x); ddy=np.diff(py)-np.diff(y)
    h0=held[:-1]>0; h1=held[1:]>0
    free=~h0&~h1                                   # neither sample held: published step must equal raw step
    res['free_step_dev_max_m']=float(np.hypot(ddx[free],ddy[free]).max()) if free.any() else 0.0
    ent=np.where(~h0&h1)[0]; ex=np.where(h0&~h1)[0]
    res['entry_exit']=dict(n_entry=len(ent),n_exit=len(ex),
        max_pub_step_at_entry_mm=float(dp[ent].max()*1e3) if len(ent) else 0.0,max_raw_step_at_entry_mm=float(dr[ent].max()*1e3) if len(ent) else 0.0,
        max_pub_step_at_exit_mm=float(dp[ex].max()*1e3) if len(ex) else 0.0,max_raw_step_at_exit_mm=float(dr[ex].max()*1e3) if len(ex) else 0.0,
        max_raw_step_overall_mm=float(dr.max()*1e3))
    # walking = state not standing: offset constant, increments equal
    wk=(~stand[:-1].astype(bool))&(~stand[1:].astype(bool))&free
    res['walking']=dict(steps=int(wk.sum()),dev_max_m=float(np.hypot(ddx[wk],ddy[wk]).max()) if wk.any() else 0.0,
        offset_changes=int((np.abs(np.diff(ox))[np.where(wk)[0]]+np.abs(np.diff(oy))[np.where(wk)[0]]>0).sum()) if wk.any() else 0,
        raw_path_m=float(dr[wk].sum()) if wk.any() else 0.0)
    first=np.argmax(held>0) if held.any() else n
    res['pre_first_hold_bit_identical']=bool(np.all(px[:first]==x[:first]) and np.all(py[:first]==y[:first])); res['pre_first_hold_samples']=int(first)
    # held while raw moving: max 0.5 s window excursion while held
    j=np.searchsorted(t,t-0.5); exc=np.hypot(x-x[j],y-y[j]); res['held_max_win_excursion_mm']=float(exc[held>0].max()*1e3) if held.any() else 0.0
    res['held_frac_standing_total']=round(float(held[stand>0].mean()),3) if stand.any() else None
    res['held_while_nonstanding_samples']=int((held[stand==0]>0).sum())
    # total path published vs raw (whole bag)
    res['net_raw_m']=float(np.hypot(x[-1]-x[0],y[-1]-y[0])); res['net_pub_m']=float(np.hypot(px[-1]-px[0],py[-1]-py[0]))
    return res,O
if __name__=='__main__':
    allr=[]
    for b in sys.argv[1:]:
        r,O=metrics(b); allr.append(r); print(json.dumps(r),flush=True)
        np.save(b+'.replay.npy',O)
    json.dump(allr,open('replay_results.json','w'),indent=1)
