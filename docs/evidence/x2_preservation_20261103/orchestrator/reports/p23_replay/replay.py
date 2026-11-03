import pickle, math, sys
from importlib.machinery import SourceFileLoader
N='/home/agi/control_backend/ws/src/navigation/'
mods={'old':SourceFileLoader('old',N+'x2_motion_adapter.py').load_module(),
      'new':SourceFileLoader('new',N+'x2_motion_adapter.py.p23_candidate').load_module()}
d=pickle.load(open('win.pkl','rb'))
def yaw(q): return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
ev=[(t,'cmd',(m['linear']['x'],m['linear']['y'],m['angular']['z'])) for t,m in d['/qazbot_d1_map/cmd_vel_authorized']]
ev+=[(t,'odom',(m['header']['stamp']['sec']+1e-9*m['header']['stamp']['nanosec'],yaw(m['pose']['pose']['orientation']))) for t,m in d['/qazbot_native/odom']]
ev.sort(key=lambda e:e[0])
T0,T1=1793349871.70,1793349897.95
B=1793349800
out={}
for name,M in mods.items():
    cfg=M.Config(source_name='sdk_qazbot_nav2_1030a', live=False)
    core=M.AdapterCore(cfg); est=M.YawRateEstimator()
    mc=M.McState(stamp=0, source=cfg.source_name, body=M.BODY_STAND, fsm=M.FSM_STABLE)
    cmd,cst=None,float('-inf'); i=0; t=T0; logt=-1e9; rows=[]; logs=[]
    while t<T1:
        while i<len(ev) and ev[i][0]<=t:
            _,k,v=ev[i]; i+=1
            if k=='cmd': cmd,cst=v,ev[i-1][0]
            else: est.update(v[0],v[1],ev[i-1][0])
        mc.stamp=t
        yr=est.get(t,cfg.yaw_meas_max_age_s)
        dd=core.step(t,cmd,cst,mc,True,'','',False,yr)
        y=core.yaw; wz=cmd[2] if (cmd and t-cst<=cfg.cmd_timeout_s) else 0.0
        u=dd.publish[2] if dd.publish else None
        rows.append((t,wz,yr,u,y.integ,'yaw_loop' in dd.notes))
        if y.active and 'yaw_loop' in dd.notes and t-logt>=1.0:
            logt=t; logs.append(f"{t-B:6.2f} yaw_loop target={y.target:+.2f} meas={y.meas:+.2f} u={y.sign*y.u:+.2f} I={y.integ:+.2f}")
        t+=0.02
    out[name]=(rows,logs)
for name,(rows,logs) in out.items():
    print(f'== {name}: 1 Hz log lines (live format)'); print('\n'.join(logs))
    lr=[r for r in rows if r[5]]
    Is=[r[4] for r in rows]
    small=[r for r in rows if r[3] is not None and abs(r[1])<0.15 and abs(r[1])>0.005 and abs(r[3])>=0.4]
    inv=[r for r in rows if r[3] is not None and r[2] is not None and abs(r[1])<0.15 and abs(r[3])>0.005 and r[3]*r[2]<0 and abs(r[3])>abs(r[1])+1e-6]
    print(f'   loop ticks={len(lr)}  max|I|={max(abs(x) for x in Is):.3f}  ticks with I!=0: {sum(1 for x in Is if abs(x)>1e-9)}')
    print(f'   small-target(|w|<0.15) lifted to |u|>=0.4: {len(small)} ticks; e.g. {[(round(r[0]-B,2),round(r[1],3),round(r[3],2)) for r in small[:6]]}')
    print(f'   small-target output amplified against measured motion: {len(inv)} ticks')
pickle.dump(out,open('replay.pkl','wb'))
# per-segment table for new vs old
def segs(rows):
    S=[];cur=None
    for r in rows:
        if r[5]:
            if cur is None: cur=[r[0],r[0],r[1],0,0]
            cur[1]=r[0]; cur[3]=max(cur[3],abs(r[4]))
        elif cur: S.append(cur); cur=None
    if cur: S.append(cur)
    return S
for name in out:
    print(name,'segments (start,end,len,target0,max|I|):',[(round(a-B,2),round(b-a,2),round(w,2),round(m,3)) for a,b,w,m,_ in segs(out[name][0])])
