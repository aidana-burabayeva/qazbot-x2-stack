import math
from importlib.machinery import SourceFileLoader
N='/home/agi/control_backend/ws/src/navigation/'
mods={'old':SourceFileLoader('old',N+'x2_motion_adapter.py').load_module(),
      'new':SourceFileLoader('new',N+'x2_motion_adapter.py.p23_candidate').load_module()}
def inv(tab,u):  # static curve u->rate
    if u<=tab[0][1]: return tab[0][0]*max(0,(u-0.3)/(tab[0][1]-0.3))
    for (x0,y0),(x1,y1) in zip(tab,tab[1:]):
        if u<=y1: return x0+(x1-x0)*(u-y0)/(y1-y0)
    return tab[-1][0]
# scenario (t0,t1,vx,wz): after-walk CW 0.6 with RPP vx blip and trust-gate zero pauses (live 1030 pattern)
SC=[(0,1.2,0,-0.6),(1.2,1.35,0.06,-0.3),(1.35,2.6,0,-0.6),(2.6,3.3,0,0),(3.3,4.6,0,-0.6),(4.6,4.65,0,-0.10),(4.65,6.0,0,-0.6),(6.0,6.5,0,0)]
def cmd_at(t):
    for a,b,vx,wz in SC:
        if a<=t<b: return (vx,0.0,wz)
    return (0.0,0.0,0.0)
for name,M in mods.items():
    cfg=M.Config(live=False); core=M.AdapterCore(cfg)
    mc=M.McState(stamp=0, source=cfg.source_name, body=M.BODY_STAND, fsm=M.FSM_STABLE)
    G=0.6; tau=0.2; dead=0.4; rate=0.0; hist=[]; t=0.0; yaw=0.0; Imax=0; trace=[]
    while t<6.5:
        mc.stamp=t; c=cmd_at(t)
        d=core.step(t,c,t,mc,True,'','',False,rate)   # perfect meas (filter not modelled)
        u=d.publish[2] if d.publish else 0.0
        hist.append((t,u))
        ud=[x for x in hist if x[0]<=t-dead]; ud=ud[-1][1] if ud else 0.0
        tgt=-G*inv(M.YAW_FF[-1],abs(ud)) if ud<0 else (G*inv(M.YAW_FF[1],ud) if ud>0 else 0.0)
        rate+= (tgt-rate)*0.02/tau; yaw+=rate*0.02
        Imax=max(Imax,abs(core.yaw.integ))
        if abs((t*50)%25)<1e-6 or abs((t*50)%25-25)<1e-6: trace.append(f"{t:.1f}:u{u:+.2f}/I{core.yaw.integ:+.2f}/m{rate:+.2f}")
        t=round(t+0.02,4)
    print(f"{name}: max|I|={Imax:.3f} total yaw={math.degrees(yaw):+.1f} deg (ideal pure-rotation cmd = {math.degrees(-0.6*(1.2+1.25+1.3+1.35)):+.1f})")
    print('  ',' '.join(trace))
# checks on new logic
M=mods['new']; cfg=M.Config(live=False); L=M.YawRateLoop(cfg)
for k in range(60): L.update(k*0.02,-0.6,-0.2)
I1=L.integ; L.pause(1.2,stopped=False); L.update(1.3,-0.6,-0.2); keep=L.integ
L.pause(1.4,stopped=True); L.update(2.6,-0.6,-0.2); expired=L.integ
for k in range(60): L.update(3+k*0.02,-0.6,-0.2)
L.pause(4.2,stopped=True); L.update(4.3,+0.6,0.0); flipped=L.integ
print(f"checks: I after 1.2s={I1:+.3f}; after non-stop pause kept={keep:+.3f}; after 1.2s pause (>hold) reset={expired:+.3f}; sign change reset={flipped:+.3f}")
