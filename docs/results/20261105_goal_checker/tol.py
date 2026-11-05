import pickle,math,bisect
D=pickle.load(open('ana.pkl','rb'));T0=1793869680.34
tm=D['tf_map_odom'];tb=D['tf_odom_base_footprint'];tbt=[x[0] for x in tb];tmt=[x[0] for x in tm]
GX,GY,GYAW=-7.9978792694671315,-3.494094157972233,-1.6349518954259117
sx,sy=-7.813,-0.478
P=[]  # (t_rel,x,y,yaw)
for o in tb:
    j=max(bisect.bisect(tmt,o[0])-1,0);m=tm[j];ym=m[3]
    x=m[1]+math.cos(ym)*o[1]-math.sin(ym)*o[2];y=m[2]+math.sin(ym)*o[1]+math.cos(ym)*o[2]
    P.append((o[0]-T0,x,y,ym+o[3]))
P=[p for p in P if -1<=p[0]<=75]
def wrap(a): return math.remainder(a,2*math.pi)
def vel(i,w=0.4):
    t=P[i][0];k=i
    while k>0 and t-P[k][0]<w: k-=1
    dt=P[i][0]-P[k][0]
    return ((P[i][1]-P[k][1])/dt,(P[i][2]-P[k][2])/dt) if dt>0.05 else (0,0)
gu=(math.cos(GYAW),math.sin(GYAW))
def rest(i,k=0.38):  # predicted rest point if the command went to zero now: coast 0.38 s * v (D-05 stop fit)
    v=vel(i);return P[i][1]+k*v[0],P[i][2]+k*v[1]
def err(x,y): return math.hypot(x-GX,y-GY)
variants=[('V0 actual: xy .10 yaw .10 stateless',.10,.10,False),('V1 Nav2 default: xy .25 yaw .25 stateful',.25,.25,True),
          ('V2 xy .25 yaw .10 stateless',.25,.10,False),('V3 xy .10 yaw .25 stateless',.10,.25,False),('V4 xy .25 yaw .25 stateless',.25,.25,False),
          ('V5 xy .15 yaw .15 stateless',.15,.15,False)]
print('Recorded track: goal (%.3f,%.3f) yaw %.3f; goal checker evaluated on the 10 Hz composed map pose (RPP/SimpleGoalChecker semantics)'%(GX,GY,GYAW))
for name,xt,yt,sf in variants:
    hit=None;xy_reached=False
    for i,p in enumerate(P):
        if p[0]<0: continue
        d=err(p[1],p[2]);ye=abs(wrap(p[3]-GYAW))
        if sf and d<=xt: xy_reached=True
        ok=((d<=xt) or (sf and xy_reached)) and ye<=yt
        if ok: hit=i;break
    if hit is None: print('%-42s never within 75 s'%name);continue
    p=P[hit];rx,ry=rest(hit);v=math.hypot(*vel(hit))
    print('%-42s first OK t=%5.1fs pose err %.3f m yaw err %.2f | speed then %.2f m/s -> predicted rest err %.3f m (along %.2f, lat %.2f)'%(name,p[0],err(p[1],p[2]),wrap(p[3]-GYAW),v,err(rx,ry),(rx-GX)*gu[0]+(ry-GY)*gu[1],-(rx-GX)*gu[1]+(ry-GY)*gu[0]))
# actual end: rest pose after the SUCCEEDED at 60.2 s
end=[p for p in P if 60.3<=p[0]<=64]
if end: p=end[-1];print('actual end (+%.1fs after SUCCEEDED): err %.3f m (along %.2f lat %.2f) yaw err %.2f'%(p[0]-60.2,err(p[1],p[2]),(p[1]-GX)*gu[0]+(p[2]-GY)*gu[1],-(p[1]-GX)*gu[1]+(p[2]-GY)*gu[0],wrap(p[3]-GYAW)))
print('\nRPP branch around first arrival (carrot = path end): t, dist to goal, angle to carrot, yaw err, MC wz, inferred branch')
mcv=[x for x in D['/aima/mc/locomotion/velocity'] if x[1]!='rc'];mt=[x[0] for x in mcv]
for p in P:
    if 6.4<=p[0]<=7.8 and int(round(p[0]*10))%1==0:
        d=err(p[1],p[2]);ang=wrap(math.atan2(GY-p[2],GX-p[1])-p[3]);q=mcv[bisect.bisect(mt,T0+p[0])-1]
        br='goal-heading' if d<0.10 else ('rotate-to-path' if abs(ang)>0.6 and d<0.5 else 'pure pursuit')
        print('%4.1f  d=%.3f  carrot angle %5.2f  yaw err %5.2f  MC vx %.2f wz %5.2f  -> %s'%(p[0],d,ang,wrap(p[3]-GYAW),q[2],q[3],br))

print('\n== StoppedGoalChecker semantics (within tolerance AND |v|<trans_stop AND |wz|<rot_stop), recorded track, no new motion ==')
ol=D['/qazbot_lio/odom_level'];olt=[x[0] for x in ol]
def twist_at(t):
    k=max(bisect.bisect(olt,T0+t)-1,0);return abs(ol[k][4]),abs(ol[k][5])
for xt,yt in ((.10,.10),(.15,.15),(.20,.20),(.25,.25)):
    for tv,rv in ((0.08,0.05),(0.08,0.10)):
        hit=None
        for p in P:
            if p[0]<0: continue
            v,w=twist_at(p[0])
            if err(p[1],p[2])<=xt and abs(wrap(p[3]-GYAW))<=yt and v<tv and w<rv: hit=p;break
        s='never (75 s)' if hit is None else 't=%.1fs err %.3f m yaw %.2f'%(hit[0],err(hit[1],hit[2]),wrap(hit[3]-GYAW))
        print('xy %.2f yaw %.2f  stopped v<%.2f w<%.2f : %s'%(xt,yt,tv,rv,s))
print('time within xy .10 / .25 (any yaw) while moving v>0.2: ',sum(1 for p in P if p[0]>=0 and err(p[1],p[2])<=.10),'/',sum(1 for p in P if p[0]>=0 and err(p[1],p[2])<=.25),'samples (10 Hz)')
