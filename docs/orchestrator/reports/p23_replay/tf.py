import pickle, math, bisect
d=pickle.load(open('win.pkl','rb'))
def yaw(q): return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
pairs={}
for t,m in d['/qazbot_d1_map/tf']:
    for tr in m['transforms']:
        k=(tr['header']['frame_id'],tr['child_frame_id']); st=tr['header']['stamp']['sec']+1e-9*tr['header']['stamp']['nanosec']
        pairs.setdefault(k,[]).append((t,st,yaw(tr['transform']['rotation']),tr['transform']['translation']['x'],tr['transform']['translation']['y']))
for k,v in pairs.items(): print(k,len(v))
mo=pairs.get(('map','odom'),[]); 
ob=[v for k,v in pairs.items() if k[0]=='odom'][0]
am=[(t,m['header']['stamp']['sec']+1e-9*m['header']['stamp']['nanosec'],yaw(m['pose']['pose']['orientation'])) for t,m in d['/qazbot_d1_map/amcl_pose']]
gp=d['/qazbot_d1_map/goal_pose'][0][1]; gy=yaw(gp['pose']['orientation']); print('goal yaw',math.degrees(gy))
def last(seq,t):
    i=bisect.bisect_right([x[0] for x in seq],t)-1; return seq[i] if i>=0 else None
print(' t(rel)  map->odom(yaw,stamp_age)  odom->base yaw  TFyaw  err_TF | amcl yaw err_amcl (amcl stamp age)')
for t in [x/20 for x in range(1880,2000,2)]+[105.0,110.0,115.0]:
    T=1793349800+t
    a=last(mo,T); b=last(ob,T); c=last(am,T)
    tfy=a[2]+b[2]; e=math.degrees(math.remainder(gy-tfy,2*math.pi)); ea=math.degrees(math.remainder(gy-c[2],2*math.pi))
    print(f"{t:6.1f} mo={math.degrees(a[2]):+7.1f}({T-a[1]:.2f}s) ob={math.degrees(b[2]):+7.1f}({T-b[1]:.2f}s) TF={math.degrees(math.remainder(tfy,2*math.pi)):+7.1f} errTF={e:+6.1f} | amcl={math.degrees(c[2]):+7.1f} err={ea:+6.1f} (stamp {T-c[1]:.2f}s ago)")
print('map->odom updates in 93-99:', [(round(x[0]-1793349800,2),round(math.degrees(x[2]),1)) for x in mo if 1793349893<x[0]<1793349899][:40])
print('--- fine: each odom->base sample 97.2..97.7, TF err with latest map->odom')
for (t,st,y,x_,y_) in ob:
    if 1793349897.2<=t<=1793349897.7:
        a=last(mo,t); tfy=a[2]+y
        print(f"{t-1793349800:.3f} stamp_age={t-st:.3f} odomyaw={math.degrees(y):+.2f} errTF={math.degrees(math.remainder(gy-tfy,2*math.pi)):+.2f} deg")
print('tol 0.35 rad =',math.degrees(0.35))
print('--- tf2-style: map->odom interpolated at the odom->base STAMP (map->odom stamps are future-dated)')
mos=sorted((x[1],x[2]) for x in mo)
def mo_at(s):
    i=bisect.bisect_right([a for a,_ in mos],s)
    a0,y0=mos[i-1]; a1,y1=mos[i]; f=(s-a0)/(a1-a0); return y0+f*math.remainder(y1-y0,2*math.pi)
for (t,st,y,x_,y_) in ob:
    if 1793349897.2<=t<=1793349897.6:
        tfy=mo_at(st)+y
        print(f"{t-1793349800:.3f} map->odom@stamp={math.degrees(mo_at(st)):+.2f} errTF={math.degrees(math.remainder(gy-tfy,2*math.pi)):+.2f} deg")
print('map->odom stamp - recv:', [round(x[1]-x[0],2) for x in mo if 1793349896<x[0]<1793349898])
