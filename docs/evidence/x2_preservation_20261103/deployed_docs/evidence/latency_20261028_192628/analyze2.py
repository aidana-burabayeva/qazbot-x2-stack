import json, sys, bisect, glob, os
EV=sys.argv[1]
def pct(v,p):
    s=sorted(v); return round(s[min(len(s)-1,int(p*(len(s)-1)+0.5))],4) if s else None
def st(v): return dict(n=len(v),p50=pct(v,.5),p95=pct(v,.95),p99=pct(v,.99),max=pct(v,1)) if v else None
R={}
for tag in ['A_nav2iface','B_single_iface']:
  for ph in ['full','noraw']:
    f=f'{EV}/probe_{ph}_{tag}.jsonl'; L=open(f).read().splitlines(); topics=json.loads(L[0])['topics']; rows=[json.loads(l) for l in L[1:]]
    T={t:i for i,t in enumerate(topics)}; by={}
    for r in rows: by.setdefault(r[0],[]).append(r)
    zero={t:sum(1 for r in by.get(i,[]) if isinstance(r[1],float) and r[1]<1e9) for t,i in T.items()}
    out={'zero_stamp_msgs':{k:v for k,v in zero.items() if v}}
    for t in ['/aima/hal/sensor/lidar_chest_front/lidar_pointcloud','/aima/hal/sensor/lidar_chest_front/imu','/aima/mc/leg_odometry','/qazbot_lio/cloud_registered','/qazbot_lio/odometry','/qazbot_lio/scan','/qazbot_native/odom']:
        rs=[r for r in by.get(T.get(t,-1),[]) if r[1]>1e9]
        if rs: out[t]=dict(age=st([r[2]-r[1] for r in rs]),stamp_gap_max=pct([b[1]-a[1] for a,b in zip(rs,rs[1:])],1),recv_gap=st([b[3]-a[3] for a,b in zip(rs,rs[1:])]))
    # decompose worst scan ages / worst tick gaps
    scan=[r for r in by[T['/qazbot_lio/scan']] if r[1]>1e9]; creg={round(r[1],6):r for r in by[T['/qazbot_lio/cloud_registered']] if r[1]>1e9}
    raw=[r for r in by.get(T.get('/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',-1),[]) if r[1]>1e9]; rst=[r[1] for r in raw]
    imu=[r for r in by[T['/aima/hal/sensor/lidar_chest_front/imu']] if r[1]>1e9]; ist=[r[1] for r in imu]
    leg=[r for r in by[T['/aima/mc/leg_odometry']]]; lmo=[r[3] for r in leg]
    worst=[]
    for a,b in zip(scan,scan[1:]):
        worst.append((b[2]-a[1], a, b))   # age just before b arrives (trust-like worst)
    worst.sort(key=lambda x:-x[0])
    dec=[]
    for w,a,b in worst[:6]:
        c=creg.get(round(b[1],6)); d=dict(max_age_before_next=round(w,3), next_scan_recv_age=round(b[2]-b[1],3), scan_stamp_gap=round(b[1]-a[1],3))
        if c: d['creg_age']=round(c[2]-c[1],3); d['scan_minus_creg']=round(b[2]-c[2],3)
        j=bisect.bisect_right(rst,b[1])-1
        if raw and j>=0: d['raw_age_vs_end']=round(raw[j][2]-raw[j][1]-0.0952,3); d['lio_stage']=round((c[2] if c else b[2])-raw[j][2],3)
        m=bisect.bisect_left(ist,b[1]); 
        if m<len(imu): d['imu_cover_age']=round(imu[m][2]-b[1],3)
        # max leg odom receipt gap in window
        k0=bisect.bisect_left(lmo,a[3]); k1=bisect.bisect_left(lmo,b[3]); g=[leg[k][3]-leg[k-1][3] for k in range(max(1,k0),min(len(leg),k1+1))]
        d['leg_recv_gap_max_in_window']=round(max(g),3) if g else None
        dec.append(d)
    out['worst_scan_windows']=dec
    R[f'{ph}_{tag}']=out
json.dump(R,open(EV+'/analysis2.json','w'),indent=1)
for k,v in R.items():
    print('####',k, json.dumps(v['zero_stamp_msgs']))
    for t in ['/aima/hal/sensor/lidar_chest_front/lidar_pointcloud','/qazbot_lio/cloud_registered','/qazbot_lio/scan']:
        if t in v: print(' ',t.split('/')[-1],json.dumps(v[t]['age']))
    for d in v['worst_scan_windows'][:4]: print('  W',json.dumps(d))
