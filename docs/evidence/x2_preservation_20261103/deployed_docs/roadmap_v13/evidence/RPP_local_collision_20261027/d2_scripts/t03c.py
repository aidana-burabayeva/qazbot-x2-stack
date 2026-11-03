import replay, numpy as np, math
rpp=replay.rpp
scans,tf=replay.load('Static_map_trial_1230'); T=replay.TF(tf)
SO=(-5.0,-2.0); fp=rpp.footprint(0.454,0.01)
def swept(t,v,w,carrot=0.5):
    # all world points on the polygon perimeters of current + projected poses (dense sampling), as world cell keys on SO grid
    x,y,th=T.at(t); poses=[(x,y,th)]
    pt=0.05/abs(v); cx,cy,ct=x,y,th; i=1
    while i*pt<1.5:
        i+=1; cx+=pt*v*math.cos(ct); cy+=pt*v*math.sin(ct); ct+=pt*w
        if math.hypot(cx-x,cy-y)>carrot: break
        poses.append((cx,cy,ct))
    S=set()
    for (px,py,pth) in poses:
        c,s=math.cos(pth),math.sin(pth); P=[(px+c*a-s*b,py+s*a+c*b) for a,b in fp]
        for k in range(16):
            (x0,y0),(x1,y1)=P[k],P[(k+1)%16]
            for f in np.linspace(0,1,30): S.add((math.floor((x0+f*(x1-x0)-SO[0])/.05),math.floor((y0+f*(y1-y0)-SO[1])/.05)))
    return S
T0=1793104000
early=set()
for t in [301.597,301.647,301.697,301.747,301.788]: early|=swept(T0+t,0.3,-0.129)
late=swept(T0+301.840,0.3,-0.129)
cand=late-early; print('candidate trigger cells (in 301.84 sweep, not in earlier 5 sweeps):',len(cand))
# replay baseline: track for each cand cell whether any clearing ray traversed it 4204..4301.8 ; and footprint coverage 4301.8..4349
cleared={c:0 for c in cand}; fpc={c:0 for c in cand}
L=replay.Layer(-5.0+round((T.at(scans[0][0])[0]-2+5)/.05)*.05,-2.0+round((T.at(scans[0][0])[1]-2+2)/.05)*.05)
for sc in scans:
    if sc[0]>T0+349.0: break
    ang,rng,fin,clr=replay.scan_rays(sc,'baseline'); p=T.at(sc[0])
    before=L.g.copy(); ox,oy=L.ox,L.oy
    # mark sentinel 7 in cand cells to detect clearing
    for (i,j) in cand:
        mi=int(round((SO[0]+i*.05-L.ox)/.05)); mj=int(round((SO[1]+j*.05-L.oy)/.05))
        if 0<=mi<80 and 0<=mj<80 and L.g[mj,mi]==0: L.g[mj,mi]=7
    L.update(p,ang,rng,fin,clr)
    for (i,j) in cand:
        mi=int(round((SO[0]+i*.05-L.ox)/.05)); mj=int(round((SO[1]+j*.05-L.oy)/.05))
        if 0<=mi<80 and 0<=mj<80:
            if L.g[mj,mi]==0:
                cx=SO[0]+(i+.5)*.05; cy=SO[1]+(j+.5)*.05
                if math.hypot(cx-p[0],cy-p[1])<0.464:
                    if sc[0]>T0+301.84: fpc[(i,j)]+=1
                elif sc[0]<T0+301.84: cleared[(i,j)]+=1
            if L.g[mj,mi]==7: L.g[mj,mi]=0
p=T.at(T0+301.84)
rows=[]
for c in cand:
    cx=SO[0]+(c[0]+.5)*.05; cy=SO[1]+(c[1]+.5)*.05
    d=math.hypot(cx-p[0],cy-p[1]); b=(math.degrees(math.atan2(cy-p[1],cx-p[0])-p[2])+180)%360-180
    rows.append((d,b,cleared[c],fpc[c],c))
rows.sort()
nc=sum(1 for r in rows if r[2]==0); nf=sum(1 for r in rows if r[2]==0 and r[3]>0)
print('candidates never raytrace-cleared 4204-4301.84:',nc,' of which footprint-cleared later (4301.84-4349):',nf)
for r in rows[:60]:
    if r[2]==0: print('  d=%.2f brg=%+.0f rays=%d fp_later=%d cell=%s'%r)
print('bearing range of all candidates', min(r[1] for r in rows), max(r[1] for r in rows))
