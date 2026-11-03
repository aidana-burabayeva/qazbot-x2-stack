import replay, numpy as np, math
scans,tf=replay.load('Static_map_trial_1230'); T=replay.TF(tf); T0=1793104000
snap=np.load(replay.EV+'Static_map_trial_1230/local_costmap_before_clear.npy').reshape(80,80); SO=(-5.0,-2.0)
cells=[(54,28),(54,29),(54,21),(53,31),(52,33),(52,34),(51,36),(51,37)]
print('snapshot values',[int(snap[j,i]) for i,j in cells])
# clearing rays after 301.84 through these cells (baseline), and were they inside the 4x4 window at all times
L=replay.Layer(-5.0+round((T.at(scans[0][0])[0]-2+5)/.05)*.05,-2.0+round((T.at(scans[0][0])[1]-2+2)/.05)*.05)
cnt={c:0 for c in cells}; out={c:0 for c in cells}
for sc in scans:
    if sc[0]>T0+349.0: break
    ang,rng,fin,clr=replay.scan_rays(sc,'baseline'); p=T.at(sc[0])
    for (i,j) in cells:
        mi=int(round((SO[0]+i*.05-L.ox)/.05)); mj=int(round((SO[1]+j*.05-L.oy)/.05))
        if 0<=mi<80 and 0<=mj<80: L.g[mj,mi]=9
    L.update(p,ang,rng,fin,clr)
    for (i,j) in cells:
        mi=int(round((SO[0]+i*.05-L.ox)/.05)); mj=int(round((SO[1]+j*.05-L.oy)/.05))
        if 0<=mi<80 and 0<=mj<80:
            if L.g[mj,mi]==0 and sc[0]>T0+301.84: cnt[(i,j)]+=1
        elif sc[0]>T0+301.84: out[(i,j)]+=1
print('cleared after 301.84 (ray or fp):',cnt); print('outside window after 301.84 (#scans):',out)
