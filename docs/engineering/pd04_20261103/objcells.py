import sys,pickle,numpy as np
sys.path.insert(0,'.'); sys.path.insert(0,'/home/agi/control_backend/tests/d04')
from cmlib import *
from analyze import EV, region_cells, emu_src
import analyze
fr=pickle.load(open(D+'/frames.pkl','rb'))
emu=analyze.emu
srcs={'A_emu':emu['out']['A_rec'],'C_emu':emu['out']['C']}
extra={}
import os
for v,nm in (('C','C_nav2'),('A_rec','A_nav2')):
    if os.path.exists(D+f'/nav2_{v}.pkl'):
        extra[nm]=nav2(v)
key=lambda xy: set(map(tuple,np.floor(np.asarray(xy)/0.05).astype(int)))
for e in EV:
    cells,_=region_cells(e)
    idx=[i for i,f in enumerate(fr) if e['on']+1<=sod(f['key'])<=e['off']]
    # obstacle points (recorded live scan_obstacles == C obstacles) in region, per frame (world keys via frame pose)
    import replay_d04 as R
    hits={}; nob=[]
    for i in idx:
        f=fr[i]; ob=f['obC'].astype(float)
        if len(ob):
            kb=np.floor(ob[:,:2]/0.05).astype(int); m=np.array([(a,b) in cells for a,b in kb])
        else: m=np.zeros(0,bool)
        nob.append(int(m.sum()))
        G=R.body_to_global(ob[m],f['pose']) if m.sum() else np.empty((0,3))
        for k in key(G[:,:2]) if len(G) else []: hits[k]=hits.get(k,0)+1
    pers={k for k,c in hits.items() if c>=0.5*len(idx)}
    line=f"{e['id']} {e['what']}: frames {len(idx)}, scan_obstacles pts in region per frame min/med/max {min(nob)}/{np.median(nob):.0f}/{max(nob)}, frames with 0: {sum(n==0 for n in nob)}; persistent obstacle cells {len(pers)}"
    print(line)
    for nm,L in srcs.items():
        lk=[len(pers & key(L[i][:, :2])) for i in idx] if pers else [0]
        print(f"   {nm}: persistent cells lethal min {min(lk)}/{len(pers)} med {np.median(lk)}")
    for nm,S in extra.items():
        # nav2 xy in odom (native) frame; use body-frame region instead: count lethal cells in region per pub
        pass

print('--- per-frame A-vs-C on persistent obstacle cells and on current-frame obstacle cells')
for e in EV[:2]:
    cells,_=region_cells(e)
    idx=[i for i,f in enumerate(fr) if e['on']+1<=sod(f['key'])<=e['off']]
    hits={}
    cur=[]
    for i in idx:
        f=fr[i]; ob=f['obC'].astype(float)
        kb=np.floor(ob[:,:2]/0.05).astype(int); m=np.array([(a,b) in cells for a,b in kb])
        G=R.body_to_global(ob[m],f['pose'])
        ks=key(G[:,:2]); cur.append(ks)
        for k in ks: hits[k]=hits.get(k,0)+1
    pers={k for k,c in hits.items() if c>=0.5*len(idx)}
    onlyA=0; onlyC=0; curmissA=0; curmissC=0; fr_lossC=0
    for j,i in enumerate(idx):
        a=key(emu['out']['A_rec'][i][:,:2]); c=key(emu['out']['C'][i][:,:2])
        onlyA+=len(pers&a-c); onlyC+=len(pers&c-a)
        curmissA+=len(cur[j]-a); curmissC+=len(cur[j]-c)
        if len(pers&a-c)>0: fr_lossC+=1
    print(e['id'], f"persistent {len(pers)}; cell-frames lethal in A not C: {onlyA}, in C not A: {onlyC}; frames where C lacks a persistent cell A has: {fr_lossC}/{len(idx)}; current-frame obstacle cells not lethal after update A {curmissA} C {curmissC}")
