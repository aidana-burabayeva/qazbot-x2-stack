import pickle, numpy as np, json, sys
D=pickle.load(open('b183.pkl','rb')); meta=D['meta']; pts=D['pts']; full=D['full']; dirs=D['dirs']
n0=D['plane0'][:3]; d0=D['plane0'][3]
xs=np.array([1.0,0,0]); f=xs-(xs@n0)*n0; f/=np.linalg.norm(f); L=np.cross(n0,f)
def lev(P,n=n0,d=d0):
    return np.column_stack((P[:,:3]@f, P[:,:3]@L, P[:,:3]@n+d))
sod=np.array([m['sod'] for m in meta])
out={}
# --- A geometry / sway
ok=[m for m in meta if 'nf' in m]
N=np.array([m['nf'] for m in ok]); Dd=np.array([m['df'] for m in ok]); sd=np.array([m['sdf'] for m in ok])
ang=np.degrees(np.arccos(np.clip(N@n0,-1,1)))
pitch=np.degrees(np.arcsin(-(N@f)))  # sensor-x tilt down positive? sign check
roll=np.degrees(np.arcsin(N@L))
out['geom']=dict(height0=float(d0),pitch0_deg=float(np.degrees(np.arcsin(-(n0@xs)))),frames=len(ok),
  tilt_vs_global_deg=[float(np.percentile(ang,q)) for q in (50,95,99,100)],
  pitch_deg_p1_p99=[float(np.percentile(pitch,1)),float(np.percentile(pitch,99))],
  roll_deg_p1_p99=[float(np.percentile(roll,1)),float(np.percentile(roll,99))],
  height_p1_p99=[float(np.percentile(Dd,1)),float(np.percentile(Dd,99))],
  fit_sd_p50_p99=[float(np.percentile(sd,50)),float(np.percentile(sd,99))],
  r3min_overall=float(min(m['r3min'] for m in meta)),
  r3min_p1=float(np.percentile([m['r3min'] for m in meta],1)),
  frames_with_r3_lt=[int(sum(m['n_r3_lt'][i]>0 for m in meta)) for i in range(5)])
# r3 min when objects near: frames with any pts r3<1.0
print(json.dumps(out['geom'],indent=0))
# --- floor residual by range (per-frame plane), empty windows
EMPTY=[(66616,66665),(66701,66845),(67019,67113),(67123,67336)]
def inwin(s,W): return any(a<=s<=b for a,b in W)
fl={}
for m in meta:
    if 'fl' not in m or not inwin(m['sod'],EMPTY): continue
    for (r0,n,med,s,p99,p999,mx) in m['fl']:
        fl.setdefault(r0,[]).append((n,med,s,p99,p999,mx))
out['floor']={}
for r0,v in sorted(fl.items()):
    v=np.array(v)
    out['floor'][r0]=dict(n_per_frame=float(v[:,0].mean()),median_bias=float(np.median(v[:,1])),sd=float(np.median(v[:,2])),p99=float(np.median(v[:,3])),p999=float(np.median(v[:,4])),max_med=float(np.median(v[:,5])),max_max=float(v[:,5].max()))
    print('floor',r0,{k:round(x,4) for k,x in out['floor'][r0].items()})
pickle.dump(out,open('an_geom.pkl','wb'))
