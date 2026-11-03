import replay, numpy as np, math
E=replay.EV+'Static_map_trial_1230/'
snap=np.load(E+'local_costmap_before_clear.npy').reshape(80,80); SO=(-5.0,-2.0)
rep=np.load('t03_replay_4349.npy')
only=(snap==254)&(rep!=254)
scans,tf=replay.load('Static_map_trial_1230'); T=replay.TF(tf)
jj,ii=np.nonzero(only); wx=SO[0]+(ii+.5)*.05; wy=SO[1]+(jj+.5)*.05
for tt in [4301.79,4349.0]:
    p=T.at(1793100000+tt); d=np.hypot(wx-p[0],wy-p[1]); b=(np.degrees(np.arctan2(wy-p[1],wx-p[0])-p[2])+180)%360-180
    print('snapshot-only cells (%d) rel. to pose at %.2f: d<1.2 -> %d'%(only.sum(),tt,(d<1.2).sum()))
    for k in np.argsort(d)[:12]: print('   d=%.2f brg=%.0f odom(%.3f,%.3f)'%(d[k],b[k],wx[k],wy[k]))
# union test at collision instants: replay state at that time + snapshot-only cells
