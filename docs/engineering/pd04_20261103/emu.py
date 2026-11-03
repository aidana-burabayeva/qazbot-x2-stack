import sys, pickle, time
sys.path.insert(0, '/home/agi/control_backend/tests/d04')
import numpy as np
import costmap_emul as ce
import replay_d04 as R
D = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 10**9
small = pickle.load(open(D + '/small.pkl', 'rb')); frames = pickle.load(open(D + '/frames.pkl', 'rb'))[:n]
L = {v: ce.ObstacleLayer(4.0, 0.05, 2.0, R.FOOTPRINT, R.FP_PAD) for v in ('A_rec', 'C')}
out = {v: [] for v in L}
t0 = time.time()
for k, f in enumerate(frames):
    pose = f['pose']; org = (pose[0], pose[1], 0.0)
    for v in L:
        if v == 'C':
            ob, cl = f['obC'], f['clC']
        else:
            ob, cl = small['ob'][f['key']][1], small['cl'][f['key']][1]
        mk = ce.Observation(R.body_to_global(ob.astype(float), pose), org, R.OBST_MAX, 0.0, R.RAY_MAX, 0.0, 0.0, 0.0)
        cr = ce.Observation(R.body_to_global(cl.astype(float), pose), org, 2.5, 0.0, R.RAY_MAX, 0.0, 0.0, 0.0)
        L[v].update(pose, [mk], [cr])
        out[v].append(L[v].lethal_world().astype(np.float32))
print('emu', len(frames), 'frames', round(time.time() - t0, 1), 's', flush=True)
pickle.dump(dict(out=out, keys=[f['key'] for f in frames], poses=[f['pose'] for f in frames]), open(D + '/emu.pkl', 'wb'))
