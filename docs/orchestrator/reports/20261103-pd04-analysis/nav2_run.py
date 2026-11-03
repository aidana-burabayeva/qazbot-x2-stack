#!/usr/bin/env python3
"""Feed the real Humble nav2_costmap_2d (domain 77, localhost only) with C (candidate replay) or A_rec
(recorded live scan_obstacles/scan_clearing) clouds of the P-D04 bag. Writes per-publication lethal cells.
usage: nav2_run.py <extract_dir> <C|A_rec> [frame_wall_s] [k0] [k1]
"""
import os
import pickle
import sys
import time
import types

sys.path.insert(0, '/home/agi/control_backend/tests/d04')
import nav2_real_costmap as N  # noqa: E402  (sets ROS_DOMAIN_ID=77, localhost only)
assert os.environ['ROS_DOMAIN_ID'] == '77' and os.environ['ROS_LOCALHOST_ONLY'] == '1'


def main():
    d, var = sys.argv[1], sys.argv[2]
    fw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
    from sensor_msgs_py import point_cloud2 as pc2
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    small = pickle.load(open(os.path.join(d, 'small.pkl'), 'rb'))
    frames = pickle.load(open(os.path.join(d, 'frames.pkl'), 'rb'))
    tfs = pickle.load(open(os.path.join(d, 'tf_msgs.pkl'), 'rb'))
    k0 = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    k1 = int(sys.argv[5]) if len(sys.argv) > 5 else len(frames)
    frames = frames[k0:k1]
    tfs.sort(key=lambda t: N._ns(t.header.stamp))
    fr_ns, clouds = [], []
    for f in frames:
        k = f['key']
        h = Header(stamp=Time(sec=k // 10**9, nanosec=k % 10**9), frame_id='base_footprint')
        if var == 'C':
            ob, cl = f['obC'], f['clC']
        else:
            ob, cl = small['ob'][k][1], small['cl'][k][1]
        clouds.append((pc2.create_cloud_xyz32(h, ob.tolist()), pc2.create_cloud_xyz32(h, cl.tolist())))
        fr_ns.append(dict(msg=types.SimpleNamespace(header=h)))
    params = N.load_params()
    t0 = time.time()
    recs = N.run_nav2(params, fr_ns, clouds, tfs, [], f'pd04_{var}', fw, d)
    out = [dict(k=r['k'] + k0 if r['k'] >= 0 else r['k'], phase=r['phase'], xy=r['xy'], md=r['md']) for r in recs]
    pickle.dump(dict(var=var, fw=fw, k0=k0, k1=k1, recs=out, wall=time.time() - t0, params=params),
                open(os.path.join(d, f'nav2_{var}.pkl'), 'wb'))
    print(var, 'pubs', len(out), 'wall', round(time.time() - t0), flush=True)


if __name__ == '__main__':
    main()
