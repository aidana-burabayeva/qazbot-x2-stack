"""Swept horizontal envelope of X2 about the navigation (LIO IMU) point, from a joint_record.py file.

Per snapshot: forward kinematics of the URDF collision geometry in the pelvis (base_link) frame,
rotated to gravity with the base_link IMU orientation, distances measured in the horizontal plane
from the control point. The control point is the chest-lidar IMU, fixed in the torso link; its
offset comes from the vendor /tf_static (base_link -> lidar_imu_chest_front) at neutral waist.
Usage: python3 envelope.py RECORD.npz [window_s]
"""
import sys

import numpy as np

URDF = __import__('os').path.join(__import__('os').path.dirname(__file__), 'x2_ultra_simple_collision.urdf')
_argv, sys.argv = sys.argv, ['x', URDF]
exec(open(__import__('os').path.join(__import__('os').path.dirname(__file__), 'fk.py')).read())
sys.argv = _argv

CP_IN_BASE_NEUTRAL = np.array([0.08674356826399789, 0.01727979678250372, 0.3344120516244009])
TORSO = next(l for l in links if l in ('torso_link', 'torso_plus_link'))


def quat_to_rot(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


# control point in the torso frame, from the neutral-waist transform
_T0 = link_T(TORSO, {})
CP_IN_TORSO = (np.linalg.inv(_T0) @ np.append(CP_IN_BASE_NEUTRAL, 1.0))[:3]


def snapshot_envelope(q, quat):
    """Returns dict with max horizontal distance from the control point and extents along/across heading."""
    pts = np.vstack(list(world_pts(q).values()))                    # pelvis frame
    cp = (link_T(TORSO, q) @ np.append(CP_IN_TORSO, 1.0))[:3]
    R = quat_to_rot(*quat)                                          # world <- base_link (gravity aligned)
    rel = (pts - cp) @ R.T
    h = np.hypot(rel[:, 0], rel[:, 1])
    fwd = np.array([1.0, 0, 0]) @ R.T
    heading = np.arctan2(fwd[1], fwd[0])
    c, s = np.cos(-heading), np.sin(-heading)
    bx, by = c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]   # along / across the body heading
    return dict(r=h.max(), x_front=bx.max(), x_back=bx.min(), y_left=by.max(), y_right=by.min(),
                ground=rel[:, 2].min())


def analyse(path, win=0.5):
    d = np.load(path)
    names, q, imu, t = list(d['names']), d['q'], d['imu_xyzw'], d['t']
    rows = []
    for k in range(0, len(t), max(1, int(round(win * 50)))):
        rows.append(snapshot_envelope(dict(zip(names, q[k])), imu[k]))
    return rows


if __name__ == '__main__':
    rows = analyse(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.5)
    for key in ('r', 'x_front', 'x_back', 'y_left', 'y_right'):
        v = np.array([r[key] for r in rows])
        print('%-8s min %.3f  median %.3f  max %.3f' % (key, v.min(), np.median(v), v.max()))
    print('snapshots analysed:', len(rows))
