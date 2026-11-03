"""Rigid-body geometry for the future X2 controller interface (no ROS side effects).

Convention: T_A_B maps coordinates in B into A. FAST-LIO body is the
lidar IMU, not the pelvis/base_link commanded by the walking controller.
"""
import math
import numpy as np


def transform(translation, quaternion):
    """Construct SE(3) from xyz and xyzw; reject malformed inputs."""
    t = np.asarray(translation, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('Expected finite xyz and xyzw')
    length = np.linalg.norm(q)
    if length == 0 or abs(length - 1) > 1e-6:
        raise ValueError('Quaternion must be normalized')
    x, y, z, w = q / length
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = t
    return out


def invert_rigid(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Expected a finite 4x4 transform')
    R = matrix[:3, :3]
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or not np.allclose(R.T @ R, np.eye(3), atol=1e-9, rtol=0) or not np.isclose(np.linalg.det(R), 1., atol=1e-9, rtol=0):
        raise ValueError('Expected a proper rigid transform')
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ matrix[:3, 3]
    return out


def world_base(world_imu, base_imu):
    """Recover actual body pose; base_imu must be evaluated at the same time."""
    return np.asarray(world_imu) @ invert_rigid(base_imu)


def rotation_quaternion(rotation):
    """Convert a proper rotation to normalized xyzw, including half turns."""
    R = np.asarray(rotation, dtype=float)
    check = np.eye(4)
    check[:3, :3] = R
    invert_rigid(check)
    trace = np.trace(R)
    if trace > 0:
        s = 2*math.sqrt(trace+1)
        q = np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, s/4])
    else:
        i = int(np.argmax(np.diag(R))); j = (i+1)%3; k = (i+2)%3
        s = 2*math.sqrt(1+R[i,i]-R[j,j]-R[k,k])
        q = np.zeros(4)
        q[i] = s/4; q[j] = (R[j,i]+R[i,j])/s; q[k] = (R[k,i]+R[i,k])/s
        q[3] = (R[k,j]-R[j,k])/s
    return q/np.linalg.norm(q)


def planar_pose(level_rotation, world_pose):
    """Project a 3D body pose into a gravity-aligned horizontal reference."""
    level_rotation = np.asarray(level_rotation, dtype=float)
    if level_rotation.shape != (3, 3) or not np.isfinite(level_rotation).all() or not np.allclose(level_rotation.T @ level_rotation, np.eye(3), atol=1e-8, rtol=0) or not np.isclose(np.linalg.det(level_rotation), 1., atol=1e-8, rtol=0):
        raise ValueError('Expected a proper leveling rotation')
    p = level_rotation @ world_pose[:3, 3]
    forward = level_rotation @ world_pose[:3, 0]
    if np.linalg.norm(forward[:2]) < 1e-8:
        raise ValueError('Body forward axis is vertical: planar heading is undefined')
    return np.array([p[0], p[1], math.atan2(forward[1], forward[0])])
