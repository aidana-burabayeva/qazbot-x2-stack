"""Offline REF02 experiment: full SE(3) native poses -> cloud at a source reference time.

This is pure geometry, not a ROS node or a replacement for level_bridge. The
output frame is the tracked IMU at reference_stamp, not base_footprint. Gravity
alignment, sensor calibration acceptance, and navigation qualification remain
separate. No extrapolation or interpolation through an excessive pose gap.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass
class Deskewed:
    points: np.ndarray
    reference_stamp: float
    maximum_interpolation_gap_s: float


def deskew(points, offsets_ns, header_stamp, pose_stamps, positions, quaternions,
           rotation_imu_lidar, translation_imu_lidar, reference_stamp, max_pose_gap_s,
           max_linear_rate_mps, max_angular_rate_radps):
    points = np.asarray(points, dtype=float)
    offsets = np.asarray(offsets_ns, dtype=float)
    stamps = np.asarray(pose_stamps, dtype=float)
    positions = np.asarray(positions, dtype=float)
    quaternions = np.asarray(quaternions, dtype=float)
    extrinsic = np.asarray(rotation_imu_lidar, dtype=float)
    lever = np.asarray(translation_imu_lidar, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or
            offsets.shape != (len(points),) or stamps.ndim != 1 or len(stamps) < 2 or
            positions.shape != (len(stamps), 3) or quaternions.shape != (len(stamps), 4) or
            extrinsic.shape != (3, 3) or lever.shape != (3,)):
        raise ValueError('invalid array shape or empty trajectory/cloud')
    if not all(np.isfinite(x).all() for x in (points, offsets, stamps, positions, quaternions, extrinsic, lever)):
        raise ValueError('non-finite cloud, timestamp, pose or extrinsic')
    budgets = [max_pose_gap_s, max_linear_rate_mps, max_angular_rate_radps]
    if not np.isfinite([header_stamp, reference_stamp] + budgets).all() or min(budgets) <= 0:
        raise ValueError('invalid reference/source stamp or interpolation gap budget')
    if header_stamp <= 0 or reference_stamp <= 0 or np.any(stamps <= 0) or np.any(offsets < 0):
        raise ValueError('nonpositive source stamp or negative point offset')
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('duplicate/backward pose stamps; source epoch is not continuous')
    if not np.allclose(np.linalg.norm(quaternions, axis=1), 1.0, atol=1e-3, rtol=0):
        raise ValueError('invalid pose quaternion')
    if not np.allclose(extrinsic.T @ extrinsic, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(np.linalg.det(extrinsic), 1):
        raise ValueError('invalid extrinsic rotation')

    times = header_stamp + offsets * 1e-9
    queries = np.r_[times, reference_stamp]
    if queries.min() < stamps[0] or queries.max() > stamps[-1]:
        raise ValueError('pose history does not bracket all points and the reference; extrapolation refused')
    hi = np.searchsorted(stamps, queries, side='left')
    lo = np.maximum(hi - 1, 0)
    lo[stamps[hi] == queries] = hi[stamps[hi] == queries]
    largest = float(np.max(stamps[hi] - stamps[lo]))
    if largest > max_pose_gap_s:
        raise ValueError('pose interpolation gap exceeds the supplied budget')

    first, last = int(lo.min()), int(hi.max()) + 1
    if last - first < 2:
        first, last = max(0, first - 1), min(len(stamps), last + 1)
    stamps, positions, quaternions = stamps[first:last], positions[first:last], quaternions[first:last]
    rotations_source = Rotation.from_quat(quaternions)
    dt = np.diff(stamps)
    linear = np.linalg.norm(np.diff(positions, axis=0), axis=1) / dt
    angular = (rotations_source[:-1].inv() * rotations_source[1:]).magnitude() / dt
    if np.any(linear > max_linear_rate_mps) or np.any(angular > max_angular_rate_radps):
        raise ValueError('pose jump exceeds the supplied linear/angular rate budget')

    # Work near zero rather than epoch-sized floating point times inside Slerp.
    epoch = stamps[0]
    rotations = Slerp(stamps - epoch, rotations_source)(queries - epoch)
    translations = np.column_stack([np.interp(queries, stamps, positions[:, k]) for k in range(3)])
    in_imu = points @ extrinsic.T + lever
    in_world = rotations[:-1].apply(in_imu) + translations[:-1]
    at_reference = rotations[-1].inv().apply(in_world - translations[-1])
    return Deskewed(at_reference, float(reference_stamp), largest)
