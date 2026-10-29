"""Swept cloud experiment using D1 raycast walls, analytic SE(3) truth and sampled native poses.

No ROS publishers or MC. The scan pattern is synthetic, not a Livox hardware model.
Truth supplies native samples: this measures interpolation/geometry, not native odometry accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from x2_twin_sim.occupancy import OccupancyMap, RayCaster

from deskew import deskew


def trajectory(t, speed, yaw_rate, sway, rotation_imu_lidar):
    s = np.asarray(t) - 100.
    yaw = -.3490658504 + yaw_rate * s
    roll = np.deg2rad(2) * np.sin(2 * np.pi * 1.6 * s) if sway else np.zeros_like(s)
    pitch = np.deg2rad(1.5) * np.sin(2 * np.pi * 1.3 * s) if sway else np.zeros_like(s)
    r = Rotation.from_euler('z', yaw) * Rotation.from_euler('xy', np.column_stack([roll, pitch]))
    r = r * Rotation.from_matrix(rotation_imu_lidar.T)
    p = np.column_stack([-7.3 + speed * s * np.cos(-.3490658504),
                         4.6 + speed * s * np.sin(-.3490658504),
                         1. + (.015 * np.sin(2 * np.pi * 1.6 * s) if sway else np.zeros_like(s))])
    return r, p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--map', default='/opt/x2twin/stack/maps/first_map_nav2_v3/map.yaml')
    ap.add_argument('--extrinsics', default='/backend_logs/x2_extrinsics.json')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    ex = json.load(open(a.extrinsics))
    extrinsic = np.asarray(ex['R_imu_lidar'])
    lever = np.asarray(ex['T_imu_lidar'])
    caster = RayCaster(OccupancyMap.from_yaml(a.map), n_beams=1440, range_min=.45, range_max=8., fov_deg=92.)
    ranges = caster.scan((-7.3, 4.6, -.3490658504))
    hit = np.flatnonzero(np.isfinite(ranges))
    if len(hit) < 20:
        raise RuntimeError('insufficient D1 wall returns at the fixed start')
    # Extrude the raycast wall intersections vertically. The scene is known exactly.
    ray = -.3490658504 + caster.angle_min + hit * caster.angle_increment
    xy = np.column_stack([-7.3 + ranges[hit] * np.cos(ray), 4.6 + ranges[hit] * np.sin(ray)])
    n = 25200
    idx = np.arange(n) % len(hit)
    world = np.column_stack([xy[idx], .4 + 1.3 * ((np.arange(n) // len(hit)) % 30) / 29.])
    offsets = np.linspace(332000., 95181000., n)
    point_t = 100. + offsets * 1e-9
    samples = 100. + np.arange(-2, 20) / 166.
    reference = point_t[-1]
    rows = []
    for label, v, w, sway in [('standing', 0, 0, False), ('forward', .3, 0, False),
                              ('left_turn', 0, .6, False), ('right_turn', 0, -.6, False),
                              ('walk_turn_sway', .3, .6, True)]:
        true_r, true_p = trajectory(point_t, v, w, sway, extrinsic)
        raw = (true_r.inv().apply(world - true_p) - lever) @ extrinsic
        ref_r, ref_p = trajectory(np.array([reference]), v, w, sway, extrinsic)
        expected = ref_r[0].inv().apply(world - ref_p[0])
        pose_r, pose_p = trajectory(samples, v, w, sway, extrinsic)
        args = dict(points=raw, offsets_ns=offsets, header_stamp=100., pose_stamps=samples,
                    positions=pose_p, quaternions=pose_r.as_quat(), rotation_imu_lidar=extrinsic,
                    translation_imu_lidar=lever, reference_stamp=reference, max_pose_gap_s=.02,
                    max_linear_rate_mps=2., max_angular_rate_radps=5.)
        result = deskew(**args)
        before = np.linalg.norm(raw @ extrinsic.T + lever - expected, axis=1)
        after = np.linalg.norm(result.points - expected, axis=1)
        elapsed = []
        for _ in range(5):
            start = time.perf_counter()
            deskew(**args)
            elapsed.append(time.perf_counter() - start)
        rows.append(dict(case=label, points=n, no_deskew_rmse_m=float(np.sqrt(np.mean(before**2))),
                         deskew_rmse_m=float(np.sqrt(np.mean(after**2))), deskew_p95_m=float(np.percentile(after, 95)),
                         deskew_max_m=float(after.max()), laptop_elapsed_median_s=float(np.median(elapsed)),
                         maximum_interpolation_gap_s=result.maximum_interpolation_gap_s))
        if label == 'walk_turn_sway':
            np.savez_compressed(a.out / 'walk_turn_sway.npz', expected=expected, distorted=raw @ extrinsic.T + lever,
                                corrected=result.points, point_offsets_s=offsets * 1e-9)
    manifest = dict(experiment='REF02 synthetic swept cloud; not a ROS/Nav2 or hardware qualification',
                    map_path=a.map, map_yaml_sha256=hashlib.sha256(Path(a.map).read_bytes()).hexdigest(),
                    native_sample_hz=166., sweep_duration_s=float((offsets[-1] - offsets[0]) * 1e-9),
                    experimental_budgets=dict(max_pose_gap_s=.02, max_linear_rate_mps=2., max_angular_rate_radps=5.),
                    reference='scan end derived from source stamp + point offset; IMU reference frame',
                    limits=['native samples generated from truth; native accuracy not measured',
                            'D1 wall extrusion and synthetic firing order; not Livox optics/occlusion',
                            'gravity alignment, LaserScan semantics and AMCL/Nav2 not included',
                            'laptop timing does not qualify Jetson CPU load'], cases=rows)
    (a.out / 'results.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
