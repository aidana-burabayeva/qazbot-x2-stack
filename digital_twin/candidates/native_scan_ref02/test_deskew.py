import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from deskew import deskew


def input_case(v=0.3, omega=0.6):
    offsets = np.linspace(332000, 95181000, 200)
    times = 100 + offsets * 1e-9
    samples = 100 + np.arange(-2, 20) / 166
    world = np.column_stack([np.full(200, 3.), np.linspace(-1.5, 1.5, 200), np.linspace(-.4, .7, 200)])
    true_r = Rotation.from_euler('z', omega * (times - 100))
    true_p = np.column_stack([v * (times - 100), np.zeros((200, 2))])
    raw = true_r.inv().apply(world - true_p)
    q = Rotation.from_euler('z', omega * (samples - 100)).as_quat()
    p = np.column_stack([v * (samples - 100), np.zeros((len(samples), 2))])
    ref = times[-1]
    expected = Rotation.from_euler('z', omega * (ref - 100)).inv().apply(world - [v * (ref - 100), 0, 0])
    return dict(points=raw, offsets_ns=offsets, header_stamp=100., pose_stamps=samples,
                positions=p, quaternions=q, rotation_imu_lidar=np.eye(3), translation_imu_lidar=np.zeros(3),
                reference_stamp=ref, max_pose_gap_s=.02, max_linear_rate_mps=2., max_angular_rate_radps=5.), expected


@pytest.mark.parametrize('v,omega', [(0, 0), (.3, 0), (0, .6), (0, -.6), (.3, .6)])
def test_known_static_geometry_recovered(v, omega):
    args, expected = input_case(v, omega)
    result = deskew(**args)
    assert np.max(np.linalg.norm(result.points - expected, axis=1)) < 1e-10
    assert result.reference_stamp == args['reference_stamp']


def test_nontrivial_calibration_and_quaternion_signs():
    args, expected = input_case()
    r = Rotation.from_euler('xyz', [.1, -.2, 1.1]).as_matrix()
    t = np.array([.004, -.015, -.011])
    args['points'] = (args['points'] - t) @ r
    args['rotation_imu_lidar'], args['translation_imu_lidar'] = r, t
    args['quaternions'][::2] *= -1
    assert np.max(np.abs(deskew(**args).points - expected)) < 1e-10


@pytest.mark.parametrize(
    'fault',
    ['gap', 'missing_end', 'missing_start', 'duplicate', 'backward', 'zero_quaternion',
     'nan_quaternion', 'nan_points', 'negative_offset', 'position_reset', 'rotation_jump'],
)
def test_faults_refuse_output(fault):
    args, _ = input_case()
    if fault == 'gap':
        keep = (args['pose_stamps'] < 100.03) | (args['pose_stamps'] > 100.08)
        for key in ('pose_stamps', 'positions', 'quaternions'):
            args[key] = args[key][keep]
    elif fault == 'missing_end':
        args['reference_stamp'] = 101.
    elif fault == 'missing_start':
        args['header_stamp'] = 99.
    elif fault == 'duplicate':
        args['pose_stamps'][5] = args['pose_stamps'][4]
    elif fault == 'backward':
        args['pose_stamps'][5] = args['pose_stamps'][4] - .01
    elif fault == 'zero_quaternion':
        args['quaternions'][4] = 0
    elif fault == 'nan_quaternion':
        args['quaternions'][4, 0] = np.nan
    elif fault == 'nan_points':
        args['points'][0, 0] = np.nan
    elif fault == 'position_reset':
        args['positions'][5:, 0] += 10.
    elif fault == 'rotation_jump':
        args['quaternions'][5] = Rotation.from_euler('z', 1.5).as_quat()
    else:
        args['offsets_ns'][0] = -1
    with pytest.raises(ValueError):
        deskew(**args)


def test_epoch_stamps_preserved():
    args, expected = input_case()
    epoch = 1793104900.
    args['header_stamp'] += epoch
    args['pose_stamps'] += epoch
    args['reference_stamp'] += epoch
    result = deskew(**args)
    assert result.reference_stamp == args['reference_stamp']
    assert np.max(np.abs(result.points - expected)) < 1e-6
