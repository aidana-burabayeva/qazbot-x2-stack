"""Unit tests of the bringup guard/summary (no ROS graph)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import bringup_info as B  # noqa: E402

LOOPBACK = os.path.join(HERE, '..', 'config', 'dds_loopback.xml')
ROBOT = os.path.join(HERE, '..', 'config', 'dds_qazbot.xml')


def test_lab_refuses_domain_zero_and_unset():
    assert B.check_environment('lab', 'lio', {'ROS_LOCALHOST_ONLY': '1'})
    assert B.check_environment('lab', 'lio', {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'})


def test_lab_needs_host_local_graph():
    assert B.check_environment('lab', 'lio', {'ROS_DOMAIN_ID': '88'})
    assert B.check_environment('lab', 'lio', {'ROS_DOMAIN_ID': '88', 'FASTRTPS_DEFAULT_PROFILES_FILE': ROBOT})
    assert not B.check_environment('lab', 'lio', {'ROS_DOMAIN_ID': '88', 'ROS_LOCALHOST_ONLY': '1'})
    assert not B.check_environment('lab', 'lio', {'ROS_DOMAIN_ID': '88', 'FASTRTPS_DEFAULT_PROFILES_FILE': LOOPBACK})


SHA40 = 'a' * 40


def _release(tmp_path, body):
    rel = tmp_path / 'rel'
    (rel / 'install').mkdir(parents=True)
    (rel / 'manifest.yaml').write_text(body)
    return str(rel / 'install')


def test_production_only_on_robot_domain(tmp_path):
    root = _release(tmp_path, f'sha: {SHA40}\ndirty: false\n')
    assert not B.check_environment('production', 'lio', {}, root=root)
    assert not B.check_environment('production', 'lio', {'ROS_DOMAIN_ID': '0'}, root=root)
    assert B.check_environment('production', 'lio', {'ROS_DOMAIN_ID': '88'}, root=root)


def test_production_accepts_env_manifest(tmp_path):
    m = tmp_path / 'm.yaml'
    m.write_text(f'sha: {SHA40}\ndirty: false\n')
    assert not B.check_environment('production', 'lio', {'QAZBOT_RELEASE_MANIFEST': str(m)}, root=str(tmp_path / 'x' / 'install'))


def test_production_refuses_without_release_manifest(tmp_path):
    errs = B.check_environment('production', 'lio', {}, root=str(tmp_path / 'nothing' / 'install'))
    assert errs and 'release' in errs[0]


def test_production_refuses_dirty_or_bad_sha(tmp_path):
    assert B.check_environment('production', 'lio', {}, root=_release(tmp_path, f'sha: {SHA40}\ndirty: true\n'))
    assert B.check_environment('production', 'lio', {}, root=_release(tmp_path / 'b', 'sha: abc123\ndirty: false\n'))
    assert B.check_environment('production', 'lio', {}, root=_release(tmp_path / 'c', f'sha: {SHA40}\n'))


def test_production_never_falls_back_to_dev_build_manifest(tmp_path):
    root = tmp_path / 'install'
    d = root / 'qazbot_bringup' / 'share' / 'qazbot_bringup'
    d.mkdir(parents=True)
    (d / 'build_manifest.yaml').write_text(f'sha: {SHA40}\ndirty: false\n')
    assert B.check_environment('production', 'lio', {}, root=str(root))


def test_unknown_mode_and_odom_source_refused():
    assert B.check_environment('sim', 'lio', {'ROS_DOMAIN_ID': '88', 'ROS_LOCALHOST_ONLY': '1'})
    errs = B.check_environment('lab', 'native_imu', {'ROS_DOMAIN_ID': '88', 'ROS_LOCALHOST_ONLY': '1'})
    assert errs and 'native_imu' in errs[0]


def test_manifest_and_summary(tmp_path):
    m = tmp_path / 'manifest.yaml'
    m.write_text('sha: abc123\ndirty: false\nprofile: x2_d1\nthird_party:\n  fast_lio: x\n')
    path, data = B.read_manifest([None, str(tmp_path / 'missing.yaml'), str(m)])
    assert path == str(m) and data['sha'] == 'abc123' and 'fast_lio' not in data
    s = B.summary('production', 'x2_d1', 'lio', '/r/install', path, data, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud')
    for key in ('mode=production', 'git_sha=abc123', 'odom_source=lio', 'lidar_source=/aima/hal', 'tf_owner=',
                'control_channel=', 'LIVE', 'workspace=/r/install'):
        assert key in s
    assert 'DRY RUN' in B.summary('lab', 'x2_d1', 'lio', '/r', None, {}, 'x')


def test_release_manifest_preferred(tmp_path):
    root = tmp_path / 'releases' / 'abc' / 'install'
    c = B.manifest_candidates(str(root), env={})
    assert c[1] == str(tmp_path / 'releases' / 'abc' / 'manifest.yaml')


def test_main_refuses_with_nonzero(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    assert B.main(['--mode', 'lab', '--profile', 'x2_d1', '--odom-source', 'lio']) == 1
    assert 'REFUSED' in capsys.readouterr().err
