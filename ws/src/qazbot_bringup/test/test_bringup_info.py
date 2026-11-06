"""Unit tests of the bringup guard/summary (no ROS graph)."""
import os
import subprocess
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


PROD_ENV = {'QAZBOT_MODE': 'production'}
GIT_ENV = dict(os.environ, GIT_AUTHOR_NAME='t', GIT_AUTHOR_EMAIL='t@t', GIT_COMMITTER_NAME='t',
               GIT_COMMITTER_EMAIL='t@t', GIT_CONFIG_NOSYSTEM='1', HOME=HERE)


def _git(repo, *a):
    return subprocess.run(['git', '-C', str(repo), *a], check=True, capture_output=True, text=True,
                          env=GIT_ENV).stdout.strip()


def _workspace(tmp_path, tag='x2-release-20261106-1', annotated=True, manifest=None, gitignore='ws/install/\n'):
    """Git repo <tmp>/repo with one commit (optional release tag) and ws/install/qazbot_build_manifest.yaml."""
    repo = tmp_path / 'repo'
    (repo / 'ws' / 'install').mkdir(parents=True)
    (repo / '.gitignore').write_text(gitignore)
    _git(repo, 'init', '-q')
    _git(repo, 'add', '.gitignore')
    _git(repo, 'commit', '-qm', 'init')
    if tag:
        _git(repo, 'tag', *(['-a', '-m', 'release'] if annotated else []), tag)
    sha = _git(repo, 'rev-parse', 'HEAD')
    body = manifest if manifest is not None else f'sha: {sha}\ndirty: false\nworkspace: {repo}\n'
    (repo / 'ws' / 'install' / B.MANIFEST).write_text(body.replace('{sha}', sha).replace('{repo}', str(repo)))
    return repo, str(repo / 'ws' / 'install')


def test_production_accepts_tagged_clean_inplace_build(tmp_path):
    repo, root = _workspace(tmp_path)
    assert not B.check_environment('production', 'lio', PROD_ENV, root=root)
    assert not B.check_environment('production', 'lio', dict(PROD_ENV, ROS_DOMAIN_ID='0', QAZBOT_INSTALL=root), root=root)
    assert not B.release_errors(root)
    assert B.release_tags(str(repo)) == ['x2-release-20261106-1']


def test_production_only_on_robot_domain(tmp_path):
    _, root = _workspace(tmp_path)
    errs = B.check_environment('production', 'lio', dict(PROD_ENV, ROS_DOMAIN_ID='88'), root=root)
    assert len(errs) == 1 and 'ROS_DOMAIN_ID' in errs[0]


def test_production_refuses_lab_env_other_install_and_overlays(tmp_path):
    _, root = _workspace(tmp_path)
    assert any('QAZBOT_MODE' in e for e in B.check_environment('production', 'lio', {}, root=root))
    errs = B.check_environment('production', 'lio', dict(PROD_ENV, QAZBOT_INSTALL=str(tmp_path / 'other')), root=root)
    assert errs and 'another install' in errs[0]
    for k in B.OVERLAYS:
        errs = B.check_environment('production', 'lio', dict(PROD_ENV, **{k: '/x/candidates/smac'}), root=root)
        assert errs and k in errs[0]


def test_production_refuses_without_manifest(tmp_path):
    repo, root = _workspace(tmp_path)
    os.remove(os.path.join(root, B.MANIFEST))
    errs = B.check_environment('production', 'lio', PROD_ENV, root=root)
    assert errs and 'no build manifest' in errs[0]


def test_production_refuses_install_outside_repo_ws(tmp_path):
    _workspace(tmp_path)
    other = tmp_path / 'prod' / 'install'          # e.g. a copied install tree, not <repo>/ws/install
    other.mkdir(parents=True)
    errs = B.check_environment('production', 'lio', PROD_ENV, root=str(other))
    assert errs and 'not <repo>/ws/install' in errs[0]
    repo = tmp_path / 'repo'
    (repo / 'build' / 'install').mkdir(parents=True)  # in the repo, but not ws/install
    assert B.check_environment('production', 'lio', PROD_ENV, root=str(repo / 'build' / 'install'))


def test_production_refuses_sha_mismatch(tmp_path):
    repo, root = _workspace(tmp_path)
    (repo / 'f').write_text('x')
    _git(repo, 'add', 'f')
    _git(repo, 'commit', '-qm', 'next')             # HEAD moved after the build
    _git(repo, 'tag', '-a', '-m', 'r', 'x2-release-20261106-2')
    errs = B.release_errors(root)
    assert len(errs) == 1 and '!= HEAD' in errs[0]


def test_production_refuses_dirty_manifest_or_tree(tmp_path):
    _, root = _workspace(tmp_path, manifest='sha: {sha}\ndirty: true\nworkspace: {repo}\n')
    errs = B.release_errors(root)
    assert len(errs) == 1 and 'dirty' in errs[0]
    repo, root = _workspace(tmp_path / 'b')
    (repo / '.gitignore').write_text('ws/install/\nchanged\n')     # tracked file modified
    errs = B.release_errors(root)
    assert len(errs) == 1 and 'not clean' in errs[0]
    repo, root = _workspace(tmp_path / 'c')
    (repo / 'untracked.py').write_text('')                          # untracked source counts as dirty
    assert any('not clean' in e for e in B.release_errors(root))
    _, root = _workspace(tmp_path / 'd', manifest='sha: {sha}\nworkspace: {repo}\n')
    assert any('dirty' in e for e in B.release_errors(root))


def test_production_refuses_manifest_built_elsewhere(tmp_path):
    _, root = _workspace(tmp_path, manifest='sha: {sha}\ndirty: false\nworkspace: /elsewhere/control_backend\n')
    errs = B.release_errors(root)
    assert len(errs) == 1 and 'workspace' in errs[0]


def test_production_requires_annotated_release_tag(tmp_path):
    for i, (tag, annotated) in enumerate([(None, True), ('x2-release-20261106-1', False), ('v1.0', True),
                                          ('x2-release-2026116-1', True), ('x2-release-20261106', True)]):
        _, root = _workspace(tmp_path / str(i), tag=tag, annotated=annotated)
        errs = B.release_errors(root)
        assert len(errs) == 1 and 'x2-release' in errs[0], (tag, annotated, errs)


def test_production_ignores_per_package_build_manifest(tmp_path):
    repo, root = _workspace(tmp_path)
    os.remove(os.path.join(root, B.MANIFEST))
    d = repo / 'ws' / 'install' / 'qazbot_bringup' / 'share' / 'qazbot_bringup'
    d.mkdir(parents=True)
    (d / 'build_manifest.yaml').write_text(f'sha: {_git(repo, "rev-parse", "HEAD")}\ndirty: false\nworkspace: {repo}\n')
    assert B.release_errors(root)


def test_release_check_cli(monkeypatch, capsys, tmp_path):
    _, root = _workspace(tmp_path)
    monkeypatch.setattr(B, 'install_root', lambda here=None: root)
    assert B.main(['--release-check']) == 0 and 'release: yes' in capsys.readouterr().out
    monkeypatch.setattr(B, 'install_root', lambda here=None: str(tmp_path / 'nothing' / 'install'))
    assert B.main(['--release-check']) == 1 and 'release: NO' in capsys.readouterr().out


def test_unknown_mode_and_odom_source_refused():
    assert B.check_environment('sim', 'lio', {'ROS_DOMAIN_ID': '88', 'ROS_LOCALHOST_ONLY': '1'})
    errs = B.check_environment('lab', 'native_imu', {'ROS_DOMAIN_ID': '88', 'ROS_LOCALHOST_ONLY': '1'})
    assert errs and 'native_imu' in errs[0]


def test_manifest_and_summary(tmp_path):
    m = tmp_path / 'manifest.yaml'
    m.write_text('sha: abc123\ndirty: false\nrelease_tag: x2-release-20261106-1\nthird_party:\n  fast_lio: x\n')
    path, data = B.read_manifest([None, str(tmp_path / 'missing.yaml'), str(m)])
    assert path == str(m) and data['sha'] == 'abc123' and 'fast_lio' not in data
    s = B.summary('production', 'x2_d1', 'lio', '/r/install', path, data, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud')
    for key in ('mode=production', 'git_sha=abc123', 'odom_source=lio', 'lidar_source=/aima/hal', 'tf_owner=',
                'control_channel=', 'LIVE', 'workspace=/r/install', 'release_tag=x2-release-20261106-1'):
        assert key in s
    assert 'DRY RUN' in B.summary('lab', 'x2_d1', 'lio', '/r', None, {}, 'x')


def test_workspace_manifest_preferred(tmp_path):
    c = B.manifest_candidates(str(tmp_path / 'ws' / 'install'))
    assert c[0] == str(tmp_path / 'ws' / 'install' / 'qazbot_build_manifest.yaml')


def test_main_refuses_with_nonzero(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    assert B.main(['--mode', 'lab', '--profile', 'x2_d1', '--odom-source', 'lio']) == 1
    assert 'REFUSED' in capsys.readouterr().err
