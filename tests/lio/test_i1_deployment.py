"""Filesystem-only proof of backup, refusal and rollback; never accesses X2."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('i1_deployment', ROOT/'tools/deploy_i1.py')
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)


@pytest.fixture
def stage(tmp_path, monkeypatch):
    robot, source = tmp_path/'robot', tmp_path/'source'
    for rel in d.BASE:
        (robot/rel).parent.mkdir(parents=True, exist_ok=True)
        (source/rel).parent.mkdir(parents=True, exist_ok=True)
        before = subprocess.check_output(['git', '-C', str(ROOT), 'show',
            'c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa:'+rel])
        (robot/rel).write_bytes(before)
        shutil.copy2(ROOT/rel, source/rel)
    (source/'tools').mkdir()
    shutil.copy2(ROOT/'tools/i1_nav_udp_only.xml', source/'tools/i1_nav_udp_only.xml')
    backup = robot/'docs/evidence/test_deploy'
    monkeypatch.setattr(d, 'no_stack', lambda: None)
    def run(restore=False):
        argv = ['deploy_i1.py', '--root', str(robot), '--backup', str(backup)]
        argv += ['--restore'] if restore else ['--source', str(source), '--source-commit', 'a'*40]
        monkeypatch.setattr(sys, 'argv', argv)
        d.main()
    return robot, source, backup, run


def test_install_backup_and_exact_restore(stage):
    robot, source, backup, run = stage
    run()
    manifest = json.loads((backup/'deployment.json').read_text())
    assert manifest['status'] == 'INSTALLED_NOT_STARTED'
    for rel, before in d.BASE.items():
        assert d.sha(robot/rel) == d.sha(source/rel)
        assert d.sha(backup/'original'/rel) == before
    run(True)
    assert all(d.sha(robot/rel) == before for rel, before in d.BASE.items())


def test_modified_robot_baseline_is_not_overwritten(stage):
    robot, _, backup, run = stage
    target = robot/next(iter(d.BASE))
    target.write_bytes(target.read_bytes()+b'\n# external change\n')
    expected = target.read_bytes()
    with pytest.raises(RuntimeError, match='baseline changed'):
        run()
    assert target.read_bytes() == expected and not backup.exists()


def test_corrupt_backup_refuses_restore(stage):
    robot, source, backup, run = stage
    run()
    (backup/'original'/next(iter(d.BASE))).write_text('damaged')
    with pytest.raises(RuntimeError, match='Backup integrity'):
        run(True)
    assert all(d.sha(robot/rel) == d.sha(source/rel) for rel in d.BASE)


def test_third_party_change_refuses_restore(stage):
    robot, _, _, run = stage
    run()
    target = robot/next(iter(d.BASE))
    target.write_bytes(target.read_bytes()+b'\n# external change\n')
    expected = target.read_bytes()
    with pytest.raises(RuntimeError, match='third-party'):
        run(True)
    assert target.read_bytes() == expected


def test_partial_install_failure_rolls_back(stage, monkeypatch):
    robot, _, backup, run = stage
    original = d.replace
    failed = False
    def once(path, data):
        nonlocal failed
        if path.name == 'nav_control.launch.py' and not failed:
            failed = True
            raise OSError('injected second-file write failure')
        original(path, data)
    monkeypatch.setattr(d, 'replace', once)
    with pytest.raises(OSError):
        run()
    assert all(d.sha(robot/rel) == before for rel, before in d.BASE.items())
    assert json.loads((backup/'deployment.json').read_text())['status'] == 'ROLLED_BACK_AFTER_INSTALL_ERROR'
