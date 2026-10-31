"""Bootstrap gates without Docker, systemd or live AI providers."""
import sqlite3
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools/nightly'
sys.path.insert(0, str(TOOLS))
import labctl
import preflight


@pytest.fixture
def db(tmp_path):
    c = sqlite3.connect(':memory:', isolation_level=None)
    c.row_factory = sqlite3.Row
    c.executescript(labctl.nq.SCHEMA)
    c.executescript('ALTER TABLE tasks ADD COLUMN base_from TEXT; ALTER TABLE tasks ADD COLUMN task_type TEXT; ALTER TABLE tasks ADD COLUMN complexity TEXT; ALTER TABLE tasks ADD COLUMN risk TEXT; ALTER TABLE tasks ADD COLUMN requires_result INTEGER DEFAULT 0;')
    return c


def test_campaign_idempotent_and_no_legacy_retry(db, monkeypatch):
    monkeypatch.setattr(labctl.nq, 'checkpoint', lambda *a: None)
    monkeypatch.setattr(labctl.lab, 'gout', lambda *a: labctl.REVIEWED_ROBOT)
    monkeypatch.setattr(labctl.preflight, 'integration_baseline_matches', lambda: True)
    monkeypatch.setattr(labctl.nq.mergeq, 'integration_head', lambda *a: 'integration')
    labctl.nq.enqueue(db, 'verify_worker_kill', 'false', state='NEEDS_ENGINEERING', reason='exit -9')
    for _ in range(2):
        labctl.prepare_campaign(db)
    assert db.execute('SELECT COUNT(*) FROM tasks').fetchone()[0] == 14
    assert db.execute("SELECT state,reason FROM tasks WHERE name='verify_worker_kill'").fetchone()[:] == ('NEEDS_ENGINEERING', 'exit -9')
    assert labctl.nq.kv_get(db, 'last_robot_processed_sha') == labctl.REVIEWED_ROBOT
    assert db.execute("SELECT name FROM tasks WHERE state='PENDING' ORDER BY priority LIMIT 1").fetchone()[0] .startswith('fidelity_audit_')


def test_future_robot_snapshot_never_marked_reviewed(db, monkeypatch):
    monkeypatch.setattr(labctl.nq, 'checkpoint', lambda *a: None)
    monkeypatch.setattr(labctl.lab, 'gout', lambda *a: 'future-unreviewed-sha')
    labctl.prepare_campaign(db)
    assert labctl.nq.kv_get(db, 'last_robot_processed_sha') is None


def test_failed_preflight_does_not_start_or_enqueue(monkeypatch):
    monkeypatch.setattr(labctl.lab, 'sh', lambda *a: (3, 'inactive'))
    monkeypatch.setattr(labctl.nq.mergeq, 'checked_out_at', lambda *a: None)
    monkeypatch.setattr(labctl.lab, 'sync_control', lambda *a: None)
    monkeypatch.setattr(labctl, 'doctor', lambda *args: 1)
    monkeypatch.setattr(labctl, 'call', lambda *a: pytest.fail('resume called after failed preflight'))
    monkeypatch.setattr(labctl.nq, 'connect', lambda: pytest.fail('queue modified after failed preflight'))
    assert labctl.start() == 1


def test_doctor_only_runs_preflight(monkeypatch):
    calls = []
    monkeypatch.setattr(labctl, 'call', lambda *a: calls.append(a) or 0)
    assert labctl.doctor() == 0
    assert calls == [(sys.executable, str(labctl.ROOT / 'tools/nightly/preflight.py'), 'all', '--tests', '--mode', 'ENGINEERING')]


def test_integration_checkout_refused(monkeypatch):
    monkeypatch.setattr(labctl.lab, 'sh', lambda *a: (3, 'inactive'))
    monkeypatch.setattr(labctl.nq.mergeq, 'checked_out_at', lambda *a: '/operator')
    monkeypatch.setattr(labctl, 'doctor', lambda *args: pytest.fail('unsafe integration checkout'))
    assert labctl.start() == 1


@pytest.mark.parametrize('key,robot,sim,ok', [
    ('odom_source', 'native_imu', 'lio', True),
    ('odom_source', 'native_imu', 'truth', False),
    ('odom_topic', '/qazbot_native/odom', '/arbitrary', False),
    ('jump_xy_max', 1.0, .10, True),
    ('jump_xy_max', 1.0, .20, False),
    ('jump_xy_max', 1.0, -1, False),
    ('jump_yaw_max', .5, float('nan'), False),
    ('jump_yaw_max', .5, True, False),
])
def test_parity_exceptions_are_bounded(key, robot, sim, ok):
    assert preflight.allowed_difference('/qazbot_d1_map/localization_trust', key, robot, sim) is ok


def test_start_success_prepares_once_then_existing_resume(tmp_path, db, monkeypatch):
    monkeypatch.setattr(labctl.lab, 'sh', lambda *a: (3, 'inactive'))
    monkeypatch.setattr(labctl.nq.mergeq, 'checked_out_at', lambda *a: None)
    monkeypatch.setattr(labctl.lab, 'sync_control', lambda *a: None)
    monkeypatch.setattr(labctl, 'doctor', lambda *args: 0)
    monkeypatch.setattr(labctl, 'RUN', tmp_path)
    (tmp_path / 'preflight').mkdir()
    (tmp_path / 'preflight/latest.json').write_text('{"git_head":"head","integration":"integration","mode":"ENGINEERING","verdict":"PASS","stamp":"test"}')
    monkeypatch.setattr(labctl.lab, 'gout', lambda *a: 'head')
    monkeypatch.setattr(labctl.nq.mergeq, 'integration_head', lambda *a: 'integration')
    monkeypatch.setattr(labctl.nq, 'connect', lambda: db)
    prepared, calls = [], []
    monkeypatch.setattr(labctl, 'prepare_campaign', lambda db,*args: prepared.append(True))
    monkeypatch.setattr(labctl, 'runtime_qualification_errors', lambda db: [])
    monkeypatch.setattr(labctl, 'call', lambda *a: calls.append(a) or 0)
    assert labctl.start() == 0
    assert prepared == [True]
    assert calls == [(sys.executable, str(labctl.ROOT / 'tools/nightly/lab.py'), 'resume', '--no-fetch')]


def test_start_ref_moved_during_doctor_no_queue_mutation(tmp_path, monkeypatch):
    monkeypatch.setattr(labctl.lab, 'sh', lambda *a: (3, 'inactive'))
    monkeypatch.setattr(labctl.nq.mergeq, 'checked_out_at', lambda *a: None)
    monkeypatch.setattr(labctl.lab, 'sync_control', lambda *a: None)
    monkeypatch.setattr(labctl, 'doctor', lambda *args: 0)
    monkeypatch.setattr(labctl, 'RUN', tmp_path)
    (tmp_path / 'preflight').mkdir()
    (tmp_path / 'preflight/latest.json').write_text('{"git_head":"old","integration":"old"}')
    monkeypatch.setattr(labctl.lab, 'gout', lambda *a: 'moved')
    monkeypatch.setattr(labctl.nq, 'connect', lambda: pytest.fail('queue changed after stale preflight'))
    assert labctl.start() == 1


def test_stop_compose_only_from_lab_worktrees(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(labctl.lab, 'ROOT', tmp_path)
    calls = []
    outputs = iter([
        (0, 'lab-owned\t' + str(tmp_path / 'run/attempts/a/digital_twin') + '\nforeign\t/other/workspace'),
        (0, 'removed'),
        (0, 'foreign'),
    ])
    monkeypatch.setattr(labctl.lab, 'sh', lambda cmd, **kw: calls.append(cmd) or next(outputs))
    result = labctl.lab.stop_sim(SimpleNamespace(no_docker=False))
    assert result == {'before': ['lab-owned'], 'remaining': [], 'error': None}
    assert calls[1][0:4] == ['docker', 'compose', '-p', 'lab-owned']
    assert 'foreign' not in calls[1]


def test_stop_compose_unavailable_is_unverified(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(labctl.lab, 'sh', lambda *a, **kw: (1, 'permission denied'))
    result = labctl.lab.stop_sim(SimpleNamespace(no_docker=False))
    assert result['remaining'] == 'UNKNOWN' and result['error']


def test_second_daemon_cannot_erase_first_owner_pid(tmp_path):
    import fcntl
    import os
    import subprocess
    run = tmp_path / 'run'
    run.mkdir()
    lock = run / 'orchestrator.lock'
    lock.write_text('123456')
    with lock.open('r+') as fh:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        env = dict(os.environ, NIGHTLY_ROOT=str(tmp_path), NIGHTLY_RUN=str(run))
        p = subprocess.run([sys.executable, str(TOOLS / 'nq.py'), 'daemon', '--max-seconds', '1'],
                           capture_output=True, text=True, env=env, timeout=10)
        assert p.returncode != 0 and 'another orchestrator' in p.stderr
        assert lock.read_text() == '123456'


def test_preflight_does_not_use_operator_backend_or_truth_overrides(monkeypatch):
    monkeypatch.setenv('TWIN_BACKEND', '/foreign/backend')
    monkeypatch.setenv('TWIN_MAP_DIR', '/foreign/map')
    monkeypatch.setenv('TWIN_LOCALIZATION', 'truth')
    seen = []
    monkeypatch.setattr(preflight, 'sh', lambda argv, **kw: seen.append(kw['env']) or (0, ''))
    preflight.dc('config')
    assert seen[0]['TWIN_BACKEND'] == str(preflight.ROOT)
    assert seen[0]['TWIN_MAP_DIR'] == str(preflight.ROOT / 'ws/src/maps/d1_qazbot_20261029')
    assert seen[0]['TWIN_LOCALIZATION'] == 'amcl'


def test_no_runtime_receipt_cannot_authorize_nightly_start(tmp_path, db):
    import qualification
    assert qualification.check(db,tmp_path/'missing',tmp_path,'git','robot','map','stack','sim') == ['missing_runtime_e2e_receipt']


def test_pass_doctor_without_e2e_receipt_cannot_enqueue_or_resume(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(labctl.lab,'sh',lambda *a:(3,'inactive'))
    monkeypatch.setattr(labctl.nq.mergeq,'checked_out_at',lambda *a:None)
    monkeypatch.setattr(labctl.lab,'sync_control',lambda *a:None)
    monkeypatch.setattr(labctl,'doctor',lambda *args:0)
    monkeypatch.setattr(labctl,'RUN',tmp_path)
    (tmp_path/'preflight').mkdir()
    (tmp_path/'preflight/latest.json').write_text('{"git_head":"head","integration":"integration","mode":"ENGINEERING","verdict":"PASS","stamp":"test"}')
    monkeypatch.setattr(labctl.lab,'gout',lambda *a:'head')
    monkeypatch.setattr(labctl.nq.mergeq,'integration_head',lambda *a:'integration')
    monkeypatch.setattr(labctl.nq,'connect',lambda:SimpleNamespace(close=lambda:None))
    monkeypatch.setattr(labctl,'runtime_qualification_errors',lambda db:['missing_runtime_e2e_receipt'])
    monkeypatch.setattr(labctl,'prepare_campaign',lambda *a:pytest.fail('unqualified campaign was enqueued'))
    monkeypatch.setattr(labctl,'call',lambda *a:pytest.fail('unqualified daemon was resumed'))
    assert labctl.start('FULL_NIGHTLY')==1
