"""resume: Robot -> Lab synchronisation from the published robot baseline branch (no merge, checkpoint only after success)."""
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
LAB = REPO / 'tools' / 'nightly' / 'lab.py'
NQ = REPO / 'tools' / 'nightly' / 'nq.py'


def g(cwd, *a):
    r = subprocess.run(['git', '-C', str(cwd), '-c', 'user.name=t', '-c', 'user.email=t@t', *a], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip()


@pytest.fixture
def lab(tmp_path):
    remote = tmp_path / 'r.git'
    subprocess.run(['git', 'init', '-q', '--bare', '-b', 'work', str(remote)], check=True)
    w = tmp_path / 'w'
    subprocess.run(['git', 'clone', '-q', str(remote), str(w)], check=True, capture_output=True)
    g(w, 'checkout', '-q', '-b', 'work')
    for f, c in {'docs/nightly/STATUS.md': '#\n', '.gitignore': 'run/\n', 'ws/src/navigation/x2_motion_adapter.py': 'A=1\n', 'docs/HANDOFF.md': 'old handoff\n'}.items():
        (w / f).parent.mkdir(parents=True, exist_ok=True); (w / f).write_text(c)
    g(w, 'add', '-A'); g(w, 'commit', '-qm', 'lab base'); g(w, 'push', '-q', '-u', 'origin', 'work')
    e = dict(os.environ, NIGHTLY_ROOT=str(w), NIGHTLY_RUN=str(w / 'run' / 'nightly'), NIGHTLY_UNIT_DIR=str(tmp_path / 'u'), QAZBOT_ROLE='LAB',
             NIGHTLY_CLAUDE_BIN='/nonexistent', NIGHTLY_CODEX_BIN='/nonexistent')
    e.pop('NIGHTLY_ROBOT_REF', None)
    return w, remote, e, tmp_path


_n = __import__('itertools').count()


def publish_robot(remote, tmp, name, files, parent='work', msg='robot change'):
    import itertools
    d = tmp / f'robot_{name.replace("/", "_")}_{next(_n)}'
    subprocess.run(['git', 'clone', '-q', '-b', parent, str(remote), str(d)], check=True, capture_output=True)
    g(d, 'checkout', '-q', '-B', name)
    for f, c in files.items():
        (d / f).parent.mkdir(parents=True, exist_ok=True); (d / f).write_text(c)
    g(d, 'add', '-A'); g(d, 'commit', '-qm', msg); g(d, 'push', '-q', '-f', 'origin', f'{name}:{name}')
    return g(d, 'rev-parse', 'HEAD')


def resume(e, *extra):
    return subprocess.run([sys.executable, str(LAB), 'resume', '--no-systemd', '--no-start', *extra], capture_output=True, text=True, env=e)


def stop(e):
    return subprocess.run([sys.executable, str(LAB), 'stop', '--no-systemd', '--no-docker', '--no-notify', '--no-tests', '--no-push'], capture_output=True, text=True, env=e)


def tasks(w, like):
    c = sqlite3.connect(w / 'run' / 'nightly' / 'queue.db'); c.row_factory = sqlite3.Row
    return c.execute('SELECT * FROM tasks WHERE name LIKE ? ORDER BY name', (like,)).fetchall()


def kv(w, k):
    r = sqlite3.connect(w / 'run' / 'nightly' / 'queue.db').execute('SELECT v FROM kv WHERE k=?', (k,)).fetchone()
    return json.loads(r[0]) if r else None


def test_new_robot_commit_without_any_feedback_file_is_detected_and_creates_tasks(lab):
    w, remote, e, tmp = lab
    stop(e)
    tip = publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/navigation/x2_motion_adapter.py': 'A=2\n', 'ws/src/maps/m/map.yaml': 'res: 0.05\n'})
    assert not (w / 'docs/hardware/ROBOT_TO_LAB_FEEDBACK.md').exists()                  # no feedback file anywhere
    r = resume(e)
    assert 'ROBOT-SYNC origin/baseline/x2-robot-20261031' in r.stdout and 'FAILED' not in r.stdout, r.stdout
    names = [t['name'] for t in tasks(w, 'robot_parity_%')]
    assert f'robot_parity_navigation_{tip[:7]}' in names and f'robot_parity_maps_{tip[:7]}' in names
    t = tasks(w, 'robot_parity_navigation_%')[0]
    assert t['kind'] == 'engineering' and t['state'] == 'PENDING' and t['klass'] == 'SIM_THEN_HARDWARE' and 'x2_motion_adapter.py' in t['repro']
    assert kv(w, 'last_robot_processed_sha') == tip


def test_second_resume_is_a_noop_no_duplicates(lab):
    w, remote, e, tmp = lab
    stop(e)
    publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/lio/x.py': 'x\n'})
    resume(e); n1 = len(tasks(w, 'robot_%'))
    stop(e); r = resume(e)
    assert 'already processed' in r.stdout and len(tasks(w, 'robot_%')) == n1 == 1


def test_only_new_commits_after_the_last_processed_baseline_are_analysed(lab):
    w, remote, e, tmp = lab
    stop(e)
    publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/lio/x.py': 'x\n'})
    resume(e)
    stop(e)
    tip2 = publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/navigation/y.py': 'y\n'}, parent='baseline/x2-robot-20261031', msg='second robot change')
    resume(e)
    names = [t['name'] for t in tasks(w, 'robot_parity_%')]
    assert any(n.startswith('robot_parity_lio_') for n in names) and f'robot_parity_navigation_{tip2[:7]}' in names
    assert not any(n == f'robot_parity_lio_{tip2[:7]}' for n in names)                 # lio was not re-reported: only the delta since the checkpoint
    assert kv(w, 'last_robot_processed_sha') == tip2


def test_blockers_in_robot_handoff_and_feedback_become_priority_tasks(lab):
    w, remote, e, tmp = lab
    stop(e)
    publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'docs/HANDOFF.md': 'old handoff\nBLOCKER: AMCL jumps 1.72 m after initialpose\nfine line\n',
                  'docs/hardware/ROBOT_TO_LAB_FEEDBACK.md': '<!-- QAZBOT-CONTRACT {"contract_version": 1} -->\nTWIN_MISMATCH: yaw rate saturates at 0.45 m/s\n'})
    resume(e)
    bl = tasks(w, 'robot_blocker_%')
    assert len(bl) == 2 and all(t['priority'] == 10 and t['task_type'] == 'root_cause_analysis' for t in bl)
    assert any('AMCL jumps 1.72' in t['repro'] for t in bl) and any('yaw rate saturates' in t['repro'] for t in bl)
    assert all('fine line' not in t['repro'] for t in bl)


def test_diverged_robot_branch_is_analysed_read_only_and_never_merged(lab):
    w, remote, e, tmp = lab
    stop(e)
    (w / 'docs/nightly/STATUS.md').write_text('# lab moved on\n'); g(w, 'commit', '-qam', 'lab change'); g(w, 'push', '-q')
    head = g(w, 'rev-parse', 'HEAD')
    publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/navigation/z.py': 'z\n'}, parent='work')
    g(remote, 'update-ref', 'refs/heads/work', head)                                    # ensure the remote lab branch is the moved-on one
    r = resume(e)
    assert g(w, 'rev-parse', 'HEAD') == head and not (w / 'ws/src/navigation/z.py').exists()
    assert tasks(w, 'robot_parity_navigation_%') and 'FAILED' not in r.stdout


def test_failure_mid_processing_does_not_advance_checkpoint_and_retry_is_idempotent(lab):
    w, remote, e, tmp = lab
    stop(e)
    tip = publish_robot(remote, tmp, 'baseline/x2-robot-20261031', {'ws/src/lio/x.py': 'x\n'})
    r = resume(e, ) if False else subprocess.run([sys.executable, str(LAB), 'resume', '--no-systemd', '--no-start'], capture_output=True, text=True, env=dict(e, NIGHTLY_TEST_FAIL_ROBOT_SYNC='1'))
    assert 'checkpoint NOT advanced' in r.stdout and kv(w, 'last_robot_processed_sha') in (None, '')
    stop(e); r2 = resume(e)
    assert kv(w, 'last_robot_processed_sha') == tip and len(tasks(w, 'robot_parity_lio_%')) == 1        # no duplicate from the failed first pass


def test_no_robot_branch_published_is_not_an_error(lab):
    w, remote, e, tmp = lab
    stop(e)
    r = resume(e)
    assert r.returncode == 0 and 'no robot baseline ref' in r.stdout and not tasks(w, 'robot_%')


def test_stop_resume_cycle_and_zero_loss_state_unaffected(lab):
    w, remote, e, tmp = lab
    assert stop(e).returncode in (0, 3)
    r = resume(e)
    assert r.returncode == 0 and kv(w, 'frozen') is False and kv(w, 'lab_state') == 'RESUMED_NOT_STARTED'


def test_robot_stable_is_the_installed_state_and_origin_main_is_never_used(lab):
    w, remote, e, tmp = lab
    stop(e)
    publish_robot(remote, tmp, 'main', {'ws/src/navigation/x2_motion_adapter.py': 'FROM_MAIN\n'})        # a stale main must not be mistaken for the X2
    publish_robot(remote, tmp, 'baseline/x2-robot-20261029', {'ws/src/lio/old.py': 'o\n'})
    tip = publish_robot(remote, tmp, 'robot/stable', {'ws/src/navigation/x2_motion_adapter.py': 'INSTALLED\n'})
    r = resume(e)
    assert 'ROBOT-SYNC origin/robot/stable' in r.stdout
    assert kv(w, 'last_robot_processed_sha') == tip and not tasks(w, 'robot_parity_lio_%')


def test_hardware_candidate_results_from_robot_stable_become_tasks_and_pointers(lab):
    w, remote, e, tmp = lab
    stop(e)
    publish_robot(remote, tmp, 'robot/stable', {
        'docs/hardware/candidate_results/cand-x-1.json': json.dumps({'id': 'cand-x-1', 'status': 'HW_FAILED', 'task_id': 'x', 'result_sha': 'a' * 40, 'evidence': 'bag:/b/1 sha256:ab'}),
        'docs/hardware/candidate_results/cand-y-2.json': json.dumps({'id': 'cand-y-2', 'status': 'HW_VERIFIED', 'task_id': 'y', 'candidate_sha': 'c' * 40, 'result_sha': 'b' * 40, 'evidence': 'bag'})})
    resume(e)
    t = tasks(w, 'hw_candidate_failed_%')
    assert len(t) == 1 and t[0]['priority'] == 8 and 'bag:/b/1' in t[0]['repro'] and t[0]['task_type'] == 'root_cause_analysis'
    assert kv(w, 'last_hardware_verified_candidate_sha') == 'c' * 40
    stop(e); resume(e)
    assert len(tasks(w, 'hw_candidate_failed_%')) == 1                                              # no duplicate on the second resume


def test_stop_publishes_lab_stable_and_writes_all_five_pointers(lab):
    w, remote, e, tmp = lab
    g(w, 'branch', 'lab/stable')
    r = subprocess.run([sys.executable, str(LAB), 'stop', '--no-systemd', '--no-docker', '--no-notify', '--no-tests'], capture_output=True, text=True, env=e)
    assert r.returncode == 0 and 'PUBLISH lab/stable published' in r.stdout, r.stdout
    assert g(remote, 'rev-parse', 'lab/stable') == g(w, 'rev-parse', 'lab/stable')
    ptr = json.loads((w / 'docs/hardware/SYNC_POINTERS.json').read_text())
    for k in ('last_robot_processed_sha', 'last_robot_snapshot_sha', 'last_lab_published_sha', 'last_lab_reviewed_sha', 'last_hardware_verified_candidate_sha'):
        assert k in ptr
    assert kv(w, 'last_lab_published_sha') == g(w, 'rev-parse', 'lab/stable')
