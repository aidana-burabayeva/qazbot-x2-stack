"""lab.py stop/resume against a throw-away remote + clone. No systemd, docker, AI agent or robot involved."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

LAB = Path(__file__).resolve().parents[2] / 'tools' / 'nightly' / 'lab.py'
NQ = LAB.with_name('nq.py')


def g(cwd, *a):
    return subprocess.run(['git', '-C', str(cwd), *a], capture_output=True, text=True, check=True).stdout.strip()


@pytest.fixture
def env(tmp_path):
    remote = tmp_path / 'remote.git'
    subprocess.run(['git', 'init', '-q', '--bare', '-b', 'work', str(remote)], check=True)
    w = tmp_path / 'w'
    subprocess.run(['git', 'clone', '-q', str(remote), str(w)], check=True, capture_output=True)
    g(w, 'config', 'user.email', 't@t'); g(w, 'config', 'user.name', 't')
    (w / 'docs' / 'nightly').mkdir(parents=True)
    (w / 'docs' / 'nightly' / 'STATUS.md').write_text('# s\n')
    (w / '.gitignore').write_text('run/\n')
    g(w, 'checkout', '-q', '-b', 'work'); g(w, 'add', '-A'); g(w, 'commit', '-qm', 'init'); g(w, 'push', '-q', '-u', 'origin', 'work')
    e = dict(os.environ, NIGHTLY_ROOT=str(w), NIGHTLY_RUN=str(w / 'run' / 'nightly'), NIGHTLY_UNIT_DIR=str(tmp_path / 'units'))
    return w, remote, e, tmp_path


def lab(e, *a):
    return subprocess.run([sys.executable, str(LAB), *a], capture_output=True, text=True, env=e)


def nqcmd(e, *a):
    return subprocess.run([sys.executable, str(NQ), *a], capture_output=True, text=True, env=e)


STOP = ('stop', '--no-systemd', '--no-docker', '--no-notify', '--no-tests')
RES = ('resume', '--no-systemd')


def kv(w, k):
    import sqlite3
    r = sqlite3.connect(w / 'run' / 'nightly' / 'queue.db').execute('SELECT v FROM kv WHERE k=?', (k,)).fetchone()
    return json.loads(r[0]) if r else None


def test_stop_publishes_and_verifies(env):
    w, remote, e, _ = env
    nqcmd(e, 'enqueue', 'a', 'true'); nqcmd(e, 'enqueue', 'b', 'true')
    (w / 'run').mkdir(exist_ok=True); (w / 'run' / 'x.log').write_text('runtime')
    r = lab(e, *STOP)
    assert r.returncode == 0, r.stdout + r.stderr
    assert kv(w, 'lab_state') == 'STOPPED_OK' and kv(w, 'frozen') is True
    assert kv(w, 'codex_stop') == 'STOP_UNVERIFIED'
    head = g(w, 'rev-parse', 'HEAD')
    assert g(remote, 'rev-parse', 'work') == head and kv(w, 'last_lab_stop_sha') == head
    assert (w / 'docs/hardware/LAB_TO_ROBOT_HANDOFF.md').exists() and (w / 'docs/hardware/TWIN_FIDELITY_REQUESTS.md').exists()
    assert 'lab-stop/' in g(remote, 'tag')
    assert 'run/' not in g(w, 'ls-files') and g(remote, 'ls-tree', '-r', '--name-only', 'work').count('x.log') == 0
    assert 'NEEDS_APPROVAL' in nqcmd(e, 'status').stdout


def test_frozen_queue_does_not_start_tasks(env):
    w, _, e, _ = env
    nqcmd(e, 'enqueue', 'a', 'touch ' + str(w / 'ran'))
    lab(e, *STOP)
    nqcmd(e, 'enqueue', 'late', 'touch ' + str(w / 'ran_late'))
    subprocess.run([sys.executable, str(NQ), 'daemon', '--poll', '0.2'], env=e, timeout=3, capture_output=True) if False else None
    import sqlite3
    sys.path.insert(0, str(NQ.parent))
    os.environ.update(NIGHTLY_ROOT=e['NIGHTLY_ROOT'], NIGHTLY_RUN=e['NIGHTLY_RUN'])
    import importlib, nq
    importlib.reload(nq)
    db = nq.connect()
    assert list(nq.ready(db)) == []
    assert not (w / 'ran').exists() and not (w / 'ran_late').exists()


def test_stop_idempotent_second_run(env):
    w, _, e, _ = env
    assert lab(e, *STOP).returncode == 0
    assert lab(e, *STOP).returncode == 0
    assert g(w, 'status', '--porcelain') == ''


def test_stop_without_reachable_remote_reports_publish_pending(env):
    w, remote, e, tmp = env
    g(w, 'remote', 'set-url', 'origin', str(tmp / 'nonexistent.git'))
    r = lab(e, *STOP)
    assert r.returncode == 3 and kv(w, 'lab_state') == 'STOPPED_PUBLISH_PENDING'
    assert g(w, 'log', '-1', '--format=%s').startswith('lab-stop:')  # local state kept


def test_stop_does_not_touch_foreign_dirty_files_and_refuses_secrets(env):
    w, _, e, _ = env
    (w / 'ws').mkdir(); (w / 'ws' / 'other.py').write_text('someone else\n')
    assert lab(e, *STOP).returncode == 0
    assert (w / 'ws' / 'other.py').read_text() == 'someone else\n'
    assert 'ws/' in g(w, 'status', '--porcelain')
    (w / 'docs' / 'leak.md').write_text('pass' + 'word = hunter2' + 'hunter2\n')
    r = lab(e, *STOP)
    assert r.returncode != 0 and 'secret' in (r.stdout + r.stderr)
    assert 'leak.md' not in g(w, 'show', '--name-only', '--format=', 'HEAD')


def test_resume_pulls_new_robot_commit_and_triages(env):
    w, remote, e, tmp = env
    lab(e, *STOP)
    stop_sha = kv(w, 'last_lab_stop_sha')
    other = tmp / 'robot'
    subprocess.run(['git', 'clone', '-q', '-b', 'work', str(remote), str(other)], check=True, capture_output=True)
    g(other, 'config', 'user.email', 'r@r'); g(other, 'config', 'user.name', 'r')
    (other / 'docs' / 'hardware').mkdir(parents=True, exist_ok=True)
    (other / 'docs' / 'hardware' / 'ROBOT_FINDINGS.md').write_text('stall at door\n')
    g(other, 'add', '-A'); g(other, 'commit', '-qm', 'robot: stall blocker at doorway'); g(other, 'push', '-q')
    r = lab(e, *RES)
    assert r.returncode == 0, r.stdout + r.stderr
    assert (w / 'docs/hardware/ROBOT_FINDINGS.md').exists()
    assert kv(w, 'frozen') is False and kv(w, 'current_integration_sha') == g(w, 'rev-parse', 'HEAD')
    assert f'DIFF {stop_sha[:9]}' in r.stdout and 'triage_robot_findings' in nqcmd(e, 'status').stdout
    assert 'SIM_THEN_HARDWARE' in r.stdout


def test_resume_diverged_never_rewrites_and_creates_integration_task(env):
    w, remote, e, tmp = env
    lab(e, *STOP)
    other = tmp / 'robot'
    subprocess.run(['git', 'clone', '-q', '-b', 'work', str(remote), str(other)], check=True, capture_output=True)
    g(other, 'config', 'user.email', 'r@r'); g(other, 'config', 'user.name', 'r')
    (other / 'r.txt').write_text('r'); g(other, 'add', '-A'); g(other, 'commit', '-qm', 'robot change'); g(other, 'push', '-q')
    (w / 'docs' / 'nightly' / 'STATUS.md').write_text('# local\n'); g(w, 'commit', '-qam', 'local lab change')
    local = g(w, 'rev-parse', 'HEAD')
    r = lab(e, *RES)
    assert g(w, 'rev-parse', 'HEAD') == local and not (w / 'r.txt').exists()
    assert 'diverged' in r.stdout and 'integrate_' in nqcmd(e, 'status').stdout
    assert kv(w, 'frozen') is False  # independent sim work continues


def test_resume_dirty_tracked_worktree_skips_pull(env):
    w, remote, e, tmp = env
    lab(e, *STOP)
    other = tmp / 'robot'
    subprocess.run(['git', 'clone', '-q', '-b', 'work', str(remote), str(other)], check=True, capture_output=True)
    g(other, 'config', 'user.email', 'r@r'); g(other, 'config', 'user.name', 'r')
    (other / 'r.txt').write_text('r'); g(other, 'add', '-A'); g(other, 'commit', '-qm', 'robot change'); g(other, 'push', '-q')
    (w / 'docs' / 'nightly' / 'STATUS.md').write_text('# dirty edit\n')
    r = lab(e, *RES)
    assert 'pull skipped' in r.stdout and not (w / 'r.txt').exists()
    assert (w / 'docs' / 'nightly' / 'STATUS.md').read_text() == '# dirty edit\n'


def test_resume_restores_parked_tasks_and_independent_tasks_survive_hardware_block(env):
    w, _, e, _ = env
    nqcmd(e, 'enqueue', 'p1', 'true'); nqcmd(e, 'enqueue', 'hw', 'true', '--state', 'NEEDS_HARDWARE')
    lab(e, *STOP)
    assert 'NEEDS_APPROVAL' in nqcmd(e, 'status').stdout
    lab(e, *RES)
    st = nqcmd(e, 'status').stdout
    assert any(ln.split()[1] == 'PENDING' and 'p1' in ln for ln in st.splitlines())
    assert any(ln.split()[1] == 'NEEDS_HARDWARE' for ln in st.splitlines())
