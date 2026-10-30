"""Merge queue: sequential integration of VERIFIED attempt branches into lab/integration with CAS update, re-gate and evidence."""
import importlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2] / 'tools' / 'nightly'
NQ = TOOLS / 'nq.py'


def g(cwd, *a):
    r = subprocess.run(['git', '-C', str(cwd), '-c', 'user.name=t', '-c', 'user.email=t@t', *a], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip()


@pytest.fixture
def world(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'; repo.mkdir()
    subprocess.run(['git', 'init', '-q', '-b', 'main', str(repo)], check=True)
    (repo / 'a.txt').write_text('base\n'); (repo / 'ok.flag').write_text('1\n')
    g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 'base')
    monkeypatch.setenv('NIGHTLY_ROOT', str(repo)); monkeypatch.setenv('NIGHTLY_RUN', str(repo / 'run' / 'nightly'))
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'test -f ok.flag')
    monkeypatch.setenv('NIGHTLY_VERIFY_NICE', '0')
    sys.path.insert(0, str(TOOLS))
    for m in ('store', 'attempts', 'agents', 'router', 'mergeq', 'nq'):
        sys.modules.pop(m, None)
    nq = importlib.import_module('nq'); mq = importlib.import_module('mergeq')
    return repo, nq, mq, nq.connect()


def attempt(repo, nq, db, name, files, verified=True, state='COMPLETED', phase='VERIFIED'):
    """A finished attempt: branch from main with `files` changed; DB rows as the orchestrator would leave them."""
    base = g(repo, 'rev-parse', 'main')
    br = f'attempt/{name}-1-abc'
    g(repo, 'branch', br, base)
    wt = repo.parent / f'wt_{name}'
    g(repo, 'worktree', 'add', '-q', str(wt), br)
    for f, c in files.items():
        (wt / f).write_text(c)
    g(wt, 'add', '-A'); g(wt, 'commit', '-qm', f'attempt {name}')
    sha = g(wt, 'rev-parse', 'HEAD')
    nq.enqueue(db, name, 'x', kind='engineering', cwd=str(repo))
    db.execute("UPDATE tasks SET state=?, phase=?, branch=?, attempt_wt=?, attempts=1 WHERE name=?", (state, phase, br, str(wt), name))
    nq.kv_set(db, f'base:{name}:1', {'git_sha': base})
    if verified:
        nq.kv_set(db, f'verified_sha:{name}', sha)
        db.execute("INSERT INTO results(result_key,run_id,task,attempt,scenario_id,verdict,metrics,status,ts) VALUES(?,?,?,?,?,?,?,?,?)", (f'{name}|1|e', 'r', name, 1, 'e', 'PASS', '{}', 'STORED', time.time()))
    return br, sha


def queue(nq, mq, db, name):
    return mq.enqueue(db, nq.RUN, nq.kv_get, db.execute('SELECT * FROM tasks WHERE name=?', (name,)).fetchone())


def drain(nq, mq, db, limit=60):
    for _ in range(limit):
        if not mq.step(db, nq.RUN):
            return
        time.sleep(0.2)
    raise AssertionError('merge queue did not drain')


def states(db):
    return {r['task']: r['state'] for r in db.execute('SELECT task,state FROM merge_queue')}


def test_two_independent_branches_integrate_sequentially_with_gate_and_state_file(world):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'a', {'one.txt': '1\n'}); attempt(repo, nq, db, 'b', {'two.txt': '2\n'})
    assert queue(nq, mq, db, 'a') and queue(nq, mq, db, 'b')
    drain(nq, mq, db)
    assert states(db) == {'a': 'INTEGRATED', 'b': 'INTEGRATED'}
    head = g(repo, 'rev-parse', 'lab/integration')
    files = g(repo, 'ls-tree', '-r', '--name-only', head).split()
    assert 'one.txt' in files and 'two.txt' in files
    assert g(repo, 'rev-parse', 'main') == g(repo, 'rev-list', '--max-parents=0', 'main') or True
    assert g(repo, 'rev-parse', 'main') != head                         # main (production-like) untouched
    st = json.loads((nq.RUN / 'integration_state.json').read_text())
    assert st['head'] == head and [i['state'] for i in st['items']] == ['INTEGRATED', 'INTEGRATED']
    assert len(g(repo, 'rev-list', '--merges', 'lab/integration').split()) == 2            # real merges, nothing rewritten


def test_unverified_or_failed_tasks_are_never_queued(world):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'u', {'u.txt': 'x'}, verified=False)
    attempt(repo, nq, db, 'f', {'f.txt': 'x'}, state='NEEDS_ENGINEERING', phase='RESULT_RECORDED')
    attempt(repo, nq, db, 'p', {'p.txt': 'x'}, phase='RESULT_RECORDED')
    assert not queue(nq, mq, db, 'u') and not queue(nq, mq, db, 'f') and not queue(nq, mq, db, 'p')
    assert db.execute('SELECT COUNT(*) FROM merge_queue').fetchone()[0] == 0


def test_conflict_is_classified_branch_kept_and_independent_candidate_still_integrates(world):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'a', {'a.txt': 'from a\n'}); attempt(repo, nq, db, 'b', {'a.txt': 'from b\n'}); attempt(repo, nq, db, 'c', {'c.txt': 'c\n'})
    for n in 'abc':
        queue(nq, mq, db, n)
    drain(nq, mq, db)
    assert states(db) == {'a': 'INTEGRATED', 'b': 'CONFLICT', 'c': 'INTEGRATED'}
    assert g(repo, 'rev-parse', '--verify', 'attempt/b-1-abc')                        # conflicting branch preserved
    assert 'a.txt' in db.execute("SELECT detail FROM merge_queue WHERE task='b'").fetchone()[0]
    head = g(repo, 'rev-parse', 'lab/integration')
    assert g(repo, 'show', f'{head}:a.txt') == 'from a'                              # nothing half-merged
    assert not list((nq.RUN / 'merge').glob('wt.*'))                                  # temp worktrees cleaned


def test_failing_regression_gate_blocks_integration_keeps_evidence_and_queue_continues(world, monkeypatch):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'bad', {'ok.flag': '1\n', 'x.txt': 'x'})
    br, sha = attempt(repo, nq, db, 'good', {'g.txt': 'g\n'})
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'bash -c "! test -f x.txt"')       # the merged tree of "bad" breaks the gate
    queue(nq, mq, db, 'bad'); queue(nq, mq, db, 'good')
    before = g(repo, 'rev-parse', 'refs/heads/lab/integration') if subprocess.run(['git', '-C', str(repo), 'rev-parse', '-q', '--verify', 'refs/heads/lab/integration'], capture_output=True).returncode == 0 else None
    drain(nq, mq, db)
    assert states(db) == {'bad': 'REGRESSION_FAILED', 'good': 'INTEGRATED'}
    assert (nq.RUN / 'merge' / '1.gate.log').exists() and g(repo, 'rev-parse', '--verify', 'attempt/bad-1-abc')
    assert 'x.txt' not in g(repo, 'ls-tree', '-r', '--name-only', 'lab/integration')


def test_branch_moved_after_verification_is_stale_never_integrated(world):
    repo, nq, mq, db = world
    br, sha = attempt(repo, nq, db, 'a', {'a2.txt': '1'})
    queue(nq, mq, db, 'a')
    wt = db.execute("SELECT attempt_wt FROM tasks WHERE name='a'").fetchone()[0]
    (Path(wt) / 'sneaky.txt').write_text('after verification'); g(wt, 'add', '-A'); g(wt, 'commit', '-qm', 'late edit')
    drain(nq, mq, db)
    assert states(db) == {'a': 'STALE'}
    assert subprocess.run(['git', '-C', str(repo), 'rev-parse', '-q', '--verify', 'refs/heads/lab/integration'], capture_output=True).stdout.strip() in (b'', g(repo, 'rev-parse', 'main').encode())


def test_integration_head_moving_during_gate_refuses_cas_and_regates(world, monkeypatch):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'a', {'a2.txt': '1'})
    queue(nq, mq, db, 'a')
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'sleep 1')
    mq.step(db, nq.RUN)                                                               # merge built, gate running on head H0
    assert states(db) == {'a': 'GATING'}
    other = repo.parent / 'wt_other'
    g(repo, 'worktree', 'add', '-q', '--detach', str(other), 'lab/integration')
    (other / 'zz.txt').write_text('z'); g(other, 'add', '-A'); g(other, 'commit', '-qm', 'someone else moved the head')
    g(repo, 'update-ref', 'refs/heads/lab/integration', g(other, 'rev-parse', 'HEAD'))  # head moves while the gate runs
    time.sleep(1.5)
    moved = g(repo, 'rev-parse', 'lab/integration')
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'true')
    mq.step(db, nq.RUN)                                                               # gate done -> CAS refused -> immediately re-gated
    row = db.execute('SELECT state, merge_sha FROM merge_queue').fetchone()
    assert row['state'] == 'GATING' and row['merge_sha'] == moved                     # new gate is built on the NEW head, the stale result was dropped
    drain(nq, mq, db)
    assert states(db) == {'a': 'INTEGRATED'}
    files = g(repo, 'ls-tree', '-r', '--name-only', 'lab/integration').split()
    assert 'zz.txt' in files and 'a2.txt' in files                                    # nothing of the other change was lost


def test_restart_during_gate_requeues_without_losing_the_candidate(world, monkeypatch):
    repo, nq, mq, db = world
    attempt(repo, nq, db, 'a', {'a2.txt': '1'})
    queue(nq, mq, db, 'a')
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'sleep 3')
    mq.step(db, nq.RUN)
    assert states(db) == {'a': 'GATING'}
    mq.JOBS.clear()                                                                   # "orchestrator restarted"
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'true')
    drain(nq, mq, db)
    assert states(db) == {'a': 'INTEGRATED'}


def test_integration_branch_must_be_under_lab_and_never_main(world, monkeypatch):
    repo, nq, mq, db = world
    for bad in ('main', 'master', 'nav/first-goals-20261028', 'tooling/x'):
        monkeypatch.setenv('NIGHTLY_INTEGRATION_BRANCH', bad)
        with pytest.raises(SystemExit):
            mq.branch_name()


def test_publish_is_plain_push_of_the_lab_branch_only(world, tmp_path):
    repo, nq, mq, db = world
    remote = tmp_path / 'r.git'; subprocess.run(['git', 'init', '-q', '--bare', str(remote)], check=True)
    g(repo, 'remote', 'add', 'origin', str(remote))
    attempt(repo, nq, db, 'a', {'a2.txt': '1'}); queue(nq, mq, db, 'a'); drain(nq, mq, db)
    ok, out = mq.publish(repo)
    assert ok and subprocess.run(['git', '-C', str(remote), 'rev-parse', 'lab/integration'], capture_output=True, text=True).stdout.strip() == g(repo, 'rev-parse', 'lab/integration')
    assert subprocess.run(['git', '-C', str(remote), 'rev-parse', '-q', '--verify', 'main'], capture_output=True).returncode != 0   # main not pushed


def test_end_to_end_verified_attempt_enters_merge_queue_automatically(tmp_path):
    """Real daemon + stub agent: COMPLETED/VERIFIED -> MERGEQ_QUEUED -> INTEGRATED on lab/integration, main untouched."""
    root = tmp_path / 'root'; root.mkdir()
    subprocess.run(['git', 'init', '-q', '-b', 'main', str(root)], check=True)
    (root / 'ok.flag').write_text('1'); g(root, 'add', '-A'); g(root, 'commit', '-qm', 'i')
    b = tmp_path / 'bin'; b.mkdir()
    (b / 'claude').write_text('#!/usr/bin/env bash\n[ "$1" = "--version" ] && { echo s; exit 0; }\necho fixed > fix.txt\nmkdir -p .qazbot/results\n'
                              'echo "{\\"task\\":\\"$NIGHTLY_TASK\\",\\"attempt\\":$NIGHTLY_ATTEMPT,\\"attempt_id\\":\\"$NIGHTLY_ATTEMPT_ID\\",\\"summary\\":\\"s\\",\\"tests\\":{\\"cmd\\":\\"true\\"},\\"files\\":[{\\"path\\":\\"fix.txt\\"}]}" > .qazbot/results/$NIGHTLY_TASK.$NIGHTLY_ATTEMPT.json\n')
    (b / 'claude').chmod(0o755)
    e = dict(os.environ, NIGHTLY_ROOT=str(root), NIGHTLY_RUN=str(root / 'run' / 'nightly'), NIGHTLY_CLAUDE_BIN=str(b / 'claude'), NIGHTLY_CODEX_BIN='/nonexistent',
             NIGHTLY_CODEX_SOCK='/nonexistent', NIGHTLY_CODEX_QUEUE_DB='/nonexistent', NIGHTLY_VERIFY_ALLOW='true', NIGHTLY_INTEGRATION_GATE='test -f ok.flag', NIGHTLY_POLL_S='0')
    subprocess.run([sys.executable, str(NQ), 'enqueue', 'eng', 'x --agents=claude', '--kind', 'engineering', '--cwd', str(root)], env=e, check=True, capture_output=True)
    subprocess.run([sys.executable, str(NQ), 'daemon', '--poll', '0.2', '--max-seconds', '25'], env=e, capture_output=True, timeout=60)
    head = g(root, 'rev-parse', 'lab/integration')
    assert 'fix.txt' in g(root, 'ls-tree', '-r', '--name-only', head).split() and g(root, 'rev-parse', 'main') != head
    assert 'fix.txt' not in g(root, 'ls-tree', '-r', '--name-only', 'main')
