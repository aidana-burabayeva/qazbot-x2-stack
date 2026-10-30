"""P1 / P1.1: Claude <-> Codex handoff and the Codex completion contract. Fake CLIs and fake observers; real nq.py daemon.
Contract under test: DISPATCHED -> ACKNOWLEDGED -> RUNNING -> RESULT_RECORDED -> VERIFIED -> COMPLETED.
Delivery of a message (exit code of `codex queue`) is never completion; the worktree lease is kept until a verified result
or a proven end of the attempt; nothing relies on an agent's final message."""
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import pytest

NQ = Path(__file__).resolve().parents[2] / 'tools' / 'nightly' / 'nq.py'
LAB = NQ.with_name('lab.py')


def stub(bindir, name, body):
    p = bindir / name
    p.write_text('#!/usr/bin/env bash\n[ "$1" = "--version" ] && { echo "stub-' + name + '"; exit 0; }\n' + body)
    p.chmod(0o755)


@pytest.fixture
def env(tmp_path):
    root = tmp_path / 'root'; root.mkdir()
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.email=a@a', '-c', 'user.name=a', 'commit', '-q', '--allow-empty', '-m', 'i'], check=True)
    b = tmp_path / 'bin'; b.mkdir()
    log = tmp_path / 'calls.log'
    obs = tmp_path / 'obs.json'
    (b / 'observer').write_text('#!/usr/bin/env bash\n[ -f "$OBS_FILE" ] && cat "$OBS_FILE" || echo \'{"queued": true, "turn_status": null, "source_ok": true}\'\n')
    (b / 'observer').chmod(0o755)
    (b / 'usage').write_text('#!/usr/bin/env bash\n[ -f "$USAGE_FILE" ] && cat "$USAGE_FILE" || exit 3\n')
    (b / 'usage').chmod(0o755)
    e = dict(os.environ, NIGHTLY_ROOT=str(root), NIGHTLY_RUN=str(root / 'run' / 'nightly'), NIGHTLY_CLAUDE_BIN=str(b / 'claude'),
             NIGHTLY_CODEX_BIN=str(b / 'codex'), NIGHTLY_CODEX_OBSERVE_CMD=str(b / 'observer'), NIGHTLY_CODEX_USAGE_CMD=str(b / 'usage'),
             NIGHTLY_CODEX_SOCK='/nonexistent', NIGHTLY_CODEX_QUEUE_DB='/nonexistent', NIGHTLY_POLL_S='0', NIGHTLY_USAGE_TTL_S='0',
             NIGHTLY_VERIFY_ALLOW='true;false', STUBLOG=str(log), OBS_FILE=str(obs), USAGE_FILE=str(tmp_path / 'usage.json'), QAZBOT_ROLE='LAB')
    return e, root, b, log


def nq(e, *a):
    return subprocess.run([sys.executable, str(NQ), *a], capture_output=True, text=True, env=e, timeout=60)


def daemon(e, secs=3):
    return subprocess.run([sys.executable, str(NQ), 'daemon', '--poll', '0.2', '--max-seconds', str(secs)], capture_output=True, text=True, env=e, timeout=90)


def q(root, sql, *a):
    c = sqlite3.connect(root / 'run' / 'nightly' / 'queue.db'); c.row_factory = sqlite3.Row
    return c.execute(sql, a).fetchall()


def task(root, name='eng'):
    return q(root, 'SELECT * FROM tasks WHERE name=?', name)[0]


def events(root, kind):
    return [r['detail'] for r in q(root, 'SELECT detail FROM events WHERE event=?', kind)]


CODEX_Q = 'echo "codex $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; echo "Queued message item-$RANDOM for thread T"; exit 0\n'
CODEX_OK = CODEX_Q
CLAUDE_LIMIT = 'echo "claude $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; echo wip > wip.txt; echo "Claude usage limit reached, try again at 5pm"; exit 1\n'
WRITE_RESULT = (
    'echo fixed > fix.txt; python3 - <<PY\n'
    'import hashlib, json, os, subprocess\n'
    'sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()\n'
    'h = hashlib.sha256(open("fix.txt", "rb").read()).hexdigest()\n'
    'json.dump(dict(task=os.environ["NIGHTLY_TASK"], attempt=int(os.environ["NIGHTLY_ATTEMPT"]), attempt_id=os.environ["NIGHTLY_ATTEMPT_ID"], git_sha=sha, summary="ok",\n'
    '  tests=dict(cmd="true", passed=3, failed=0), files=[dict(path="fix.txt", sha256=h)], root_causes=[]), open(os.environ["NIGHTLY_RESULT_DIR"] + "/engineering_result.json", "w"))\n'
    'PY\n')
CLAUDE_OK = 'echo "claude $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; ' + WRITE_RESULT + 'exit 0\n'
CLAUDE_NORESULT = 'echo "claude $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; exit 0\n'


def sha(root):
    return subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()


def write_result(root, name='eng', failed=0, passed=3, **over):
    t = task(root, name)
    (root / 'fix.txt').write_text('fixed\n')
    import hashlib
    fh = hashlib.sha256((root / 'fix.txt').read_bytes()).hexdigest()
    d = dict(task=name, attempt=t['attempts'], attempt_id=t['attempt_id'], git_sha=sha(root), summary='fixed',
             tests={'cmd': 'true' if not failed else 'false', 'passed': passed, 'failed': failed}, files=[{'path': 'fix.txt', 'sha256': fh}], root_causes=[])
    d.update(over)
    rd = root / 'run' / 'nightly' / 'work' / f'{name}.{t["attempts"]}'
    rd.mkdir(parents=True, exist_ok=True)
    (rd / 'engineering_result.json').write_text(json.dumps(d))


def delegate(e, root, b, name='eng', agents='codex', extra=()):
    stub(b, 'codex', CODEX_Q); stub(b, 'claude', CLAUDE_OK)
    nq(e, 'enqueue', name, f'x --agents={agents}', '--kind', 'engineering', '--cwd', str(root), *extra)
    daemon(e, 2)


# ---------------------------------------------------------------- P1: handoff on agent loss (claude side is a real process)

def test_limit_hits_claude_task_is_delegated_to_codex_with_context_not_completed(env):
    e, root, b, log = env
    stub(b, 'claude', CLAUDE_LIMIT); stub(b, 'codex', CODEX_Q)
    nq(e, 'root-cause', 'rc1', 'goal tolerances differ', '--status', 'VERIFIED', '--evidence', 'ccw_gate/')
    nq(e, 'enqueue', 'eng', 'investigate --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    t = task(root)
    assert t['state'] == 'DELEGATED' and t['phase'] == 'DISPATCHED' and t['owner'] == 'codex'      # delivered, NOT completed
    calls = log.read_text().splitlines()
    assert calls[0].startswith('claude ') and calls[1].startswith('codex queue')
    assert 'goal tolerances differ' in calls[1] and 'Last checkpoint (orchestrator-finish' in calls[1] and 'wip.txt' in calls[1]
    assert f"QAZBOT-TASK:eng:{t['attempts']}:{t['attempt_id']}" in calls[1]
    st = json.loads(q(root, "SELECT v FROM kv WHERE k='agents'")[0]['v'])
    assert st['claude']['status'] == 'usage_limited' and st['claude']['retry_after'] is None and st['claude']['reset_known'] is False   # no invented reset
    assert q(root, 'SELECT * FROM results') == []


def test_claude_killed_silently_is_lost_then_delegated(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "claude $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; kill -9 $$\n'); stub(b, 'codex', CODEX_Q)
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    assert task(root)['state'] == 'DELEGATED' and any('(lost)' in d for d in events(root, 'HANDOFF'))


def test_codex_limit_text_hands_off_to_claude_which_really_runs(env):
    e, root, b, log = env
    stub(b, 'codex', 'echo "codex $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; echo "rate limit exceeded"; exit 1\n'); stub(b, 'claude', CLAUDE_OK)
    nq(e, 'enqueue', 'eng', 'x --agents=codex,claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    t = task(root)
    assert t['state'] == 'COMPLETED' and t['owner'] == 'claude'


def test_genuine_agent_error_is_not_classified_as_usage_limit(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "claude $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; echo "Traceback: real bug in the task"; exit 2\n'); stub(b, 'codex', CODEX_Q)
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    assert task(root)['state'] == 'NEEDS_ENGINEERING'
    assert not any(ln.startswith('codex ') for ln in log.read_text().splitlines())
    assert 'usage_limited' not in q(root, "SELECT v FROM kv WHERE k='agents'")[0]['v']


def test_codex_dispatch_error_that_is_not_a_limit_goes_to_engineering(env):
    e, root, b, log = env
    stub(b, 'codex', 'echo "codex $*" | tr "\\n" " " >> "$STUBLOG"; echo >> "$STUBLOG"; echo "Error: no such thread"; exit 2\n'); stub(b, 'claude', CLAUDE_OK)
    nq(e, 'enqueue', 'eng', 'x --agents=codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    assert task(root)['state'] in ('NEEDS_ENGINEERING', 'PAUSED_AGENT') and events(root, 'DISPATCH_FAILED')


# ---------------------------------------------------------------- P1.1: Codex completion contract

def test_delivery_exit_code_is_not_completion_and_lease_is_held(env):
    e, root, b, log = env
    delegate(e, root, b)
    t = task(root)
    assert (t['state'], t['phase']) == ('DELEGATED', 'DISPATCHED') and q(root, 'SELECT * FROM results') == []
    nq(e, 'enqueue', 'second', 'y --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 2)
    assert task(root, 'second')['state'] == 'PENDING'                       # claude cannot take the busy worktree
    assert not any(ln.startswith('claude ') for ln in log.read_text().splitlines())


def test_accepted_and_running_keeps_waiting_no_second_dispatch(env):
    e, root, b, log = env
    delegate(e, root, b)
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'inProgress', 'turn_id': 't1', 'source_ok': True}))
    for _ in range(3):
        daemon(e, 2)
    t = task(root)
    assert (t['state'], t['phase']) == ('DELEGATED', 'RUNNING')
    assert sum(ln.startswith('codex queue') for ln in log.read_text().splitlines()) == 1     # never re-sent while possibly active


def test_lost_connection_after_accept_changes_nothing(env):
    e, root, b, log = env
    delegate(e, root, b)
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': None, 'turn_status': None, 'source_ok': False}))
    for _ in range(3):
        daemon(e, 2)
    t = task(root)
    assert (t['state'], t['phase']) == ('DELEGATED', 'DISPATCHED') and q(root, 'SELECT * FROM results') == []
    assert sum(ln.startswith('codex queue') for ln in log.read_text().splitlines()) == 1


def test_codex_finishes_without_any_final_text_but_with_verified_result_completes(env):
    e, root, b, log = env
    delegate(e, root, b)
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'completed', 'source_ok': True}))   # no agent message at all
    write_result(root)
    daemon(e, 2)
    t = task(root)
    assert (t['state'], t['phase']) == ('COMPLETED', 'VERIFIED')
    r = q(root, 'SELECT * FROM results')[0]
    assert r['verdict'] == 'PASS' and r['seed'] == t['attempt_id'] and sha(root) in r['metrics'] and r['status'] == 'STORED'
    assert nq(e, 'verify-evidence').returncode == 0


def test_turn_ended_without_result_is_not_success_and_releases_lease_only_after_proof(env):
    e, root, b, log = env
    delegate(e, root, b)
    e2 = dict(e, NIGHTLY_RESULT_GRACE_S='0')
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'interrupted', 'source_ok': True}))
    daemon(e2, 3)
    t = task(root)
    assert t['state'] == 'NEEDS_ENGINEERING' and q(root, 'SELECT * FROM results') == [] and events(root, 'ATTEMPT_ENDED_NO_RESULT')
    nq(e, 'enqueue', 'second', 'y --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    assert task(root, 'second')['state'] == 'COMPLETED'                     # lease released only now


@pytest.mark.parametrize('mutate,why', [
    ({'attempt_id': 'deadbeef'}, 'STALE_ATTEMPT'),
    ({'git_sha': '0' * 40}, 'not a commit'),
    ({'tests': {'cmd': 'x'}}, 'not verifiable'),
    ({'task': 'other'}, 'another task'),
    ({'files': [{'path': 'nope.txt', 'sha256': 'ab'}]}, 'file path rejected'),
    ({'files': [{'path': '../../etc/hostname', 'sha256': 'ab'}]}, 'file path rejected'),
    ({'files': [{'path': '/etc/hostname', 'sha256': 'ab'}]}, 'file path rejected'),
    ({'files': [{'path': 'fix.txt', 'sha256': 'ab'}]}, 'hash mismatch'),
    ({'files': []}, 'non-empty'),
    ({'files': None}, 'non-empty'),
    ({'files': ['x']}, 'not an object'),
    ({'root_causes': 'x'}, 'root_causes'),
    ({'tests': {'cmd': 'rm -rf /', 'passed': 1, 'failed': 0}}, 'not allow-listed'),
    ({'git_sha': 5}, 'not a commit'),
])
def test_forged_or_stale_results_are_rejected_and_task_stays_delegated(env, mutate, why):
    e, root, b, log = env
    delegate(e, root, b)
    write_result(root, **mutate)
    daemon(e, 2)
    assert task(root)['state'] == 'DELEGATED' and q(root, 'SELECT * FROM results') == []
    assert any(why in d for d in events(root, 'RESULT_REJECTED'))


def test_symlinked_result_file_is_rejected_and_daemon_survives_garbage_json(env):
    e, root, b, log = env
    delegate(e, root, b)
    rd = root / 'run' / 'nightly' / 'work' / f"eng.{task(root)['attempts']}"
    rd.mkdir(parents=True, exist_ok=True)
    (rd / 'engineering_result.json').write_text('[]')                       # valid JSON, wrong shape: used to crash the orchestrator
    assert daemon(e, 2).returncode == 0 and task(root)['state'] == 'DELEGATED'
    (rd / 'engineering_result.json').write_text('{not json')
    assert daemon(e, 2).returncode == 0 and task(root)['state'] == 'DELEGATED'
    os.remove(rd / 'engineering_result.json')
    os.symlink('/etc/hostname', rd / 'engineering_result.json')
    assert daemon(e, 2).returncode == 0 and task(root)['state'] == 'DELEGATED' and q(root, 'SELECT * FROM results') == []


def test_symlink_inside_result_dir_is_never_copied_into_evidence(env, tmp_path):
    sys.path.insert(0, str(NQ.parent))
    import store
    src = tmp_path / 'src'; src.mkdir()
    (src / 'a.txt').write_text('a'); os.symlink('/etc/hostname', src / 'leak')
    ev, _ = store.publish(tmp_path / 'run', 't', 1, src, {})
    assert (Path(ev) / 'a.txt').exists() and not (Path(ev) / 'leak').exists()


def test_recorded_result_with_failing_tests_is_stored_as_fail_not_completed(env):
    e, root, b, log = env
    delegate(e, root, b)
    write_result(root, failed=2)
    daemon(e, 2)
    assert task(root)['state'] == 'NEEDS_ENGINEERING' and q(root, 'SELECT verdict FROM results')[0]['verdict'] == 'FAIL'


def test_orchestrator_restart_mid_codex_run_resumes_observation(env):
    e, root, b, log = env
    delegate(e, root, b)
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'inProgress', 'source_ok': True}))
    daemon(e, 2)                                                            # orchestrator "dies" (exits) while Codex runs
    assert task(root)['state'] == 'DELEGATED'
    write_result(root)                                                      # Codex finishes while nobody is watching
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'completed', 'source_ok': True}))
    daemon(e, 2)                                                            # new orchestrator picks it up from SQLite
    assert task(root)['state'] == 'COMPLETED'
    assert sum(ln.startswith('codex queue') for ln in log.read_text().splitlines()) == 1


def test_operator_stop_with_codex_running_keeps_lease_and_reports_unverified(env, tmp_path):
    e, root, b, log = env
    (root / '.gitignore').write_text('run/\n')
    subprocess.run(['git', '-C', str(root), 'add', '-A'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.email=a@a', '-c', 'user.name=a', 'commit', '-q', '-m', 'ig'], check=True)
    delegate(e, root, b)
    Path(e['OBS_FILE']).write_text(json.dumps({'queued': False, 'turn_status': 'inProgress', 'source_ok': True}))
    daemon(e, 2)
    r = subprocess.run([sys.executable, str(LAB), 'stop', '--no-systemd', '--no-docker', '--no-tests', '--no-push'], capture_output=True, text=True, env=e)
    assert 'DELEGATED_OPEN' in r.stdout and 'eng' in r.stdout
    t = task(root)
    assert t['state'] == 'DELEGATED'                                        # not parked, not released: process not proven dead
    assert q(root, "SELECT v FROM kv WHERE k='codex_stop'")[0]['v'] == '"STOP_UNVERIFIED"'
    nq(e, 'enqueue', 'second', 'y --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 2)
    assert task(root, 'second')['state'] == 'PENDING'                       # frozen + leased


def test_foreign_agent_tui_in_worktree_blocks_new_writer(env):
    e, root, b, log = env
    stub(b, 'claude', CLAUDE_OK)
    tui = subprocess.Popen(['bash', '-c', 'exec -a codex sleep 60'], cwd=str(root))          # operator's own Codex TUI
    try:
        time.sleep(0.3)
        nq(e, 'enqueue', 'eng', 'x --agents=claude', '--kind', 'engineering', '--cwd', str(root))
        daemon(e, 2)
        assert task(root)['state'] == 'PENDING' and events(root, 'LEASE_BLOCKED_FOREIGN_WRITER')
        assert not log.exists()
    finally:
        tui.send_signal(signal.SIGKILL); tui.wait()
    daemon(e, 3)
    assert task(root)['state'] == 'COMPLETED'


def test_supersede_never_automatic_and_manual_needs_explicit_flag_and_late_result_is_rejected(env):
    e, root, b, log = env
    delegate(e, root, b, agents='codex,claude')
    old = task(root)['attempt_id']
    daemon(dict(e, NIGHTLY_SUPERSEDE_AFTER_S='0'), 3)                       # the old auto-supersede switch no longer exists
    assert task(root)['state'] == 'DELEGATED' and not events(root, 'SUPERSEDED')
    assert nq(e, 'supersede', 'eng').returncode != 0 and task(root)['state'] == 'DELEGATED'
    assert nq(e, 'supersede', 'eng', '--force-lease-release').returncode == 0
    assert task(root)['state'] == 'PENDING' and events(root, 'SUPERSEDED')
    t = task(root)
    stub(b, 'claude', CLAUDE_OK)
    nq(e, 'agent', 'codex', 'usage_limited')
    daemon(e, 3)
    assert task(root)['state'] == 'COMPLETED' and task(root)['owner'] == 'claude' and task(root)['attempt_id'] != old
    # a late result of the OLD attempt cannot be accepted
    assert not q(root, "SELECT * FROM results WHERE seed=?", old)


def test_two_pending_tasks_same_worktree_only_one_dispatch_per_tick(env):
    e, root, b, log = env
    stub(b, 'codex', CODEX_Q); stub(b, 'claude', CLAUDE_OK)
    nq(e, 'enqueue', 'a', 'x --agents=codex', '--kind', 'engineering', '--cwd', str(root))
    nq(e, 'enqueue', 'b', 'y --agents=codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    assert sum(ln.startswith('codex queue') for ln in log.read_text().splitlines()) == 1
    assert {task(root, 'a')['state'], task(root, 'b')['state']} == {'DELEGATED', 'PENDING'}


def test_dispatch_timeout_keeps_intent_and_lease_and_never_resends(env):
    e, root, b, log = env
    stub(b, 'codex', 'echo "codex queue" >> "$STUBLOG"; sleep 5; echo "Queued message item-1 for thread T"\n'); stub(b, 'claude', CLAUDE_OK)
    e2 = dict(e, NIGHTLY_DISPATCH_TIMEOUT_S='1')
    nq(e, 'enqueue', 'eng', 'x --agents=codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e2, 4)
    t = task(root)
    assert t['state'] == 'DELEGATED' and t['phase'] in ('DISPATCHING', 'DISPATCHED') and events(root, 'DISPATCH_UNKNOWN') and t['attempt_id']
    daemon(e2, 3)
    assert sum(ln.startswith('codex queue') for ln in log.read_text().splitlines()) == 1


def test_engineering_agent_exit_zero_without_result_is_not_completed(env):
    e, root, b, log = env
    stub(b, 'claude', CLAUDE_NORESULT)
    nq(e, 'enqueue', 'eng', 'x --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    t = task(root)
    assert t['state'] == 'NEEDS_ENGINEERING' and 'without a verified result' in t['reason'] and q(root, 'SELECT * FROM results') == []


def test_commit_extra_statements_are_atomic_with_completion(env, tmp_path):
    sys.path.insert(0, str(NQ.parent))
    import store
    db = sqlite3.connect(tmp_path / 'x.db'); db.row_factory = sqlite3.Row; db.isolation_level = None
    db.executescript("CREATE TABLE tasks(id INTEGER PRIMARY KEY, state TEXT, exit_code, pid, finished, signature, reason, phase);" + store.SCHEMA)
    db.execute("INSERT INTO tasks(id,state) VALUES(1,'DELEGATED')")
    item = {'scenario_id': 's', 'seed': 1, 'verdict': 'PASS', 'metrics': {}, 'safety_violations': 0}
    with pytest.raises(sqlite3.OperationalError):
        store.commit(db, 1, [item], '/e', 'h', {'task': 't', 'attempt': 1}, 'COMPLETED', 0, None, None, extra=[('UPDATE nonexistent SET x=1', ())])
    assert db.execute('SELECT state FROM tasks').fetchone()[0] == 'DELEGATED' and db.execute('SELECT COUNT(*) FROM results').fetchone()[0] == 0


def test_turn_match_is_anchored_to_the_first_user_message(env):
    sys.path.insert(0, str(NQ.parent))
    import agents
    tok = 'QAZBOT-TASK:eng:1:abc'
    quoting = {'id': 'later', 'status': 'completed', 'items': [{'type': 'userMessage', 'content': [{'type': 'text', 'text': 'please look at ' + 'other'}]}, {'type': 'agentMessage', 'text': tok}]}
    real = {'id': 'real', 'status': 'inProgress', 'items': [{'type': 'userMessage', 'content': [{'type': 'text', 'text': f'[{tok}] do it'}]}]}
    assert agents.match_turn([quoting, real], tok)['id'] == 'real'
    assert agents.match_turn([quoting], tok) is None


def test_app_server_client_refuses_any_non_allowlisted_method():
    sys.path.insert(0, str(NQ.parent))
    import agents
    a = agents.AppServer()
    for m in ('account/rateLimitResetCredit/consume', 'thread/delete', 'account/logout', 'anything/new'):
        with pytest.raises(PermissionError):
            a.call(m, None)


def test_handoff_cap_stops_bouncing(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "x" >> "$STUBLOG"; kill -9 $$\n'); stub(b, 'codex', 'echo "y" >> "$STUBLOG"; kill -9 $$\n')
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(dict(e, NIGHTLY_MAX_HANDOFFS='1'), 4)
    assert task(root)['state'] in ('NEEDS_ENGINEERING', 'PAUSED_AGENT') and len(log.read_text().splitlines()) <= 3


# ---------------------------------------------------------------- quota facts

def test_codex_usage_from_provider_limited_sets_official_reset_and_blocks(env):
    e, root, b, log = env
    stub(b, 'codex', CODEX_Q); stub(b, 'claude', CLAUDE_OK)
    reset = int(time.time()) + 7200
    Path(e['USAGE_FILE']).write_text(json.dumps({'ordinaryUsageAllowed': False, 'rateLimits': {'primary': {'usedPercent': 100, 'windowDurationMins': 300, 'resetsAt': reset}, 'secondary': {'usedPercent': 43, 'resetsAt': reset + 99999}, 'rateLimitReachedType': 'rate_limit_reached'}}))
    nq(e, 'enqueue', 'eng', 'x --agents=codex,claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e, 3)
    st = json.loads(q(root, "SELECT v FROM kv WHERE k='agents'")[0]['v'])['codex']
    assert st['status'] == 'usage_limited' and st['retry_after'] == reset and st['reset_known'] is True
    assert task(root)['owner'] == 'claude' and task(root)['state'] == 'COMPLETED'


def test_unknown_usage_is_not_available_and_not_limited(env):
    e, root, b, log = env
    out = nq(e, 'usage')
    u = json.loads(out.stdout)
    assert u['codex']['status'] == 'UNKNOWN' and u['claude']['status'] == 'UNKNOWN' and u['codex']['reset_at'] is None


def test_lab_stop_notification_goes_through_adapter_binary_never_a_hardcoded_codex(env):
    e, root, b, log = env
    stub(b, 'codex', 'echo "codex $*" | tr "\\n" " " >> "$STUBLOG"; echo; echo "Queued message item-1 for thread T"; exit 0\n')
    (root / '.gitignore').write_text('run/\n')
    subprocess.run(['git', '-C', str(root), 'add', '-A'], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.email=a@a', '-c', 'user.name=a', 'commit', '-q', '-m', 'ig'], check=True)
    r = subprocess.run([sys.executable, str(LAB), 'stop', '--no-systemd', '--no-docker', '--no-tests', '--no-push'], capture_output=True, text=True, env=e)
    assert 'OPERATOR STOP' in log.read_text() and 'rc=0' in r.stdout


def test_codex_limited_without_reset_time_gets_bounded_recheck_not_available(env):
    e, root, b, log = env
    Path(e['USAGE_FILE']).write_text(json.dumps({'ordinaryUsageAllowed': False, 'rateLimits': {'primary': {'usedPercent': 100, 'windowDurationMins': 300, 'resetsAt': None}, 'rateLimitReachedType': 'rate_limit_reached'}}))
    nq(e, 'usage')
    st = json.loads(q(root, "SELECT v FROM kv WHERE k='agents'")[0]['v'])['codex']
    assert st['status'] == 'usage_limited' and st['retry_after'] is None and st['recheck_after'] > time.time() and st['reset_known'] is False
