"""Zero-loss execution: crash at every point of result publication, false-PASS guards, evidence integrity, agent loss.
Everything runs the real nq.py daemon in a temp NIGHTLY_RUN; no systemd, docker, AI agent or robot."""
import hashlib
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

NQ = Path(__file__).resolve().parents[2] / 'tools' / 'nightly' / 'nq.py'


@pytest.fixture
def env(tmp_path):
    root = tmp_path / 'root'
    root.mkdir()
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.email=a@a', '-c', 'user.name=a', 'commit', '-q', '--allow-empty', '-m', 'i'], check=True)
    return dict(os.environ, NIGHTLY_ROOT=str(root), NIGHTLY_RUN=str(root / 'run' / 'nightly')), root


def nq(e, *a, **kw):
    return subprocess.run([sys.executable, str(NQ), *a], capture_output=True, text=True, env=e, timeout=60, **kw)


def daemon(e, fault=None):
    e2 = dict(e, NQ_FAULT=fault) if fault else e
    return subprocess.run([sys.executable, str(NQ), 'daemon', '--poll', '0.2'], capture_output=True, text=True, env=e2, timeout=60)


def db(root):
    c = sqlite3.connect(root / 'run' / 'nightly' / 'queue.db')
    c.row_factory = sqlite3.Row
    return c


def writer(items, rc=0):
    body = json.dumps(items).replace("'", "'\\''")
    return f"echo '{body}' > $NIGHTLY_RESULT_DIR/result.json; echo evidence > $NIGHTLY_RESULT_DIR/extra.txt; exit {rc}"


GOOD = [{'scenario_id': 's1', 'seed': 1, 'verdict': 'PASS', 'metrics': {'t': 4.1}, 'safety_violations': 0},
        {'scenario_id': 's2', 'seed': 2, 'verdict': 'FAIL', 'metrics': {'t': 9.9}, 'safety_violations': 1}]


def rows(root, task='t'):
    return db(root).execute('SELECT * FROM results WHERE task=? ORDER BY scenario_id, attempt', (task,)).fetchall()


def state(root, task='t'):
    return db(root).execute('SELECT state, attempts FROM tasks WHERE name=?', (task,)).fetchone()


def test_normal_publish_indexes_and_hashes(env):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    assert daemon(e).returncode == 0
    r = rows(root)
    assert [x['scenario_id'] for x in r] == ['s1', 's2'] and state(root)['state'] == 'COMPLETED'
    assert r[0]['git_sha'] and r[0]['config_sha'] and r[1]['verdict'] == 'FAIL'
    ev = Path(r[0]['evidence_path'])
    man = json.loads((ev / 'MANIFEST.json').read_text())
    assert set(man['files']) >= {'result.json', 'extra.txt', 'task.log'}
    assert all(hashlib.sha256((ev / f).read_bytes()).hexdigest() == h for f, h in man['files'].items())
    assert nq(e, 'verify-evidence').returncode == 0


@pytest.mark.parametrize('point', ['before_result_write', 'during_evidence'])
def test_crash_before_evidence_is_durable_reruns_without_duplicates(env, point):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    assert daemon(e, point).returncode == 137
    assert rows(root) == [] and state(root)['state'] == 'RUNNING'
    assert daemon(e).returncode == 0
    r = rows(root)
    assert [(x['scenario_id'], x['attempt']) for x in r] == [('s1', 2), ('s2', 2)]  # exactly one set, from the re-run
    assert state(root)['state'] == 'COMPLETED'
    ev = root / 'run' / 'nightly' / 'evidence'
    assert not [p for p in ev.iterdir() if p.name.startswith('.tmp-')]
    if point == 'during_evidence':
        assert list((ev / '_quarantine').iterdir())  # partial publish kept, not deleted


def test_crash_after_evidence_before_commit_is_adopted_not_rerun(env):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    assert daemon(e, 'after_evidence_before_commit').returncode == 137
    assert rows(root) == []
    assert daemon(e).returncode == 0
    r = rows(root)
    assert [(x['scenario_id'], x['attempt']) for x in r] == [('s1', 1), ('s2', 1)]
    assert tuple(state(root)) == ('COMPLETED', 1)  # task not executed a second time
    assert db(root).execute("SELECT COUNT(*) FROM events WHERE event='STORE_RECOVER'").fetchone()[0] == 1


def test_crash_after_commit_nothing_lost_nothing_duplicated(env):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    assert daemon(e, 'after_commit').returncode == 137
    before = [tuple(x) for x in rows(root)]
    assert len(before) == 2 and state(root)['state'] == 'COMPLETED'
    assert daemon(e).returncode == 0
    assert [tuple(x) for x in rows(root)] == before and state(root)['attempts'] == 1


def test_failed_attempt_is_kept_with_results(env):
    e, root = env
    nq(e, 'enqueue', 't', writer([{'scenario_id': 'f', 'seed': 3, 'verdict': 'FAIL', 'metrics': {}, 'safety_violations': 2}], rc=1), '--requires-result')
    daemon(e)
    r = rows(root)
    assert len(r) == 1 and r[0]['verdict'] == 'FAIL' and state(root)['state'] != 'COMPLETED'
    assert (Path(r[0]['evidence_path']) / 'task.log').exists()


@pytest.mark.parametrize('payload,verdict', [
    ([{'scenario_id': 'a', 'seed': 1, 'verdict': 'PASS', 'metrics': {}, 'safety_violations': 1}], 'FAIL'),
    ([{'scenario_id': 'a', 'verdict': 'PASS', 'metrics': {}}], 'UNVERIFIED'),
])
def test_false_pass_is_downgraded(env, payload, verdict):
    e, root = env
    nq(e, 'enqueue', 't', writer(payload), '--requires-result')
    daemon(e)
    assert rows(root)[0]['verdict'] == verdict


@pytest.mark.parametrize('cmd', ['true', 'echo {not json > $NIGHTLY_RESULT_DIR/result.json',
                                 "echo '[{\"scenario_id\":\"a\",\"verdict\":\"MAYBE\",\"metrics\":{}}]' > $NIGHTLY_RESULT_DIR/result.json"])
def test_missing_or_invalid_result_is_never_completed(env, cmd):
    e, root = env
    nq(e, 'enqueue', 't', cmd, '--requires-result')
    daemon(e)
    assert state(root)['state'] == 'NEEDS_ENGINEERING' and rows(root) == []


def test_legacy_task_without_result_contract_still_completes(env):
    e, root = env
    nq(e, 'enqueue', 't', 'true')
    daemon(e)
    assert state(root)['state'] == 'COMPLETED'


def test_tampered_evidence_is_detected_and_result_distrusted(env):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    daemon(e)
    ev = Path(rows(root)[0]['evidence_path'])
    (ev / 'extra.txt').write_text('tampered')
    assert nq(e, 'verify-evidence').returncode == 1
    assert {x['status'] for x in rows(root)} == {'EVIDENCE_MISSING'} and state(root)['state'] == 'NEEDS_ENGINEERING'
    assert (ev / 'extra.txt').exists()  # still kept for inspection


def test_history_survives_new_runs(env):
    e, root = env
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    daemon(e)
    h1 = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (root / 'run' / 'nightly' / 'evidence').rglob('*') if p.is_file()}
    nq(e, 'enqueue', 'u', writer(GOOD), '--requires-result')
    daemon(e)
    assert all(hashlib.sha256(p.read_bytes()).hexdigest() == h for p, h in h1.items())
    assert len(rows(root, 'u')) == 2 and len(rows(root, 't')) == 2


def test_engineering_task_agent_dies_without_message_state_is_recovered(env):
    e, root = env
    (root / 'wip.py').write_text('unfinished\n')
    sys.path.insert(0, str(NQ.parent))
    os.environ.update(NIGHTLY_ROOT=e['NIGHTLY_ROOT'], NIGHTLY_RUN=e['NIGHTLY_RUN'])
    import importlib, nq as nqm
    importlib.reload(nqm)
    d = nqm.connect()
    nqm.enqueue(d, 'eng', 'x', kind='engineering', cwd=str(root))
    d.execute("UPDATE tasks SET state='RUNNING', attempts=1, pid=NULL WHERE name='eng'")
    nqm.recover(d)
    c = d.execute("SELECT * FROM eng_checkpoints WHERE task='eng'").fetchone()
    assert c and c['source'] == 'orchestrator-recover' and c['verified'] == 0
    assert 'wip.py' in c['unfinished'] and c['git_sha']
    assert d.execute("SELECT state FROM tasks WHERE name='eng'").fetchone()['state'] == 'PENDING'


def test_verified_root_cause_is_not_downgraded_and_reaches_next_agent(env):
    e, root = env
    assert nq(e, 'root-cause', 'rc1', 'goal tolerances differ', '--status', 'VERIFIED', '--evidence', 'ccw_gate/').stdout.strip() == 'stored'
    assert 'kept' in nq(e, 'root-cause', 'rc1', 'maybe windup', '--status', 'HYPOTHESIS').stdout
    sys.path.insert(0, str(NQ.parent))
    os.environ.update(NIGHTLY_ROOT=e['NIGHTLY_ROOT'], NIGHTLY_RUN=e['NIGHTLY_RUN'])
    import importlib, nq as nqm
    importlib.reload(nqm)
    nqm.enqueue(nqm.connect(), 'eng2', 'x', kind='engineering')
    row = nqm.connect().execute("SELECT * FROM tasks WHERE name='eng2'").fetchone()
    cmd = nqm.agent_command('claude', row)
    assert 'goal tolerances differ' in cmd and 'do not re-investigate' in cmd


def test_both_agents_limited_engineering_paused_mechanical_results_still_stored(env):
    e, root = env
    nq(e, 'agent', 'claude', 'usage_limited'); nq(e, 'agent', 'codex', 'usage_limited')
    nq(e, 'enqueue', 'eng', 'investigate', '--kind', 'engineering')
    nq(e, 'enqueue', 't', writer(GOOD), '--requires-result')
    daemon(e)
    assert state(root, 'eng')['state'] == 'PAUSED_AGENT'
    assert state(root)['state'] == 'COMPLETED' and len(rows(root)) == 2
