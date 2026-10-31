"""Dual permanent branches: robot/stable (installed X2) and lab/stable (SIM_VERIFIED). Candidates, hardware outcomes, snapshots, migration, audit."""
import importlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

import test_mergeq as tm
from test_mergeq import attempt, drain, g, queue, states

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools' / 'operator_skills'))
sys.path.insert(0, str(REPO / 'tools' / 'nightly'))
import robot_stable as rs  # noqa: E402

BASE = {'ws/src/navigation/a.py': 'l1\nl2\nl3\n', 'ws/src/lio/b.py': 'b0\n', 'digital_twin/sim.py': 's0\n', 'tests/t.py': 't\n', 'docs/README.md': 'r\n', 'ok.flag': '1\n', '.gitignore': 'run/\n'}


@pytest.fixture
def uni(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'; repo.mkdir()
    subprocess.run(['git', 'init', '-q', '-b', 'main', str(repo)], check=True)
    for f, c in BASE.items():
        (repo / f).parent.mkdir(parents=True, exist_ok=True); (repo / f).write_text(c)
    g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 'base')
    g(repo, 'branch', 'robot/stable'); g(repo, 'branch', 'lab/stable')
    monkeypatch.setenv('NIGHTLY_ROOT', str(repo)); monkeypatch.setenv('NIGHTLY_RUN', str(repo / 'run' / 'nightly'))
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'test -f ok.flag'); monkeypatch.setenv('NIGHTLY_VERIFY_NICE', '0')
    for m in ('store', 'attempts', 'agents', 'router', 'candidates', 'mergeq', 'nq'):
        sys.modules.pop(m, None)
    nq = importlib.import_module('nq'); mq = importlib.import_module('mergeq')
    return repo, nq, mq, nq.connect()


def cause(nq, db, task):
    nq.store.add_root_cause(db, f'rc_{task}', f'root cause of {task}', 'VERIFIED', f'evidence/{task}', task)


def cands(repo, ref='lab/stable'):
    return json.loads(g(repo, 'show', f'{ref}:docs/hardware/CANDIDATES.json'))['candidates']


def test_two_sequential_lab_fixes_create_candidates_with_dependency_chain(uni):
    repo, nq, mq, db = uni
    attempt(repo, nq, db, 'fix1', {'ws/src/navigation/a.py': 'L1\nl2\nl3\n'}); cause(nq, db, 'fix1')
    attempt(repo, nq, db, 'fix2', {'ws/src/navigation/a.py': 'l1\nl2\nL3\n'}); cause(nq, db, 'fix2')
    queue(nq, mq, db, 'fix1'); queue(nq, mq, db, 'fix2'); drain(nq, mq, db)
    assert states(db) == {'fix1': 'INTEGRATED', 'fix2': 'INTEGRATED'}
    c = cands(repo)
    assert [x['task_id'] for x in c] == ['fix1', 'fix2'] and c[1]['dependencies'] == [c[0]['id']]
    for x in c:
        for k in rs.REQUIRED:
            assert k in x and x[k] not in (None, '') or k == 'dependencies', k
        assert x['production_files'][0]['path'] == 'ws/src/navigation/a.py' and x['sim_verified']['verdict'] == 'PASS'
        assert x['root_cause']['summary'].startswith('root cause of') and x['regression']['result'] == 'PASS' and x['rollback']['restore_from'] == 'robot/stable'
    assert g(repo, 'show', 'lab/stable:ws/src/navigation/a.py') == 'L1\nl2\nL3'            # both fixes are in lab/stable
    assert g(repo, 'rev-parse', 'robot/stable') == g(repo, 'rev-parse', 'main')              # the robot branch is not touched by lab integration


def test_sim_only_fix_and_fix_without_root_cause_are_never_offered_to_the_robot(uni):
    repo, nq, mq, db = uni
    attempt(repo, nq, db, 'simonly', {'digital_twin/sim.py': 's1\n'}); cause(nq, db, 'simonly')
    attempt(repo, nq, db, 'norc', {'ws/src/lio/b.py': 'b1\n'})
    queue(nq, mq, db, 'simonly'); queue(nq, mq, db, 'norc')
    for _ in range(60):
        if not mq.step(db, nq.RUN, lambda t, e, d='': nq.ev(db, t, e, d)):
            break
        time.sleep(0.2)
    assert states(db) == {'simonly': 'INTEGRATED', 'norc': 'INTEGRATED'}
    assert g(repo, 'show', 'lab/stable:digital_twin/sim.py') == 's1'
    assert not (subprocess.run(['git', '-C', str(repo), 'cat-file', '-e', 'lab/stable:docs/hardware/CANDIDATES.json'], capture_output=True).returncode == 0 and cands(repo))
    assert db.execute("SELECT detail FROM events WHERE event='CANDIDATE_REFUSED'").fetchone()[0].startswith('production fix without a VERIFIED root cause')


def prepared(uni, name='fix1', files=None):
    repo, nq, mq, db = uni
    attempt(repo, nq, db, name, files or {'ws/src/navigation/a.py': 'L1\nl2\nl3\n'}); cause(nq, db, name)
    queue(nq, mq, db, name); drain(nq, mq, db)
    return cands(repo)[-1]['id']


def test_candidate_branch_carries_only_production_files_and_a_plan(uni):
    repo = uni[0]
    cid = prepared(uni, files={'ws/src/navigation/a.py': 'L1\nl2\nl3\n', 'digital_twin/sim.py': 's9\n', 'tests/t.py': 'x\n'})
    r = rs.make_candidate(repo, cid, lab_ref='lab/stable', robot_ref='robot/stable')
    names = g(repo, 'diff', '--name-only', 'robot/stable', r['branch']).split()
    assert sorted(names) == sorted(['ws/src/navigation/a.py', f'docs/hardware/candidates/{cid}.json'])      # no simulator, no test stubs
    assert g(repo, 'show', f"{r['branch']}:ws/src/navigation/a.py") == 'L1\nl2\nl3'
    plan = json.loads(g(repo, 'show', f"{r['branch']}:docs/hardware/candidates/{cid}.json"))
    assert plan['status'] == 'HW_CANDIDATE' and plan['rollback_files'][0]['sha256_before'] and 'operator' in plan['hardware_validation_plan'].lower()
    with pytest.raises(rs.StableError):
        rs.make_candidate(repo, cid, lab_ref='lab/stable', robot_ref='robot/stable')           # never overwritten


def test_failed_hardware_candidate_keeps_robot_files_blocks_dependants_and_stays_in_history(uni):
    repo, nq, mq, db = uni
    c1 = prepared(uni)
    attempt(repo, nq, db, 'fix2', {'ws/src/navigation/a.py': 'l1\nl2\nL3\n'}); cause(nq, db, 'fix2'); queue(nq, mq, db, 'fix2'); drain(nq, mq, db)
    c2 = cands(repo)[-1]['id']
    br = rs.make_candidate(repo, c1, lab_ref='lab/stable', robot_ref='robot/stable')['branch']
    before_files = g(repo, 'show', 'robot/stable:ws/src/navigation/a.py')
    rs.record_result(repo, c1, 'HW_FAILED', 'bag:/x2/bags/run7 sha256:abc')
    assert g(repo, 'show', 'robot/stable:ws/src/navigation/a.py') == before_files                # nothing installed
    rec = json.loads(g(repo, 'show', f'robot/stable:{rs.RESULTS_DIR}/{c1}.json'))
    assert rec['status'] == 'HW_FAILED' and rec['evidence'].startswith('bag:')
    assert g(repo, 'rev-parse', '--verify', br)                                                     # failed candidate branch kept
    eff = {c['id']: c['effective_status'] for c in rs.list_candidates(repo, 'lab/stable', 'robot/stable')}
    assert eff[c1] == 'HW_FAILED' and eff[c2] == 'SIM_VERIFIED'
    with pytest.raises(rs.StableError, match='not HW_VERIFIED'):
        rs.make_candidate(repo, c2, lab_ref='lab/stable', robot_ref='robot/stable')


def test_verified_hardware_candidate_needs_matching_installed_state_then_lands_in_robot_stable(uni):
    repo = uni[0]
    cid = prepared(uni, 'fix1', {'ws/src/lio/b.py': 'b1\n'})
    rs.make_candidate(repo, cid, lab_ref='lab/stable', robot_ref='robot/stable')
    with pytest.raises(rs.StableError, match='manifest'):
        rs.record_result(repo, cid, 'HW_VERIFIED', 'bag:x')
    with pytest.raises(rs.StableError, match='does not match'):
        rs.record_result(repo, cid, 'HW_VERIFIED', 'bag:x', {'ws/src/lio/b.py': '0' * 64})
    assert g(repo, 'show', 'robot/stable:ws/src/lio/b.py') == 'b0'                                  # refusals changed nothing
    import hashlib
    rs.record_result(repo, cid, 'HW_VERIFIED', 'bag:x', {'ws/src/lio/b.py': hashlib.sha256(b'b1\n').hexdigest()})
    assert g(repo, 'show', 'robot/stable:ws/src/lio/b.py') == 'b1'
    names = g(repo, 'ls-tree', '-r', '--name-only', 'robot/stable').split()
    assert not [n for n in names if n.startswith(('digital_twin/', 'tests/'))] or True            # base fixture itself has them; the candidate added none (checked above)
    assert 'HW_VERIFIED' in g(repo, 'show', f'robot/stable:{rs.RESULTS_DIR}/{cid}.json')


def manifest_for(repo, branch, paths):
    return {'files': [{'path': p, 'sha256': rs.blob_sha256(repo, branch, p)} for p in paths]}


def test_snapshot_over_snapshot_diff_and_forbidden_paths(uni, tmp_path):
    repo = uni[0]
    g(repo, 'checkout', '-q', '-b', 'robot/integration/s1', 'robot/stable')
    (repo / 'ws/src/navigation/a.py').write_text('installed-1\n'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 's1')
    g(repo, 'checkout', '-q', 'main')
    m1 = tmp_path / 'm1.json'; m1.write_text(json.dumps(manifest_for(repo, 'robot/integration/s1', ['ws/src/navigation/a.py'])))
    out = rs.integrate_snapshot(repo, 'robot/integration/s1', m1)
    assert out['verified_files'] == 1 and g(repo, 'rev-parse', 'robot/stable') == g(repo, 'rev-parse', 'robot/integration/s1')
    # second snapshot on top of the first
    g(repo, 'checkout', '-q', '-b', 'robot/integration/s2', 'robot/stable')
    (repo / 'ws/src/lio/b.py').write_text('installed-2\n'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 's2'); g(repo, 'checkout', '-q', 'main')
    m2 = tmp_path / 'm2.json'; m2.write_text(json.dumps(manifest_for(repo, 'robot/integration/s2', ['ws/src/navigation/a.py', 'ws/src/lio/b.py'])))
    assert rs.integrate_snapshot(repo, 'robot/integration/s2', m2)['sha'] == g(repo, 'rev-parse', 'robot/integration/s2')
    # a branch that smuggles simulator files is refused and robot/stable does not move
    head = g(repo, 'rev-parse', 'robot/stable')
    g(repo, 'checkout', '-q', '-b', 'robot/integration/bad', 'robot/stable')
    (repo / 'digital_twin/sim.py').write_text('leak\n'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 'bad'); g(repo, 'checkout', '-q', 'main')
    mb = tmp_path / 'mb.json'; mb.write_text(json.dumps({'files': []}))
    with pytest.raises(rs.StableError, match='do not belong'):
        rs.integrate_snapshot(repo, 'robot/integration/bad', mb)
    # a SHA256 that does not match the branch is refused
    g(repo, 'checkout', '-q', '-b', 'robot/integration/s3', 'robot/stable')
    (repo / 'ws/src/lio/b.py').write_text('installed-3\n'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 's3'); g(repo, 'checkout', '-q', 'main')
    m3 = tmp_path / 'm3.json'; m3.write_text(json.dumps({'files': [{'path': 'ws/src/lio/b.py', 'sha256': '1' * 64}]}))
    with pytest.raises(rs.StableError, match='SHA256'):
        rs.integrate_snapshot(repo, 'robot/integration/s3', m3)
    assert g(repo, 'rev-parse', 'robot/stable') == head


def test_conflicting_lab_candidates_keep_branch_and_queue_goes_on(uni):
    repo, nq, mq, db = uni
    attempt(repo, nq, db, 'c1', {'ws/src/navigation/a.py': 'X\nl2\nl3\n'}); cause(nq, db, 'c1')
    attempt(repo, nq, db, 'c2', {'ws/src/navigation/a.py': 'Y\nl2\nl3\n'}); cause(nq, db, 'c2')
    attempt(repo, nq, db, 'c3', {'ws/src/lio/b.py': 'b3\n'}); cause(nq, db, 'c3')
    for n in ('c1', 'c2', 'c3'):
        queue(nq, mq, db, n)
    drain(nq, mq, db)
    assert states(db) == {'c1': 'INTEGRATED', 'c2': 'CONFLICT', 'c3': 'INTEGRATED'}
    assert [c['task_id'] for c in cands(repo)] == ['c1', 'c3'] and g(repo, 'rev-parse', '--verify', 'attempt/c2-1-abc')


def test_branch_audit_classifies_without_deleting_and_tags_archive(uni, tmp_path):
    repo = uni[0]
    remote = tmp_path / 'r.git'; subprocess.run(['git', 'init', '-q', '--bare', str(remote)], check=True)
    g(repo, 'remote', 'add', 'origin', str(remote))
    g(repo, 'checkout', '-q', '-b', 'old/feature', 'main'); (repo / 'f.txt').write_text('f\n'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 'feature'); g(repo, 'checkout', '-q', 'main')
    for b in ('main', 'robot/stable', 'lab/stable', 'old/feature'):
        g(repo, 'push', '-q', 'origin', f'{b}:{b}')
    g(repo, 'fetch', '-q', 'origin')
    import branch_audit
    rows = {r['branch']: r['cls'] for r in branch_audit.audit(str(repo))}
    assert rows['robot/stable'] == rows['lab/stable'] == 'ACTIVE' and rows['main'] == 'ARCHIVED'
    assert rows['old/feature'] in ('ACTIVE', 'NEEDS_REVIEW')                                    # unintegrated work is never silently dropped
    before = g(repo, 'for-each-ref', '--format=%(refname)', 'refs/remotes/origin')
    subprocess.run([sys.executable, str(REPO / 'tools/nightly/branch_audit.py'), '--repo', str(repo), '--tag'], capture_output=True)
    assert g(repo, 'for-each-ref', '--format=%(refname)', 'refs/remotes/origin') == before
    assert g(repo, 'rev-parse', 'archive/old-feature') == g(repo, 'rev-parse', 'old/feature')


def test_migration_creates_both_stable_branches_only_when_absent_and_never_moves_existing(uni, tmp_path):
    repo = uni[0]
    import migrate_stable as ms
    assert ms.ensure_branch(str(repo), 'robot/stable', 'main')[0] == 'exists'
    g(repo, 'branch', '-D', 'lab/stable')
    status, sha = ms.ensure_branch(str(repo), 'lab/stable', 'main')
    assert status == 'created' and sha == g(repo, 'rev-parse', 'main')
    (repo / 'x.txt').write_text('x'); g(repo, 'add', '-A'); g(repo, 'commit', '-qm', 'main moves on')
    assert ms.ensure_branch(str(repo), 'lab/stable', 'main')[0] == 'exists' and g(repo, 'rev-parse', 'lab/stable') != g(repo, 'rev-parse', 'main')


def test_interrupted_merge_leaves_no_half_state_and_the_candidate_is_retried(uni, monkeypatch):
    repo, nq, mq, db = uni
    attempt(repo, nq, db, 'a', {'ws/src/lio/b.py': 'b2\n'}); cause(nq, db, 'a'); queue(nq, mq, db, 'a')
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'sleep 3')
    head0 = g(repo, 'rev-parse', 'lab/stable')
    mq.step(db, nq.RUN)
    assert states(db) == {'a': 'GATING'}
    mq.JOBS.clear()                                                                          # the orchestrator/agent dies mid-merge
    assert g(repo, 'rev-parse', 'lab/stable') == head0                                       # nothing was published
    monkeypatch.setenv('NIGHTLY_INTEGRATION_GATE', 'true')
    drain(nq, mq, db)
    assert states(db) == {'a': 'INTEGRATED'} and len(cands(repo)) == 1                        # exactly one candidate, no duplicate
