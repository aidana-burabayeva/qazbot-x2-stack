"""Targeted campaign policy with real SQLite; no agents, daemon, containers or engineering runs."""
import fcntl
import json
import sqlite3
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/nightly'))
import execution
import lab
import labctl
import nq
import workflow


@pytest.fixture
def db(tmp_path, monkeypatch):
    run = tmp_path/'run'
    monkeypatch.setattr(nq, 'ROOT', tmp_path)
    monkeypatch.setattr(nq, 'RUN', run)
    monkeypatch.setattr(nq, 'DB', run/'queue.db')
    monkeypatch.setattr(nq, 'LOGS', run/'logs')
    monkeypatch.setattr(nq, 'checkpoint', lambda *a: None)
    c = nq.connect()
    for i in range(79):
        nq.enqueue(c, 'legacy_'+str(i), 'true', priority=-100,
                   state=('PENDING', 'PAUSED_LIMIT', 'PAUSED_AGENT', 'COMPLETED', 'NEEDS_ENGINEERING')[i % 5])
    for i in range(11):
        c.execute('INSERT INTO results(result_key,run_id,task,attempt,scenario_id,verdict) VALUES(?,?,?,?,?,?)',
                  ('fixture_'+str(i), 'historical_fixture', 'legacy_'+str(i), 1, 'fixture', 'UNVERIFIED'))
    yield c
    c.close()


def history(db):
    return ([tuple(r) for r in db.execute("SELECT * FROM tasks WHERE name LIKE 'legacy_%' ORDER BY id")],
            [tuple(r) for r in db.execute('SELECT * FROM results ORDER BY id')])


def activate(db):
    nq.kv_set(db, 'campaign_scope', execution.X2_CAMPAIGN)
    labctl.prepare_campaign(db)


def test_exact_three_idempotent_and_all_general_generators_disabled(db):
    before = history(db)
    activate(db); activate(db)
    workflow.enqueue_engineering(db)
    assert workflow.daily(db) == []
    workflow.ingest(db, dict(component='amcl', error='fixture failure'))
    assert 'skipped' in lab.robot_sync(db)[0]
    assert not nq.enqueue(db, 'fidelity_audit_forbidden', 'true')
    assert history(db) == before
    assert db.execute('SELECT count(*) FROM tasks').fetchone()[0] == 82
    assert [r['name'] for r in nq.ready(db)] == [execution.X2_TASKS[0]]


def test_sequential_order_including_blocked_outcome(db):
    activate(db)
    for name, outcome in zip(execution.X2_TASKS, ('NEEDS_HARDWARE', 'COMPLETED', 'FAILED')):
        assert [r['name'] for r in nq.ready(db)] == [name]
        db.execute("UPDATE tasks SET state='VERIFYING' WHERE name=?", (name,))
        assert list(nq.ready(db)) == []
        db.execute('UPDATE tasks SET state=? WHERE name=?', (outcome, name))
    assert list(nq.ready(db)) == []  # no fall-through to old work when campaign ends


@pytest.mark.parametrize('state', execution.ACTIVE_TASK_STATES)
def test_foreign_live_lease_blocks_dispatch_and_recovery_without_release(db, monkeypatch, state):
    activate(db)
    db.execute('UPDATE tasks SET state=?,pid=123456 WHERE name=?', (state, 'legacy_0'))
    before = history(db)
    monkeypatch.setattr(nq.os, 'killpg', lambda *a: pytest.fail('foreign process signalled'))
    assert not execution.scope_idle(db)
    assert list(nq.ready(db)) == []
    with pytest.raises(SystemExit, match='leases retained'):
        nq.recover(db)
    assert history(db) == before


def test_unconfirmed_delivery_and_merge_gate_are_serial_barriers(db):
    activate(db)
    nq.kv_set(db, 'delegated_open', ['legacy_delivery'])
    assert list(nq.ready(db)) == []
    nq.kv_set(db, 'delegated_open', [])
    db.execute("INSERT INTO merge_queue(task,repo,branch,sha,state) VALUES('legacy_0','/fixture','b','sha','GATING')")
    assert list(nq.ready(db)) == []


def test_direct_dispatch_cannot_bypass_scope_or_concurrent_task(db):
    activate(db)
    legacy = db.execute("SELECT * FROM tasks WHERE name='legacy_0'").fetchone()
    with pytest.raises(RuntimeError, match='refuses dispatch'):
        nq.run_mechanical(db, legacy)
    with pytest.raises(RuntimeError, match='refuses dispatch'):
        nq.dispatch_codex(db, legacy)
    db.execute("UPDATE tasks SET state='RUNNING' WHERE name=?", (execution.X2_TASKS[0],))
    second = db.execute('SELECT * FROM tasks WHERE name=?', (execution.X2_TASKS[1],)).fetchone()
    with pytest.raises(RuntimeError, match='refuses dispatch'):
        nq.run_mechanical(db, second)


def test_tick_and_revive_do_not_requeue_old_paused_tasks(db, monkeypatch):
    activate(db)
    before = history(db)
    monkeypatch.setattr(nq, 'select_agent', lambda *a: None)
    monkeypatch.setattr(nq, 'run_mechanical', lambda *a: pytest.fail('process started'))
    assert nq.tick(db, None) is None
    nq.kv_set(db, 'agents', {'claude': {'installed': True, 'status': 'unknown'}})
    nq.revive_agents(db)
    assert history(db) == before


def test_unknown_scope_fails_closed(db):
    nq.kv_set(db, 'campaign_scope', 'misspelled-scope')
    assert list(nq.ready(db)) == []
    assert not nq.enqueue(db, 'unexpected', 'true')


def test_plan_copies_live_db_to_memory_and_never_calls_agents(db, monkeypatch, capsys):
    before = history(db)
    monkeypatch.setattr(labctl, 'RUN', nq.RUN)
    monkeypatch.setattr(nq, 'agent_probe', lambda *a: pytest.fail('agent probe'))
    monkeypatch.setattr(nq, 'select_agent', lambda *a: pytest.fail('routing dispatch'))
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **k: pytest.fail('subprocess started'))
    assert labctl.plan(execution.X2_CAMPAIGN) == 0
    plan = json.loads(capsys.readouterr().out)
    assert len(plan['tasks']) == 3 and len(plan['excluded_tasks']) == 79
    assert plan['max_parallel'] == 1 and plan['agents_started'] == 0
    assert plan['next_task'] == execution.X2_TASKS[0]
    assert not plan['automatic_general_enqueue']
    assert history(db) == before and db.execute('SELECT count(*) FROM tasks').fetchone()[0] == 79
    assert nq.kv_get(db, 'campaign_scope') is None


def test_existing_orchestrator_lock_refuses_start_without_probing(db, monkeypatch):
    monkeypatch.setattr(labctl, 'RUN', nq.RUN)
    monkeypatch.setattr(labctl, '_start', lambda *a, **k: pytest.fail('startup reached'))
    with (nq.RUN/'orchestrator.lock').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert labctl.start(campaign=execution.X2_CAMPAIGN) == 1
        # A failed contender has not released our lock.
        with (nq.RUN/'orchestrator.lock').open('a') as other:
            with pytest.raises(BlockingIOError):
                fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)


def test_targeted_resume_does_not_restore_old_parked_or_triage(db, monkeypatch):
    activate(db)
    nq.kv_set(db, 'parked', {'legacy_0': 'PENDING'})
    nq.kv_set(db, 'last_lab_stop_sha', 'old')
    before = history(db)
    monkeypatch.setattr(nq, 'connect', lambda: db)
    monkeypatch.setattr(lab, 'HW', nq.ROOT/'docs/hardware')
    monkeypatch.setattr(lab, 'git_view', lambda: dict(dirty=[], upstream=None, behind=0, ahead=0, branch='lab/control', head='control'))
    monkeypatch.setattr(lab, 'sync_control', lambda *a: None)
    monkeypatch.setattr(lab, 'gout', lambda *a: 'sha')
    monkeypatch.setattr(lab, 'triage', lambda *a: pytest.fail('generic triage'))
    monkeypatch.setattr(lab, 'write_pointers', lambda *a: pytest.fail('control files changed'))
    monkeypatch.setattr(nq.mergeq, 'integration_head', lambda *a: 'integration')
    lab.cmd_resume(SimpleNamespace(no_start=True, no_systemd=True, no_fetch=True))
    assert history(db) == before
    assert nq.kv_get(db, 'parked') == {'legacy_0': 'PENDING'}


def test_targeted_inline_capability_probe_has_finite_timeout_and_no_side_task(db, monkeypatch):
    activate(db)
    monkeypatch.setattr(nq.execution, 'managed', lambda *a: True)
    monkeypatch.setattr(nq, 'agent_probe', lambda *a: {'codex': {'installed': True}, 'claude': {'installed': False}})
    monkeypatch.setattr(nq, 'refresh_usage', lambda *a: None)
    monkeypatch.setattr(nq.router, 'route', lambda *a, **k: [{'agent':'codex','model':'fixture','effort':'high'}])
    monkeypatch.setattr(nq.lead, 'health', lambda *a: None)
    monkeypatch.setattr(nq.lead, 'fingerprint', lambda *a: 'current')
    monkeypatch.setattr(nq.lead, 'ensure_probe', lambda *a: pytest.fail('fourth queue task'))
    calls=[]
    def probe(queue, provider, model, timeout_s):
        calls.append((provider,model,timeout_s))
        nq.kv_set(db, 'execution_capability:codex:fixture', dict(fingerprint='current',status='EXEC_VERIFIED',ts=1))
        return {'passed':True}
    monkeypatch.setattr(nq.lead, 'probe', probe)
    before=history(db)
    row=db.execute('SELECT * FROM tasks WHERE name=?',(execution.X2_TASKS[0],)).fetchone()
    selected=nq.select_agent(db,row)
    assert selected['agent']=='codex' and calls==[('codex','fixture',120)]
    assert db.execute('SELECT count(*) FROM tasks').fetchone()[0]==82
    assert history(db)==before


def test_targeted_start_prepares_scope_frozen_before_existing_resume(db, monkeypatch):
    before=history(db)
    monkeypatch.setattr(labctl, 'ROOT', nq.ROOT)
    monkeypatch.setattr(labctl, 'RUN', nq.RUN)
    monkeypatch.setattr(lab, 'sh', lambda *a, **k: (3,'inactive'))
    monkeypatch.setattr(lab, 'sync_control', lambda *a: None)
    monkeypatch.setattr(lab, 'gout', lambda *a: 'sha')
    monkeypatch.setattr(nq.mergeq, 'checked_out_at', lambda *a: None)
    monkeypatch.setattr(nq.mergeq, 'integration_head', lambda *a: 'sha')
    def doctor(mode):
        p=nq.RUN/'preflight';p.mkdir()
        (p/'latest.json').write_text(json.dumps(dict(git_head='sha',integration='sha',mode=mode,verdict='PASS',stamp='unit-fixture',campaign_scope=execution.X2_CAMPAIGN)))
        return 0
    monkeypatch.setattr(labctl, 'doctor', doctor)
    calls=[]
    def resume(*argv):
        assert nq.kv_get(db,'campaign_scope')==execution.X2_CAMPAIGN
        assert nq.kv_get(db,'frozen') is True
        assert db.execute('SELECT count(*) FROM tasks').fetchone()[0]==82
        calls.append(argv)
        return 0
    monkeypatch.setattr(labctl, 'call', resume)
    assert labctl.start(campaign=execution.X2_CAMPAIGN)==0
    assert len(calls)==1 and 'resume' in calls[0]
    assert history(db)==before


def test_codex_only_repro_and_no_claude_version_probe(db, monkeypatch):
    activate(db)
    for name in execution.X2_TASKS:
        row=db.execute('SELECT * FROM tasks WHERE name=?',(name,)).fetchone()
        assert nq.db_get_agents(row)==['codex']
    monkeypatch.setattr(nq.agents.ADAPTERS['claude'],'version',lambda:pytest.fail('Claude called before review phase'))
    monkeypatch.setattr(nq.agents.ADAPTERS['codex'],'version',lambda:'fixture')
    assert nq.agent_probe(db)['codex']['installed']


def test_limited_codex_waits_without_claude_fallback_or_probe_loop(db, monkeypatch):
    activate(db)
    reset=nq.time.time()+3600
    statuses={'codex':dict(installed=True,status='usage_limited',retry_after=reset),
              'claude':dict(installed=True,status='available')}
    monkeypatch.setattr(nq,'agent_probe',lambda *a:statuses)
    monkeypatch.setattr(nq,'refresh_usage',lambda *a:statuses)
    monkeypatch.setattr(nq.lead,'probe',lambda *a,**k:pytest.fail('limited provider called'))
    row=db.execute('SELECT * FROM tasks WHERE name=?',(execution.X2_TASKS[0],)).fetchone()
    result=nq.select_agent(db,row)
    assert result['wait_until']>=reset and 'agent' not in result
    db.execute("UPDATE tasks SET owner='codex',attempt_wt='/fixture/kept',branch='attempt/kept' WHERE name=?",(row['name'],))
    nq.kv_set(db,'agents',statuses)
    full=db.execute('SELECT * FROM tasks WHERE name=?',(row['name'],)).fetchone()
    state,reason=nq.handoff_decision(db,full,1,'fail','rate limit')
    assert state=='PAUSED_LIMIT' and 'checkpoint/worktree retained' in reason
    db.execute('UPDATE tasks SET state=? WHERE name=?',(state,row['name']))
    monkeypatch.setattr(nq,'select_agent',lambda *a:pytest.fail('request repeated before reset'))
    for _ in range(3): assert nq.tick(db,None) is None
    kept=db.execute('SELECT * FROM tasks WHERE name=?',(row['name'],)).fetchone()
    assert kept['attempt_wt']=='/fixture/kept' and kept['branch']=='attempt/kept'


def test_scoped_stop_parks_only_target_tasks_and_retains_old_leases(db, monkeypatch):
    activate(db)
    db.execute("UPDATE tasks SET state='RUNNING',pid=123456 WHERE name='legacy_0'")
    before=history(db)
    monkeypatch.setattr(nq.os,'killpg',lambda *a:pytest.fail('old lease killed'))
    lab.park_tasks(db)
    assert history(db)==before
    assert all(db.execute('SELECT state FROM tasks WHERE name=?',(n,)).fetchone()[0]=='NEEDS_APPROVAL'
               for n in execution.X2_TASKS)


def test_target_preflight_never_authenticates_or_invokes_claude(monkeypatch):
    import preflight
    monkeypatch.setenv('NIGHTLY_CAMPAIGN',execution.X2_CAMPAIGN)
    monkeypatch.setattr(nq.agents.ADAPTERS['claude'],'version',lambda:pytest.fail('Claude preflight call'))
    monkeypatch.setattr(nq.agents.ADAPTERS['codex'],'version',lambda:'fixture')
    calls=[]
    monkeypatch.setattr(preflight,'sh',lambda cmd,**kw:(calls.append(cmd) or (0,'Logged in')))
    monkeypatch.setattr(preflight,'RES',[])
    preflight.check_agents()
    assert calls==['codex login status']
