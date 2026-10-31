"""Execution/plan invariants. Provider E2E is reported separately, never mocked into PASS."""
import json
import shlex
import sqlite3
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/nightly'))
import agents
import lead
import nq


@pytest.fixture
def db():
    c=sqlite3.connect(':memory:',isolation_level=None);c.row_factory=sqlite3.Row;c.executescript(nq.SCHEMA)
    lead.init(c)
    return c


@pytest.mark.parametrize('text,category',[
 ('workspace routing discovery failed\nresult.json missing','PROVIDER_TRANSPORT'),
 ('401 Unauthorized','AUTHENTICATION'),('You have hit your usage limit','RATE_LIMIT'),
 ('bwrap: creating namespace failed','SANDBOX_NETWORK'),('Read-only file system','SANDBOX_NETWORK'),
 ('model is not available','MODEL_UNAVAILABLE'),('Traceback: exception in task','AGENT_EXECUTION'),
 ('database is locked','SQLITE_CONCURRENCY'),('--agents file not found: blah','ADAPTER_INVOCATION')])
def test_primary_failure_categories(text,category):
    assert lead.classify(text,1)[0]==category


def test_missing_result_without_provider_error_is_its_own_failure():
    assert lead.classify('completed without artefact',0)[0]=='MISSING_RESULT'


def test_claude_prompt_cannot_be_a_cli_option():
    argv=shlex.split(agents.ClaudeAdapter().command(dict(repro='--agents=claude,codex find cause'),''))
    assert argv[argv.index('--')+1].startswith('--agents=')
    assert '--agents=claude,codex' not in argv[:argv.index('--')]


def test_one_transport_incident_three_preserved_attempts(db):
    for name in ('amcl','sensor','p23'):
        sig=lead.observe(db,'codex','PROVIDER_TRANSPORT','workspace routing discovery failed','logs/'+name,task=name,attempt=1)
    assert db.execute('SELECT COUNT(*) FROM execution_incidents').fetchone()[0]==1
    assert db.execute('SELECT COUNT(*) FROM execution_affected').fetchone()[0]==3
    assert lead.health(db,'codex')['status']=='BLOCKED'
    lead.observe(db,'codex','EXECUTION_OK','completed challenge','probe.log',success=True)
    assert lead.health(db,'codex')['status']=='EXEC_VERIFIED'


def test_domain_failure_does_not_revoke_an_independent_provider(db):
    lead.observe(db,'codex','EXECUTION_OK','ok','e',success=True)
    category,_=lead.classify('assert AMCL pose mismatch',1)
    assert category not in lead.INFRA
    assert lead.health(db,'codex')['status']=='EXEC_VERIFIED'


def test_navigation_order_and_no_replanning_without_fact_change(db):
    for name,prio in [('engineering_gpu_egl_v3',1),('engineering_p23_motion_v3',2),('engineering_amcl_initialpose_v3',100)]:
        nq.enqueue(db,name,'true',kind='engineering',priority=prio)
    rows=db.execute('SELECT * FROM tasks').fetchall()
    assert lead.order(db,rows)[0]['name']=='engineering_amcl_initialpose_v3'
    count=db.execute('SELECT count(*) FROM planning_decisions').fetchone()[0]
    lead.order(db,rows)
    assert db.execute('SELECT count(*) FROM planning_decisions').fetchone()[0]==count
    db.execute("UPDATE tasks SET state='COMPLETED' WHERE name='engineering_amcl_initialpose_v3'")
    lead.order(db,db.execute('SELECT * FROM tasks').fetchall())
    assert db.execute('SELECT count(*) FROM planning_decisions').fetchone()[0]>count


def test_sqlite_cursor_is_not_held_over_dispatch_and_concurrent_commit(tmp_path,monkeypatch):
    monkeypatch.setattr(nq,'ROOT',tmp_path)
    a=sqlite3.connect(tmp_path/'q.db',isolation_level=None);a.row_factory=sqlite3.Row
    a.execute('PRAGMA journal_mode=WAL');a.executescript(nq.SCHEMA)
    for name in ('a','b'):nq.enqueue(a,name,'true')
    b=sqlite3.connect(tmp_path/'q.db',isolation_level=None)
    pending=nq.ready(a);assert next(pending)['name']=='a'
    b.execute("INSERT INTO kv VALUES('second-writer','1')")
    a.execute("INSERT INTO kv VALUES('dispatcher','1')")
    assert next(pending)['name']=='b'
    a.close();b.close()


def test_app_server_available_does_not_authorize_exec_and_probe_is_deduplicated(db,tmp_path,monkeypatch):
    import execution
    for col in ('task_type','complexity','risk','pin_model','pin_effort'):
        db.execute('ALTER TABLE tasks ADD COLUMN '+col+' TEXT')
    monkeypatch.setattr(execution,'managed',lambda root:True)
    monkeypatch.setattr(nq,'ROOT',tmp_path)
    states={'codex':{'installed':True,'status':'available'},'claude':{'installed':False}}
    monkeypatch.setattr(nq,'agent_probe',lambda db:states)
    monkeypatch.setattr(nq,'refresh_usage',lambda db:states)
    monkeypatch.setattr(nq.router,'route',lambda *a,**k:[dict(agent='codex',model='test',effort='low')])
    monkeypatch.setattr(lead,'fingerprint',lambda *a:'facts')
    nq.enqueue(db,'engineering_amcl','true',kind='engineering',repro='--agents=codex')
    row=db.execute("SELECT * FROM tasks WHERE name='engineering_amcl'").fetchone()
    assert nq.select_agent(db,row) is None
    assert nq.select_agent(db,row) is None
    assert db.execute("SELECT count(*) FROM tasks WHERE name LIKE 'execution_probe_%'").fetchone()[0]==1
    nq.kv_set(db,'execution_capability:codex:test',dict(fingerprint='facts',status='EXEC_VERIFIED',ts=100))
    assert nq.select_agent(db,row)['agent']=='codex'
    lead.observe(db,'codex','PROVIDER_TRANSPORT','workspace routing discovery failed','error.log')
    assert nq.select_agent(db,row) is None


def test_limited_provider_is_not_probed_and_deterministic_work_remains_ready(db,tmp_path,monkeypatch):
    import execution,time
    for col in ('task_type','complexity','risk','pin_model','pin_effort'):
        db.execute('ALTER TABLE tasks ADD COLUMN '+col+' TEXT')
    states={'claude':{'installed':True,'status':'usage_limited','retry_after':time.time()+1000}}
    monkeypatch.setattr(nq,'agent_probe',lambda db:states);monkeypatch.setattr(nq,'refresh_usage',lambda db:states)
    monkeypatch.setattr(nq.router,'route',lambda *a,**k:[dict(agent='claude',model='test',effort='low')])
    monkeypatch.setattr(execution,'managed',lambda root:True)
    nq.enqueue(db,'engineering_amcl','true',kind='engineering',repro='--agents=claude')
    r=db.execute('SELECT * FROM tasks').fetchone()
    assert 'wait_until' in nq.select_agent(db,r)
    assert db.execute("SELECT count(*) FROM tasks WHERE name LIKE 'execution_probe_%'").fetchone()[0]==0
    monkeypatch.setattr(execution,'managed',lambda root:False)
    nq.enqueue(db,'offline_metric','true')
    assert 'offline_metric' in [r['name'] for r in nq.ready(db)]
