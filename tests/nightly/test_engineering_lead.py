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
    import subprocess
    pending=nq.ready(a);assert next(pending)['name']=='a'
    subprocess.run([sys.executable,'-c',"import sqlite3,sys; c=sqlite3.connect(sys.argv[1],isolation_level=None);c.execute(\"INSERT INTO kv VALUES('second-writer','1')\");c.close()",str(tmp_path/'q.db')],check=True)
    a.execute("INSERT INTO kv VALUES('dispatcher','1')")
    assert next(pending)['name']=='b'
    a.close()


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


def test_trusted_probe_cannot_escape_into_parent_shell():
    assert lead.trusted_probe('python3 tools/nightly/workflow.py capability-probe --provider codex --model gpt-6.1-sol')
    for bad in ('$(id)','foo;id','`id`','foo\nid'):
        assert not lead.trusted_probe('python3 tools/nightly/workflow.py capability-probe --provider codex --model '+bad)


def test_fingerprint_ignores_volatile_provider_state_but_tracks_config(tmp_path,monkeypatch):
    """~/.claude.json is rewritten by every CLI/interactive session; it must not invalidate a capability."""
    import time
    from pathlib import Path
    home=tmp_path;(home/'.claude').mkdir();(home/'.codex').mkdir()
    (home/'.claude.json').write_text('{"n":1}');(home/'.claude/settings.json').write_text('{}')
    (home/'.codex/config.toml').write_text('a=1');(home/'.codex/auth.json').write_text('{}')
    monkeypatch.setattr(Path,'home',classmethod(lambda cls:home))
    monkeypatch.setenv('HOME',str(home))
    for provider in ('claude','codex'):
        before=lead.fingerprint(provider,'m')
        time.sleep(0.01);(home/'.claude.json').write_text('{"n":2,"startups":99}')
        assert lead.fingerprint(provider,'m')==before
    c=lead.fingerprint('claude','m');(home/'.claude/settings.json').write_text('{"model":"x"}')
    assert lead.fingerprint('claude','m')!=c
    x=lead.fingerprint('codex','m');(home/'.codex/config.toml').write_text('a=2')
    assert lead.fingerprint('codex','m')!=x


def test_capability_helper_preserves_daemon_environment_without_login_shell(db,tmp_path,monkeypatch):
    import execution,subprocess
    monkeypatch.setattr(nq,'ROOT',tmp_path)
    monkeypatch.setattr(nq,'RUN',tmp_path)
    monkeypatch.setattr(nq,'LOGS',tmp_path)
    monkeypatch.setattr(nq,'checkpoint',lambda *a:None)
    monkeypatch.setattr(nq,'git',lambda *a:'a'*40)
    monkeypatch.setattr(execution,'managed',lambda root:True)
    monkeypatch.setenv('PATH','/usr/local/bin:/usr/bin:/bin:/home/adn/.local/bin')
    seen={}
    class Child:
        pid=123
    def launch(argv,**kwargs):
        seen.update(argv=argv,env=kwargs['env']);return Child()
    monkeypatch.setattr(subprocess,'Popen',launch)
    cmd='python3 tools/nightly/workflow.py capability-probe --provider codex --model gpt-6.1-sol'
    nq.enqueue(db,'probe',cmd,execution_tier='ENGINEERING',branch='attempt/t-1')
    row=db.execute("SELECT * FROM tasks WHERE name='probe'").fetchone()
    nq.run_mechanical(db,row)
    assert seen['argv']==[sys.executable,str(tmp_path/'tools/nightly/workflow.py'),'capability-probe','--provider','codex','--model','gpt-6.1-sol']
    assert seen['env']['PATH']=='/usr/local/bin:/usr/bin:/bin:/home/adn/.local/bin'
    assert seen['env']['ROS_LOCALHOST_ONLY']=='1'


def test_claude_streams_events_so_heartbeat_sees_progress():
    """text mode prints nothing until exit: a 15 min analysis was SIGKILLed as 'hang' (empty log, exit -9)."""
    argv=shlex.split(agents.ClaudeAdapter().command(dict(repro='x'),''))
    opts=argv[:argv.index('--')]
    assert opts[opts.index('--output-format')+1]=='stream-json' and '--verbose' in opts


def test_claude_final_text_only_from_successful_result_event():
    ok='\n'.join([json.dumps(dict(type='system',subtype='init')),json.dumps(dict(type='assistant',message='partial')),
                  json.dumps(dict(type='result',subtype='success',is_error=False,result='TOKEN'))])
    assert lead.claude_final_text(ok)=='TOKEN'
    assert lead.claude_final_text(ok.replace('"is_error": false','"is_error": true'))is None
    assert lead.claude_final_text('TOKEN')is None
    assert lead.claude_final_text(json.dumps(dict(type='assistant',message='TOKEN')))is None


def test_probe_pending_candidate_is_not_parked_until_other_providers_reset(db,tmp_path,monkeypatch):
    """claude only awaits its minutes-long qualification probe; codex is limited for ~2h. The task must recheck soon."""
    import execution,time
    for col in ('task_type','complexity','risk','pin_model','pin_effort'):
        db.execute('ALTER TABLE tasks ADD COLUMN '+col+' TEXT')
    monkeypatch.setattr(execution,'managed',lambda root:True)
    monkeypatch.setattr(nq,'ROOT',tmp_path)
    now=time.time()
    states={'codex':{'installed':True,'status':'usage_limited','retry_after':now+7000},'claude':{'installed':True,'status':'unknown'}}
    monkeypatch.setattr(nq,'agent_probe',lambda db:states);monkeypatch.setattr(nq,'refresh_usage',lambda db:states)
    monkeypatch.setattr(nq.router,'route',lambda *a,**k:[dict(agent='codex',model='m1',effort='low'),dict(agent='claude',model='m2',effort='low')])
    monkeypatch.setattr(lead,'fingerprint',lambda *a:'facts')
    nq.enqueue(db,'engineering_x','true',kind='engineering',repro='--agents=codex,claude',priority=50)
    row=db.execute("SELECT * FROM tasks WHERE name='engineering_x'").fetchone()
    sel=nq.select_agent(db,row)
    assert sel['wait_until']-now<=nq.PROBE_RECHECK_S+10, sel
    assert 'probe' in sel['reason']
    nq.kv_set(db,'execution_capability:claude:m2',dict(fingerprint='facts',status='EXEC_VERIFIED',ts=100))
    assert nq.select_agent(db,row)['agent']=='claude'


def _attempt_repo(tmp_path, db, listed):
    import subprocess
    g=lambda *a:subprocess.run(['git','-C',str(tmp_path),*a],check=True,capture_output=True,text=True).stdout.strip()
    g('init','-q');g('config','user.email','a@b');g('config','user.name','t')
    (tmp_path/'a.txt').write_text('a');g('add','.');g('commit','-qm','base');base=g('rev-parse','HEAD')
    (tmp_path/'fix.txt').write_text('fix');g('add','fix.txt');g('commit','-qm','attempt')   # .qazbot is never committed
    own=tmp_path/'.qazbot/results/t.1.json';own.parent.mkdir(parents=True)
    nq.kv_set(db,'base:t:1',{'git_sha':base})
    row=dict(name='t',attempts=1,attempt_id='aid',attempt_wt=str(tmp_path),cwd=str(tmp_path),execution_tier='ENGINEERING',branch='attempt/t-1')
    own.write_text(json.dumps(dict(task='t',attempt=1,attempt_id='aid',summary='s',tests=dict(cmd='python3 -m pytest -q x'),files=[dict(path=p) for p in listed])))
    return row,own


def test_result_file_listing_itself_is_accepted_if_a_real_artefact_is_also_listed(db,tmp_path):
    """6 attempts (Codex rc 0, real analysis) were rejected 'file not changed by this attempt: .qazbot/results/<task>.<n>.json'."""
    row,own=_attempt_repo(tmp_path,db,['fix.txt','.qazbot/results/t.1.json'])
    ok,_,why=nq._verify(db,row,own)
    assert ok,why


def test_result_file_alone_or_other_untracked_files_still_rejected(db,tmp_path):
    row,own=_attempt_repo(tmp_path,db,['.qazbot/results/t.1.json'])
    ok,_,why=nq._verify(db,row,own)
    assert not ok and 'besides the result file' in why
    (tmp_path/'.qazbot/other.json').write_text('{}')
    row,own=_attempt_repo.__wrapped__ if False else (row,own)
    own.write_text(json.dumps(dict(task='t',attempt=1,attempt_id='aid',summary='s',tests=dict(cmd='python3 -m pytest -q x'),files=[dict(path='.qazbot/other.json')])))
    ok,_,why=nq._verify(db,row,own)
    assert not ok and 'not changed' in why
