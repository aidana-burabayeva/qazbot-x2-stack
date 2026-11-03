"""Real SQLite/Git transition tests, synthetic unit fixtures only; no agents/ROS/Docker started."""
import json
import hashlib
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/nightly'))
import nq,execution,qualification as q
from test_target_campaign import db, activate, history


def git(root,*args):
    p=subprocess.run(['git','-C',str(root),'-c','user.name=fixture','-c','user.email=fixture@localhost',*args],capture_output=True,text=True)
    assert p.returncode==0,p.stderr
    return p.stdout.strip()


def seal(db,ev):
    path=ev/'engineering_result.json';manifest=ev/'MANIFEST.json'
    manifest.write_text(json.dumps({'files':{'engineering_result.json':hashlib.sha256(path.read_bytes()).hexdigest()}}))
    db.execute("UPDATE results SET manifest_sha256=? WHERE result_key='unit-fix'",(hashlib.sha256(manifest.read_bytes()).hexdigest(),))


@pytest.fixture
def candidate(db,monkeypatch):
    activate(db);nq.kv_set(db,'x2_auto_l3_enabled',True)
    root=nq.ROOT
    git(root,'init','-q');(root/'.gitignore').write_text('run/\n');(root/'value.txt').write_text('0')
    git(root,'add','.');git(root,'commit','-qm','unit baseline');base=git(root,'rev-parse','HEAD')
    (root/'tests').mkdir();(root/'tests/test_fix.py').write_text("from pathlib import Path\ndef test_fixed():\n assert Path('value.txt').read_text() == '1'\n")
    (root/'value.txt').write_text('1');git(root,'add','.');git(root,'commit','-qm','unit fix');fix=git(root,'rev-parse','HEAD');git(root,'branch','lab/stable')
    producer=execution.X2_TASKS[1]
    db.execute("UPDATE tasks SET state='COMPLETED',phase='VERIFIED',attempts=1 WHERE name=?",(producer,))
    nq.kv_set(db,'verified_sha:'+producer,fix)
    spec=dict(blocker='d05',fix_sha=fix,baseline_sha=base,root_cause='unit-cause',tests=['tests/test_fix.py'],runtime_tests=['tests/test_fix.py'],
              safety_invariants=True,dependencies_available=True,nav2_version='1.1.20',requires_mc=False,inputs=[])
    evidence=nq.RUN/'fixture-result';evidence.mkdir()
    payload=dict(l3_fixes=[spec],diagnosis={k:'unit-fixture' for k in ('hypotheses','reproduction','causal_evidence','original_fail_retest','regression')})
    (evidence/'engineering_result.json').write_text(json.dumps(payload))
    db.execute("INSERT INTO results(result_key,run_id,task,attempt,scenario_id,verdict,evidence_path) VALUES('unit-fix','unit',?,1,'unit','PASS',?)",(producer,str(evidence)))
    db.execute("UPDATE results SET metrics=? WHERE result_key='unit-fix'",(json.dumps({'git_sha':fix}),))
    seal(db,evidence)
    db.execute("INSERT INTO root_causes(key,summary,status,evidence,task) VALUES('unit-cause','unit fixture','VERIFIED','unit evidence',?)",(producer,))
    db.execute("INSERT INTO merge_queue(task,repo,branch,sha,state,merge_sha) VALUES(?,?,?,?, 'INTEGRATED',?)",(producer,str(root),'fixture',fix,fix))
    monkeypatch.setattr(q,'x2_conditions',lambda root:dict(images={'stack':'stack','sim':'sim'},map='map',parameters='parameters',nav2='version'))
    monkeypatch.setattr(q,'x2_admission_errors',lambda *a:[])
    yield db,root,spec,evidence
    q.X2_JOBS.clear()


def discover(candidate):
    db,root,spec,ev=candidate;q.x2_discover(db,root,nq.RUN)
    return dict(db.execute('SELECT * FROM x2_l3 WHERE blocker="d05"').fetchone())


def test_no_promotion_from_completion_or_unintegrated_result(candidate):
    db,root,*_=candidate
    db.execute('DELETE FROM merge_queue');q.x2_discover(db,root,nq.RUN)
    assert db.execute('SELECT count(*) FROM x2_l3').fetchone()[0]==0
    db.execute("UPDATE tasks SET phase=NULL WHERE name=?",(execution.X2_TASKS[1],));q.x2_discover(db,root,nq.RUN)
    assert db.execute('SELECT count(*) FROM x2_l3').fetchone()[0]==0


def test_verified_root_cause_and_recipe_are_mandatory(candidate):
    db,root,spec,ev=candidate
    (ev/'engineering_result.json').write_text('{}')
    seal(db,ev)
    row=discover(candidate)
    assert row['state']=='L3_BLOCKED' and 'completion alone' in row['detail']
    assert list(nq.ready(db))[0]['name']==execution.X2_TASKS[0]  # independent work remains possible


def test_restarts_deduplicate_and_changed_source_invalidates(candidate):
    db,root,*_=candidate
    row=discover(candidate)
    for _ in range(3):q.x2_discover(db,root,nq.RUN)
    assert db.execute('SELECT count(*) FROM x2_l3').fetchone()[0]==1
    db.execute("UPDATE x2_l3 SET state='L3_PASS'")
    (root/'new.txt').write_text('changed');git(root,'add','.');git(root,'commit','-qm','new source');git(root,'update-ref','refs/heads/lab/stable','HEAD')
    q.x2_discover(db,root,nq.RUN)
    assert db.execute('SELECT count(*) FROM x2_l3').fetchone()[0]==2
    assert db.execute('SELECT source_sha FROM x2_l3 WHERE state="ENGINEERING_VERIFIED"').fetchone()[0]!=row['source_sha']


def test_active_engineering_lease_prevents_preflight_and_is_not_signalled(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);before=history(db)
    db.execute("UPDATE tasks SET state='RUNNING',pid=123456 WHERE name=?",(execution.X2_TASKS[0],))
    monkeypatch.setattr(q,'x2_recover',lambda *a:None)
    monkeypatch.setattr(nq.attempts,'VerifyJob',lambda *a:pytest.fail('preflight beside engineer'))
    q.x2_advance(db,root,nq.RUN)
    assert db.execute('SELECT state FROM x2_l3').fetchone()[0]=='ENGINEERING_VERIFIED'
    assert history(db)==before


def test_failed_preflight_never_grants_capability(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);out=nq.RUN/'l3-fixes'/row['key'];out.mkdir(parents=True)
    db.execute("UPDATE x2_l3 SET state='L3_PREFLIGHT'")
    (out/'preflight.json').write_text(json.dumps(dict(ok=False,reason='map mismatch')))
    monkeypatch.setattr(execution,'authorize',lambda *a:pytest.fail('bad preflight authorized'))
    q.x2_advance(db,root,nq.RUN)
    assert db.execute('SELECT state,detail FROM x2_l3').fetchone()[:]==('L3_BLOCKED','map mismatch')
    assert not db.execute("SELECT 1 FROM tasks WHERE name LIKE 'x2_l3_%'").fetchone()


def test_each_run_requires_new_ordinary_authorization_and_single_dispatch(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);out=nq.RUN/'l3-fixes'/row['key'];out.mkdir(parents=True)
    conditions=q.x2_case_conditions(root,json.loads(row['spec']))
    report=dict(mode=execution.L3,verdict='PASS',stamp='unit',integration=row['source_sha'])
    (out/'preflight.json').write_text(json.dumps(dict(ok=True,report=report,conditions=conditions)))
    db.execute("UPDATE x2_l3 SET state='L3_PREFLIGHT'")
    monkeypatch.setattr(q,'first_l3_errors',lambda *a:[])
    q.x2_advance(db,root,nq.RUN)
    auth=nq.kv_get(db,'execution_authorization');assert auth['l3_remaining']==1
    tasks=[r for r in nq.ready(db)];assert len(tasks)==1 and tasks[0]['execution_tier']==execution.L3
    execution.consume_l3(db,tasks[0]['name'],1)
    with pytest.raises(ValueError,match='exhausted'):execution.consume_l3(db)
    db.execute("UPDATE tasks SET state='RUNNING' WHERE name=?",(tasks[0]['name'],))
    assert list(nq.ready(db))==[]


def test_orphan_preflight_cannot_restart_or_duplicate(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);out=nq.RUN/'l3-fixes'/row['key'];out.mkdir(parents=True)
    db.execute("UPDATE x2_l3 SET state='L3_PREFLIGHT',pid=99999999")
    monkeypatch.setattr(nq.attempts,'VerifyJob',lambda *a:pytest.fail('duplicate preflight'))
    q.x2_advance(db,root,nq.RUN)
    assert db.execute('SELECT state FROM x2_l3').fetchone()[0]=='L3_BLOCKED'


def test_real_original_fail_and_after_regression_proof(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);out=nq.RUN/'proof';out.mkdir()
    monkeypatch.setattr(execution,'engineering_sandbox',lambda argv,*a,**kw:argv)
    result=q.x2_proof_fix(db,row,root,nq.RUN,out)
    assert result['ok']
    assert result['measurements']['before']['rc']==1
    assert result['measurements']['after']['rc']==0


def test_stop_cancels_own_transition_and_blocks_dispatch(candidate,monkeypatch):
    import lab
    db,root,*_=candidate;discover(candidate)
    db.execute("UPDATE x2_l3 SET state='L3_PREFLIGHT'")
    monkeypatch.setattr(q.os,'killpg',lambda *a:pytest.fail('foreign pid signalled'))
    lab.freeze(db)
    assert list(nq.ready(db))==[]
    assert db.execute('SELECT state FROM x2_l3').fetchone()[0]=='L3_BLOCKED'


def test_d04_d07_shared_commit_is_not_two_independent_fixes(candidate):
    db,root,spec,ev=candidate
    producer=execution.X2_TASKS[2]
    db.execute('UPDATE tasks SET state="COMPLETED",phase="VERIFIED",attempts=1 WHERE name=?',(producer,));nq.kv_set(db,'verified_sha:'+producer,spec['fix_sha'])
    db.execute('UPDATE root_causes SET task=?',(producer,));db.execute('UPDATE results SET task=? WHERE result_key="unit-fix"',(producer,));db.execute('UPDATE merge_queue SET task=?',(producer,))
    payload=json.loads((ev/'engineering_result.json').read_text());payload['l3_fixes']=[dict(spec,blocker=b) for b in ('d04','d07')];(ev/'engineering_result.json').write_text(json.dumps(payload))
    seal(db,ev)
    q.x2_discover(db,root,nq.RUN)
    assert db.execute("SELECT count(*) FROM x2_l3 WHERE state='L3_BLOCKED' AND detail LIKE '%independent%'").fetchone()[0]==2


def test_pending_l3_is_invalidated_when_integration_moves(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate)
    name='x2_l3_d05_unit';db.execute("UPDATE x2_l3 SET state='L3_READY',task=?",(name,))
    nq.enqueue(db,name,'unit',execution_tier=execution.L3,resource='sim')
    (root/'changed.txt').write_text('new');git(root,'add','.');git(root,'commit','-qm','changed source');git(root,'update-ref','refs/heads/lab/stable','HEAD')
    nq.kv_set(db,'frozen',False)
    q.x2_advance(db,root,nq.RUN)
    assert db.execute('SELECT state FROM tasks WHERE name=?',(name,)).fetchone()[0]=='NEEDS_ENGINEERING'
    assert db.execute('SELECT state FROM x2_l3 WHERE key=?',(row['key'],)).fetchone()[0]=='L3_BLOCKED'


def test_completed_l3_returns_engineering_and_new_fix_gets_fresh_capability(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate);out=nq.RUN/'l3-fixes'/row['key'];out.mkdir(parents=True)
    conditions=q.x2_case_conditions(root,json.loads(row['spec']))
    report=dict(mode=execution.L3,verdict='PASS',stamp='unit',integration=row['source_sha'])
    (out/'preflight.json').write_text(json.dumps(dict(ok=True,report=report,conditions=conditions)))
    monkeypatch.setattr(q,'first_l3_errors',lambda *a:[])
    db.execute("UPDATE x2_l3 SET state='L3_PREFLIGHT'");q.x2_advance(db,root,nq.RUN)
    task=db.execute('SELECT * FROM tasks WHERE name LIKE "x2_l3_%"').fetchone()
    execution.consume_l3(db,task['name'],1)
    taskout=nq.RUN/'work'/f"{task['name']}.1";taskout.mkdir(parents=True)
    (taskout/'l3_outcome.json').write_text(json.dumps(dict(status='L3_PASS',reason='unit fixture')))
    db.execute("UPDATE tasks SET state='COMPLETED',attempts=1 WHERE name=?",(task['name'],))
    q.x2_advance(db,root,nq.RUN)
    assert nq.kv_get(db,'execution_mode')==execution.ENGINEERING
    assert nq.kv_get(db,'execution_authorization')['l3_remaining']==0
    # A distinct candidate cannot borrow the consumed grant: it gets authorize() again after a positive preflight.
    newkey='another-unit-fix'
    db.execute('INSERT INTO x2_l3(key,blocker,producer,fix_sha,source_sha,conditions,spec,state,updated) VALUES(?,?,?,?,?,?,?,?,0)',
               (newkey,'d07',execution.X2_TASKS[2],row['fix_sha'],row['source_sha'],row['conditions'],row['spec'],'L3_PREFLIGHT'))
    other=nq.RUN/'l3-fixes'/newkey;other.mkdir();(other/'preflight.json').write_text(json.dumps(dict(ok=True,report=report,conditions=conditions)))
    q.x2_advance(db,root,nq.RUN)
    assert nq.kv_get(db,'execution_authorization')['l3_remaining']==1
    assert db.execute("SELECT count(*) FROM tasks WHERE name LIKE 'x2_l3_%'").fetchone()[0]==2


def test_tampered_stored_evidence_cannot_admit_l3(candidate):
    db,root,spec,ev=candidate
    data=json.loads((ev/'engineering_result.json').read_text());data['l3_fixes'][0]['safety_invariants']=False
    (ev/'engineering_result.json').write_text(json.dumps(data))  # intentionally leave the durable manifest unchanged
    row=discover(candidate)
    assert row['state']=='L3_BLOCKED' and 'missing per-fix' in row['detail']


def test_runtime_entry_requires_real_claim_before_any_process(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate)
    monkeypatch.setattr(q.subprocess,'run',lambda *a,**k:pytest.fail('runtime started without capability'))
    with pytest.raises(SystemExit,match='execution refused'):
        q.x2_runtime_fix(db,row,root,nq.RUN,nq.RUN)


def test_moving_mc_requirement_is_never_substituted_with_lite(candidate,monkeypatch):
    db,root,*_=candidate;row=discover(candidate)
    spec=json.loads(row['spec']);spec['requires_mc']=True;row['spec']=json.dumps(spec)
    monkeypatch.setattr(execution,'require',lambda *a,**k:{'name':row['task']})
    monkeypatch.setattr(q.subprocess,'run',lambda *a,**k:pytest.fail('unqualified MC/lite runtime started'))
    with pytest.raises(ValueError,match='lite substitution refused'):
        q.x2_runtime_fix(db,row,root,nq.RUN,nq.RUN)


def test_unconfirmed_teardown_holds_admission_until_container_absence_proven(candidate,monkeypatch):
    db,root,*_=candidate;discover(candidate)
    db.execute("UPDATE x2_l3 SET state='L3_BLOCKED',teardown_pending=1")
    assert q.x2_busy(db) and list(nq.ready(db))==[]
    monkeypatch.setattr(q.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0,stdout='',stderr=''))
    q.x2_recover(db,root,nq.RUN)
    assert not q.x2_busy(db)
    assert db.execute('SELECT state FROM x2_l3').fetchone()[0]=='L3_BLOCKED'


def test_runtime_binds_explicit_inputs_readonly_and_cleans_only_own_project(candidate,monkeypatch):
    import shutil
    db,root,*_=candidate;row=discover(candidate)
    raw=root/'recorded.bin';raw.write_bytes(b'unit-fixture')
    spec=json.loads(row['spec']);spec['inputs']=[dict(path=str(raw),sha256=hashlib.sha256(raw.read_bytes()).hexdigest())]
    row['spec']=json.dumps(spec);row['conditions']=json.dumps(q.x2_case_conditions(root,spec))
    out=nq.RUN/'unit-runtime';out.mkdir();(root/'digital_twin').mkdir();(root/'digital_twin/.env.lab').write_text('TWIN_MAP=d1_qazbot_20261029\nTWIN_VERSION=dev\n')
    monkeypatch.setattr(execution,'require',lambda *a,**k:{'name':row['task']})
    monkeypatch.setattr(shutil,'disk_usage',lambda *a:SimpleNamespace(free=10*2**30))
    execution.authorize(db,execution.L3,dict(mode=execution.L3,verdict='PASS',stamp='unit-fixture'),root,row['source_sha'])
    original=subprocess.run;calls=[]
    def run(argv,**kw):
        if argv[0]=='git':return original(argv,**kw)
        calls.append(argv)
        if 'run' in argv:
            (out/'runtime.xml').write_text('<testsuite><testcase classname="fixture" name="runtime"/></testsuite>')
            (out/'runtime_contract.json').write_text(json.dumps(dict(fidelity='REAL_ROS2_NAV2',source_sha=row['source_sha'],nav2_version=spec['nav2_version'],safety_violations=0,cases={k:True for k in q.X2_CASES['d05']})))
            (out/'graph_witness.json').write_text(json.dumps(dict(nodes=['controller_server','collision_monitor','goal_coordinator','motion_adapter'],topics={'/cloud':'sensor_msgs/msg/PointCloud2','/cmd':'geometry_msgs/msg/Twist'},counts={'/cloud':1,'/cmd':1},publisher_gids=['unit-gid'])))
        return SimpleNamespace(returncode=0,stdout='',stderr='')
    monkeypatch.setattr(subprocess,'run',run)
    result=q.x2_runtime_fix(db,row,root,nq.RUN,out)
    assert result['status']=='L3_PASS'  # mocked unit runtime, never a real qualification receipt
    assert str(raw)+':'+str(raw)+':ro' in calls[0]
    assert 'stack-lite' in calls[0] and '--no-deps' in calls[0]
    assert calls[-1][-2:]==['down','--remove-orphans'] and '-v' not in calls[-1]


def test_orchestrator_splits_disjoint_d04_d07_commits_without_shared_branch_updates(candidate):
    db,root,*_=candidate
    (root/'d04.txt').write_text('D04 fixture');(root/'d07.txt').write_text('D07 fixture')
    before=git(root,'rev-parse','lab/stable')
    groups=nq.attempts.commit_components(root,'unit',1,[dict(blocker='d04',files=['d04.txt']),dict(blocker='d07',files=['d07.txt'])])
    assert groups['d04']['fix_sha']!=groups['d07']['fix_sha']
    assert git(root,'diff-tree','--no-commit-id','--name-only','-r',groups['d04']['fix_sha'])=='d04.txt'
    assert git(root,'diff-tree','--no-commit-id','--name-only','-r',groups['d07']['fix_sha'])=='d07.txt'
    assert git(root,'rev-parse','lab/stable')==before


def test_orchestrator_splits_shared_file_hunks_into_independent_commits(candidate):
    import difflib
    db,root,*_=candidate
    before='d04=0\nd07=0\n';middle='d04=1\nd07=0\n';after='d04=1\nd07=1\n'
    (root/'both.txt').write_text(before);git(root,'add','both.txt');git(root,'commit','-qm','unit shared baseline')
    (root/'both.txt').write_text(after)
    def patch(a,b):return ''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='a/both.txt',tofile='b/both.txt'))
    plans=[dict(blocker='d04',files=['both.txt'],patch=patch(before,middle)),dict(blocker='d07',files=['both.txt'],patch=patch(middle,after))]
    groups=nq.attempts.commit_components(root,'unit',1,plans)
    assert git(root,'show',groups['d04']['fix_sha']+':both.txt')==middle.strip()
    assert git(root,'show',groups['d07']['fix_sha']+':both.txt')==after.strip()
    assert (root/'both.txt').read_text()==after
