"""Capabilities with real SQLite and queue dispatch policy; no actual provider/simulation."""
import json
import sqlite3
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/nightly'))
import execution
import nq
import workflow
import campaign


@pytest.fixture
def db():
    d=sqlite3.connect(':memory:',isolation_level=None);d.row_factory=sqlite3.Row
    d.executescript(nq.SCHEMA)
    for column in ('base_from TEXT','task_type TEXT','complexity TEXT','risk TEXT','requires_result INTEGER DEFAULT 0','owner TEXT','phase TEXT','klass TEXT'):
        d.execute('ALTER TABLE tasks ADD COLUMN '+column)
    return d


def authorize(db,root,mode):
    return execution.authorize(db,mode,dict(mode=mode,verdict='PASS',stamp='test'),root,'a'*40)


def test_engineering_without_l3_receipt_can_dispatch_only_offline(db,tmp_path,monkeypatch):
    authorize(db,tmp_path,execution.ENGINEERING)
    (tmp_path/'tools/nightly').mkdir(parents=True);(tmp_path/'tools/nightly/labctl.sh').touch()
    monkeypatch.setattr(nq,'ROOT',tmp_path)
    monkeypatch.setattr(nq.mergeq,'integration_head',lambda *a:'a'*40)
    nq.enqueue(db,'offline','true',kind='engineering',execution_tier=execution.ENGINEERING)
    nq.enqueue(db,'sim','true',resource='sim',execution_tier=execution.L3)
    nq.enqueue(db,'full','true',execution_tier=execution.FULL)
    assert [r['name'] for r in nq.ready(db)]==['offline']


def test_first_l3_needs_no_future_dynamic_receipts_but_is_single_use(db,tmp_path):
    authorize(db,tmp_path,execution.L3)
    assert execution.allowed(db,tmp_path,execution.L3,'a'*40)[0]
    execution.consume_l3(db)
    assert not execution.allowed(db,tmp_path,execution.L3,'a'*40)[0]
    assert execution.allowed(db,tmp_path,execution.ENGINEERING)[0]


def test_stale_sha_blocks_sim_without_stopping_engineering(db,tmp_path):
    authorize(db,tmp_path,execution.FULL)
    assert not execution.allowed(db,tmp_path,execution.L3,'b'*40)[0]
    assert execution.allowed(db,tmp_path,execution.ENGINEERING)[0]


def test_environment_mode_cannot_grant_authority(db,tmp_path,monkeypatch):
    monkeypatch.setenv('NIGHTLY_MODE','FULL_NIGHTLY')
    assert not execution.allowed(db,tmp_path,execution.L3,'a'*40)[0]


def test_authorize_refuses_failed_or_different_preflight(db,tmp_path):
    with pytest.raises(ValueError):execution.authorize(db,execution.ENGINEERING,dict(mode=execution.FULL,verdict='PASS'),tmp_path,'a')
    with pytest.raises(ValueError):execution.authorize(db,execution.ENGINEERING,dict(mode=execution.ENGINEERING,verdict='FAIL'),tmp_path,'a')


def test_repeated_engineering_prepare_and_failure_signatures_preserve_history(db,monkeypatch):
    monkeypatch.setattr(nq,'checkpoint',lambda *a:None)
    workflow.enqueue_engineering(db);workflow.enqueue_engineering(db)
    assert db.execute('SELECT COUNT(*) FROM tasks').fetchone()[0]==12
    assert all(r['execution_tier']==execution.ENGINEERING for r in db.execute('SELECT * FROM tasks'))
    event=dict(component='nav2',fidelity='L3',error='timeout after 10 seconds',git_sha='abc',experiment_id='one',seed=1)
    sig=workflow.ingest(db,event)
    workflow.ingest(db,event)
    workflow.ingest(db,dict(event,experiment_id='two',seed=2,error='timeout after 20 seconds'))
    assert db.execute('SELECT COUNT(*) FROM failure_events').fetchone()[0]==2
    assert db.execute('SELECT COUNT(*) FROM tasks WHERE name=?',('engineering_defect_'+sig,)).fetchone()[0]==1


def test_missing_hardware_fact_does_not_block_independent_offline_work(db,tmp_path,monkeypatch):
    authorize(db,tmp_path,execution.ENGINEERING)
    workflow.ingest(db,dict(component='mc',error='missing PC1 identity',needs_hardware=True,fidelity='AUDIT'))
    assert db.execute('SELECT state FROM tasks').fetchone()[0]=='NEEDS_HARDWARE'
    assert execution.allowed(db,tmp_path,execution.ENGINEERING)[0]


def test_timer_engineering_never_enqueues_historical_or_simulation(db,tmp_path,monkeypatch):
    authorize(db,tmp_path,execution.ENGINEERING)
    monkeypatch.setattr(nq,'ROOT',tmp_path);monkeypatch.setattr(nq,'checkpoint',lambda *a:None)
    workflow.daily(db);workflow.daily(db)
    rows=db.execute('SELECT * FROM tasks').fetchall()
    assert len(rows)==13
    assert all(r['execution_tier']==execution.ENGINEERING and r['resource']!='sim' for r in rows)
    assert all('first_map_nav2' not in r['cmd'] and 'run_l1.sh' not in r['cmd'] for r in rows)


def test_bad_campaign_fov_or_map_rejected(monkeypatch):
    source=Path(__file__).resolve().parents[2]/campaign.PATH
    d=json.loads(source.read_text())
    monkeypatch.setattr(campaign.subprocess,'check_output',lambda *a,**k:json.dumps(d).encode())
    _,sha=campaign.load('source')
    assert len(sha)==64
    d['scan_fov_deg']=130
    with pytest.raises(ValueError):campaign.load('source')
    d['scan_fov_deg']=103;d['map']='first_map_nav2_v2'
    with pytest.raises(ValueError):campaign.load('source')


def test_engineering_bwrap_hides_docker_and_processes(tmp_path,monkeypatch):
    monkeypatch.setattr(__import__('shutil'),'which',lambda *a:'/usr/bin/bwrap')
    argv=execution.engineering_sandbox(['codex','exec'],tmp_path,tmp_path/'result')
    assert '--unshare-pid' in argv and '--tmpfs' in argv
    assert '/run' in argv and argv[-2:]==['codex','exec']


def test_protected_safety_ast_detects_threshold_changes_but_ignores_comments():
    import safety_guard
    source="X2_FOOTPRINT = [1, 2]\nx = {'xy_goal_tolerance': 0.1, 'use_collision_detection': True}\nclass TrustConfig:\n    max_age = 0.2\n"
    assert safety_guard.protected(source)==safety_guard.protected('# comment\n'+source)
    for altered in (source.replace('0.1','0.3'),source.replace('True','False'),source.replace('[1, 2]','[1, 3]'),source.replace('0.2','0.4')):
        assert safety_guard.protected(source)!=safety_guard.protected(altered)


def test_sim_entrypoint_refuses_missing_queue_even_with_full_environment(tmp_path,monkeypatch):
    monkeypatch.setenv('NIGHTLY_MODE',execution.FULL)
    monkeypatch.setenv('QAZBOT_ROLE','LAB')
    with pytest.raises(SystemExit,match='execution refused'):
        execution.require(execution.L3,root=tmp_path,run=tmp_path/'missing')


def test_l3_claim_keeps_original_attempt_and_cannot_be_reclaimed(db,tmp_path):
    authorize(db,tmp_path,execution.L3)
    execution.consume_l3(db,'bounded',3)
    assert execution.get(db,'execution_authorization')['l3_claim']=={'task':'bounded','attempt':3}
    with pytest.raises(ValueError,match='budget exhausted'):
        execution.consume_l3(db,'bounded',4)


def test_qualification_always_attempts_teardown_after_log_timeout(db,tmp_path,monkeypatch):
    import subprocess
    import preflight
    calls=[]
    profile=dict(id='test',qualification_scope='stationary_official_mc',map='d1_qazbot_20261029',
                 max_wall_s=60,duration_s=1,seeds=[1],robot_sha='r',map_sha256='m')
    monkeypatch.setattr(campaign,'load',lambda sha:(profile,'digest'))
    monkeypatch.setattr(execution,'require',lambda tier:dict(name='bounded',attempts=1))
    monkeypatch.setattr(nq.mergeq,'integration_head',lambda *a:'source')
    monkeypatch.setattr(nq,'ROOT',tmp_path);monkeypatch.setattr(nq,'RUN',tmp_path)
    monkeypatch.setattr(nq,'connect',lambda:db)
    monkeypatch.setattr(preflight,'env_file',lambda *a:{'TWIN_START':'0,0,0'})
    monkeypatch.setattr(preflight,'vendor_identity',lambda:{})
    execution.put(db,'execution_authorization',dict(mode=execution.L3,vendor_identity={},images=dict(stack='image',sim='image')))
    monkeypatch.setenv('NIGHTLY_RESULT_DIR',str(tmp_path/'result'))
    monkeypatch.setattr(workflow,'analyze',lambda *a:None)
    def command(argv,**kw):
        calls.append(argv)
        if 'image' in argv:return subprocess.CompletedProcess(argv,0,'image','')
        if 'down' in argv:return subprocess.CompletedProcess(argv,0,'stopped','')
        raise subprocess.TimeoutExpired(argv,1)
    monkeypatch.setattr(subprocess,'run',command)
    assert campaign.run('source','digest',execution.L3)==1
    assert any('down' in argv for argv in calls)
    saved=json.loads((tmp_path/'result/qualification.json').read_text())
    assert saved['qualification_status']=='UNVERIFIED'
    assert saved['rtf'] is None and saved['raw_contacts'] is None
    assert not (tmp_path/'result/worker_authorization.json').exists()
