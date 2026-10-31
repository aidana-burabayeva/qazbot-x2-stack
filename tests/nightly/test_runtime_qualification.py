"""Synthetic receipt validation only; never claims actual MC/DDS qualification."""
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/nightly'))
import optimization_archive as archive
import qualification
from test_optimization_archive import fixture


def setup_receipt(tmp_path):
    db,d=fixture(tmp_path)
    db.execute('CREATE TABLE tasks(name TEXT PRIMARY KEY,owner TEXT,phase TEXT,state TEXT)')
    for provider in ('claude','codex'):
        db.execute("INSERT INTO tasks VALUES(?,?,'VERIFIED','COMPLETED')",(provider,provider))
        db.execute("INSERT INTO results(result_key,run_id,task,attempt,scenario_id,verdict,status,git_sha) VALUES(?,?,?,1,'s','PASS','STORED',?)",(provider,provider,provider,d['git_sha']))
    d['robot_sha']='f'*40
    d['runtime_gates']={k:True for k in qualification.GATES}
    d['cli_results']={k:k for k in ('claude','codex')}
    path=tmp_path/'receipt.json'
    return db,d,path


def check(db,d,path,tmp_path):
    rid=archive.record(db,d);path.write_text(json.dumps({'archive_run_id':rid}))
    return qualification.check(db,path,tmp_path,'a'*40,'f'*40,'b'*64,'sha256:'+'d'*64,'sha256:'+'e'*64)


def test_complete_synthetic_receipt_and_real_provider_owner_binding(tmp_path):
    db,d,path=setup_receipt(tmp_path)
    assert check(db,d,path,tmp_path)==[]
    db.execute("UPDATE tasks SET owner='claude' WHERE name='codex'")
    assert 'missing_verified_cli_task:codex' in check(db,d,path,tmp_path)


def test_stale_images_sources_and_lite_cannot_authorize(tmp_path):
    db,d,path=setup_receipt(tmp_path)
    d['fidelity']='L1';d['git_sha']='0'*40;d['simulation_image_digest']='sha256:'+'0'*64
    errors=check(db,d,path,tmp_path)
    assert {'official_l3_required','stale_or_unknown:git_sha','stale_or_unknown:simulation_image_digest'} <= set(errors)


def test_missing_runtime_gate_or_damaged_evidence_blocks(tmp_path):
    db,d,path=setup_receipt(tmp_path)
    d['runtime_gates'].pop('stop_resume')
    (tmp_path/'evidence.txt').write_text('tampered')
    errors=check(db,d,path,tmp_path)
    assert 'gate:stop_resume' in errors and 'invalid_evidence' in errors


def test_missed_mc_cycles_or_old_cli_result_blocks(tmp_path):
    db,d,path=setup_receipt(tmp_path)
    d['resource_profile']['missed_mc_cycles']=1
    db.execute("UPDATE results SET git_sha='old' WHERE result_key='claude'")
    errors=check(db,d,path,tmp_path)
    assert 'l3_resources_unqualified' in errors and 'missing_verified_cli_task:claude' in errors
