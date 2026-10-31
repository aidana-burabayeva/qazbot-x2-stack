import copy
import hashlib
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/nightly'))
import optimization_archive as opt
import store


def fixture(tmp_path):
    c = sqlite3.connect(':memory:', isolation_level=None)
    c.row_factory = sqlite3.Row
    store.init(c)
    f = tmp_path / 'evidence.txt'; f.write_text('measured fixture; not actual simulator qualification')
    d = dict(config_id='baseline', baseline_id='baseline', fidelity='L3', git_sha='a' * 40,
             map_sha256='b' * 64, world_sha256='c' * 64, stack_image_digest='sha256:' + 'd' * 64,
             simulation_image_digest='sha256:' + 'e' * 64, ros_version='humble', nav2_version='1.1.20', mc_version='fixture',
             scenario_set_sha256='f' * 64, seeds=[1, 2], scenarios=['cw', 'ccw'], fixed_parameters_sha256='1' * 64,
             safety_contract_sha256='2' * 64, controller_parameters={'desired_linear_vel': .3}, evidence=[{'path': 'evidence.txt', 'sha256': hashlib.sha256(f.read_bytes()).hexdigest()}],
             resource_profile=dict(cpu_peak_percent=100, ram_peak_bytes=1000, disk_delta_bytes=100, evidence_bytes=100, rtf_min=1, missed_mc_cycles=0, wall_duration_s=10, workers=1, timing_isolation_status='PASS'),
             qualification_status='MC_MUJOCO_QUALIFIED', independent_validation='PASS', result_key='fixture',
             hard_failure_flags={k: False for k in opt.FLAGS},
             metrics={'duration_s': 10, 'position_error_m': .1, 'yaw_error_rad': .1, 'min_clearance_m': .2, 'success_rate': 1})
    c.execute("INSERT INTO results(result_key,run_id,task,attempt,scenario_id,verdict,status,git_sha,map_sha) VALUES('fixture','r','t',1,'s','PASS','STORED',?,?)", (d['git_sha'],d['map_sha256']))
    return c,d


def test_hard_fail_never_enters_front_and_every_record_retained(tmp_path):
    db,d=fixture(tmp_path)
    baseline=opt.record(db,d)
    unsafe=copy.deepcopy(d); unsafe['config_id']='fast-contact';unsafe['hard_failure_flags']['collision']=True;unsafe['metrics']['duration_s']=1
    bad=opt.record(db,unsafe)
    assert opt.record(db,d)==baseline
    a=opt.archive(db,tmp_path)
    assert len(a['all_runs'])==2
    group=next(iter(a['cohorts'].values()))
    assert group['selected']['pareto']==[baseline] and group['selected']['fastest_among_safe']==baseline
    assert next(r for r in a['all_runs'] if r['run_id']==bad)['eligibility']=='REJECTED'
    assert a['automatic_optimization_enabled'] is False


def test_fidelity_versions_and_seed_sets_never_mix(tmp_path):
    db,d=fixture(tmp_path)
    opt.record(db,d)
    for key,value in [('fidelity','L1'),('mc_version','different'),('seeds',[3,4])]:
        item=copy.deepcopy(d); item[key]=value;opt.record(db,item)
    assert len(opt.archive(db,tmp_path)['cohorts'])==4


def test_tampered_evidence_revokes_eligibility_without_losing_original(tmp_path):
    db,d=fixture(tmp_path);opt.record(db,d)
    (tmp_path/'evidence.txt').write_text('changed')
    a=opt.archive(db,tmp_path)
    assert len(a['all_runs'])==1 and a['all_runs'][0]['eligibility']=='REJECTED'
    assert next(iter(a['cohorts'].values()))['selected']['pareto']==[]


def test_nondominated_tradeoffs_and_dominated_candidate(tmp_path):
    db,d=fixture(tmp_path);a=opt.record(db,d)
    faster=copy.deepcopy(d);faster['config_id']='faster';faster['metrics']['duration_s']=8;faster['metrics']['position_error_m']=.2
    b=opt.record(db,faster)
    worse=copy.deepcopy(d);worse['config_id']='worse';worse['metrics']['duration_s']=15;opt.record(db,worse)
    group=next(iter(opt.archive(db,tmp_path)['cohorts'].values()))
    assert set(group['selected']['pareto'])=={a,b}
    assert group['selected']['most_accurate']==a and group['selected']['fastest_among_safe']==b


def test_missing_gates_and_unverified_versions_not_promoted(tmp_path):
    db,d=fixture(tmp_path)
    d['hard_failure_flags'].pop('t5_failure');d['qualification_status']='UNVERIFIED';d['result_key']='missing'
    rid=opt.record(db,d);a=opt.archive(db,tmp_path)
    assert a['all_runs'][0]['run_id']==rid
    assert {'hard_failure_or_missing_gate','unqualified_fidelity','no_matching_stored_pass_result'} <= set(a['all_runs'][0]['reasons'])


def test_missing_resource_profile_and_additional_hard_failure_reject(tmp_path):
    db,d=fixture(tmp_path)
    d['resource_profile']['workers']=2
    d['hard_failure_flags']['unexpected_collision_gate_failure']=True
    opt.record(db,d)
    errors=opt.archive(db,tmp_path)['all_runs'][0]['reasons']
    assert 'hard_failure_or_missing_gate' in errors and 'l3_resources_unqualified' in errors
