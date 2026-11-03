#!/usr/bin/env python3
"""Finite audit/feedback jobs in the existing nq queue. No worker loop or new orchestrator."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time
import urllib.request

import nq
import execution

AREAS=(('amcl_initialpose','Replay the saved AMCL initialpose episodes; reproduce and diagnose offline before any live SIM experiment. Never use truth as AMCL evidence.'),
       ('sensor_contract','Analyze saved hardware cloud fixture, PointCloud2 marking/clearing and missing-scan behavior. Source-level/unit integrations only; live Nav2 requires L3 capability.'),
       ('p23_motion','Compare saved physical P20/P23 traces with saved official-MC results; separate firmware and gait differences. Do not run MC or tune around a systemic defect.'),
       ('frame_anchor','Reproduce frame-anchor/reset problems with existing offline fixtures. Preserve single TF ownership and reset/freshness gates.'),
       ('gpu_egl','Read vendor ELF/configuration and saved GPU evidence. Do not assume EGL from NVIDIA presence; design a bounded measured A/B for later L3.'),
       ('odometry_reset','Reproduce saved odometry reset and T1/freshness cases offline; preserve actual thresholds. Missing physical facts become NEEDS_HARDWARE.'))


def enqueue_engineering(db):
    if execution.restricted(db):
        return
    for index,(area,prompt) in enumerate(AREAS):
        name='engineering_'+area+'_v3'
        if nq.enqueue(db,name,'true',kind='engineering',execution_tier=execution.ENGINEERING,resource='engineering',priority=5+index,
                      repro='--agents=claude,codex '+prompt+' Work in the isolated attempt only. Reproduce FAIL, test hypotheses, prove root cause, minimal fix, repeat FAIL and regressions; never claim SIM_VERIFIED from offline tests.'):
            db.execute('UPDATE tasks SET task_type=?,complexity=?,risk=?,requires_result=1 WHERE name=?',('root_cause_analysis','high','high',name))
        db.execute('UPDATE tasks SET review_required=1 WHERE name=?',(name,))
        review=name+'_review'
        if nq.enqueue(db,review,'true',kind='engineering',execution_tier=execution.ENGINEERING,resource='engineering',priority=20+index,deps=[name],
                      repro='--agents=codex,claude Independently review '+name+' using stored evidence and its verified SHA. Rerun tests and reject unsupported simulation/hardware claims. Offline only.'):
            db.execute('UPDATE tasks SET base_from=?,task_type=?,requires_result=1 WHERE name=?',(name,'root_cause_analysis',review))



TARGETS = (
    (execution.X2_TASKS[0], 0, 'P0 Preservation & Restore',
     'Source /home/adn/QazBot/x2_preservation_20261103/: use existing Git/SHA256 manifests, not a new robot inventory. '
     'Confirm restoration of current production sources; restore the Git bundle in a separate scratch directory; '
     'verify tar readability, permissions and symlinks; external dependencies; restored D1 map readability; '
     'backend imports; isolated C++ projector build when dependencies exist. '
     'Verify independent copies of all three D04 bags and their SHA256; identify dependencies missing from Git/backup. '
     'Do not bulk-copy large archives before checking resources. Do not weaken sandbox if private archives are inaccessible: '
     'mark the exact part BLOCKED and prepare a minimal authorized read-only host verification procedure. '
     'Result must be RESTORE_VERIFIED or evidence-backed RESTORE_BLOCKED; missing bags must not prevent independent D05 safety work.'),
    (execution.X2_TASKS[1], 1, 'P0 D-05 / Collision Monitor',
     'Use tests/collision_monitor/, tests/d05/cancel_latency/, R1/R2/R5/correction reports and existing T1, Goal Coordinator and Motion Adapter. '
     'Confirmed issues: fail-open on lost cloud and lost TF; unknown actual stopping time; commanded/actual speed mismatch; unproven C1/C2. '
     'First assess stock Nav2 Humble and existing QazBot mechanisms; measure actual installed Nav2 version and compatibility with historical 1.1.18. '
     'Implement sensor/TF freshness contract, held safe STOP, sensor-recovery behavior, and cancel/stop contract through existing safety gates. '
     'Separate MEASURED, ESTIMATED and UNKNOWN stopping-distance terms. Reproduce original FAIL then before/after regressions. '
     'Do not create another Motion Adapter or parallel MC command path, weaken T1/T5, footprint/collision thresholds, or claim hardware safety PASS without physical braking measurements. '
     'Missing bags/SDK block only dependent checks; continue independently supported safety tests. No SDK fakes. Separate safety/perception/localization/MC commits.'),
    (execution.X2_TASKS[2], 20, 'P1 D-04 C++ / D-07',
     'D04: use imported qazbot_lidar_projector; establish the cause of three missed C++ publications using timestamps, QoS, DDS fragmentation, TF and odometry pairing. '
     'Retain parity with Python candidate C and single TF ownership; minimally fix only a proven defect. '
     'Do not invent a cause if original recording is absent; mark that causal question BLOCKED. '
     'D07: evaluate stock Nav2 ObstacleLayer/VoxelLayer; fix exclusion of low obstacles while preserving floor filtering, marking/clearing and SLAM LaserScan. '
     'Test synthetic low obstacles, floor tilt and sensor dropout. Bag d04_static_bag_183015 contains no claimed boxes; synthetic PASS is not hardware PASS. '
     'D04 and D07 must be independent commits. No rejected odom_hold or unreviewed planar-refactor import, deployment, or safety-threshold relaxation.'),
)

def enqueue_targeted(db):
    if nq.kv_get(db, 'campaign_scope') != execution.X2_CAMPAIGN:
        raise ValueError('explicit x2-blockers campaign scope required')
    for name, priority, title, prompt in TARGETS:
        if nq.enqueue(db, name, 'true', kind='engineering', execution_tier=execution.ENGINEERING,
                      resource='engineering', priority=priority, max_attempts=2,
                      timeout_s=3600, heartbeat_s=900,
                      repro='--agents=codex '+title+': '+prompt+
                      ' Work alone in the existing isolated attempt. No subagents, parallel research group, hardware access, movement, installs, ROS goals or full replay/Nightly. '
                      'Use the existing model router with Codex only. Claude is held for a later independently confirmed review phase; never infer availability from 06:00. '
                      'Record OFFLINE_VERIFIED only; return the existing result contract with exact tests/evidence and partial BLOCKED findings. No SIM_VERIFIED or hardware claims.'):
            db.execute('UPDATE tasks SET task_type=?,complexity=?,risk=?,requires_result=1 WHERE name=?',
                       ('root_cause_analysis', 'high', 'high', name))
    return list(execution.X2_TASKS)


def target_plan(db):
    """Pure planning on a caller-owned SQLite memory snapshot; never probes or dispatches."""
    nq.kv_set(db, 'campaign_scope', execution.X2_CAMPAIGN)
    enqueue_targeted(db)
    tasks = [dict(db.execute('SELECT name,state,priority,resource,execution_tier FROM tasks WHERE name=?', (n,)).fetchone())
             for n in execution.X2_TASKS]
    excluded = [r['name'] for r in db.execute('SELECT name FROM tasks ORDER BY id') if not execution.scope_allows(db, r['name'])]
    return dict(campaign=execution.X2_CAMPAIGN, mode=execution.ENGINEERING, dry_run=True,
                agents_started=0, max_parallel=1, tasks=tasks, excluded_tasks=excluded,
                next_task=execution.next_target(db), active_leases=execution.active_leases(db),
                start_blocked_by_leases=not execution.scope_idle(db),
                authorization='requires fresh real ENGINEERING preflight at operator start',
                automatic_general_enqueue=False)


def init(db):
    db.execute('CREATE TABLE IF NOT EXISTS failure_events(event_id TEXT PRIMARY KEY,signature TEXT,payload TEXT,ts REAL)')


def ingest(db, event):
    init(db)
    encoded=json.dumps(event,sort_keys=True,allow_nan=False)
    eid=hashlib.sha256(encoded.encode()).hexdigest()
    error=str(event.get('error') or event.get('outcome') or 'incomplete evidence')
    norm=re.sub(r'0x[0-9a-f]+|\b\d+(?:\.\d+)?\b','N',error.lower())
    norm=re.sub(r'/[^\s,;]+','PATH',norm)
    kind=str(event.get('component','unknown'))
    sig=hashlib.sha256((str(event.get('fidelity','UNKNOWN'))+'|'+kind+'|'+norm).encode()).hexdigest()[:16]
    if db.execute('SELECT 1 FROM failure_events WHERE event_id=?',(eid,)).fetchone():
        return sig
    db.execute('INSERT INTO failure_events VALUES(?,?,?,?)',(eid,sig,encoded,time.time()))
    if execution.restricted(db):
        nq.ev(db, '*', 'SCOPED_FAILURE_EVIDENCE', eid)
        return sig  # preserve evidence without creating unrelated tasks/reviews
    hardware=event.get('needs_hardware') is True
    name=('hardware_fact_' if hardware else 'engineering_defect_')+sig
    previous=db.execute('SELECT state FROM tasks WHERE name=?',(name,)).fetchone()
    if previous and previous['state']=='COMPLETED' and event.get('git_sha'):
        name+='_recur_'+str(event['git_sha'])[:12]
    text=('NEEDS_HARDWARE: no physical robot access; collect no new hardware data here. ' if hardware else 'Engineering reproduction only. ')
    text+=('Failure cluster '+sig+'; event '+eid+'. '+error+'. Source experiment metadata is preserved in failure_events. '
           'Reproduce; record testable hypotheses; prove a cause before minimal fix and A/B; independent review and Merge Queue. '
           'No repeated attempt without a new hypothesis or new data. No simulation launch in Engineering; request bounded L3 capability separately.')
    nq.enqueue(db,name,'true',kind='engineering',execution_tier=execution.ENGINEERING,resource='engineering',priority=2,
               state='NEEDS_HARDWARE' if hardware else 'PENDING',repro='--agents=claude,codex '+text,reason='TWIN_FIDELITY_REQUEST '+sig)
    db.execute('UPDATE tasks SET klass=?,task_type=?,requires_result=1 WHERE name=?',('TWIN_FIDELITY_REQUEST','root_cause_analysis',name))
    db.execute('UPDATE tasks SET review_required=1 WHERE name=?',(name,))
    if not hardware:
        review=name+'_review'
        if nq.enqueue(db,review,'true',kind='engineering',execution_tier=execution.ENGINEERING,resource='engineering',priority=15,deps=[name],
                      repro='--agents=codex,claude Independently review '+name+' from stored evidence; rerun reproduction/regression. Offline only.'):
            db.execute('UPDATE tasks SET base_from=?,requires_result=1 WHERE name=?',(name,review))
    nq.ev(db,name,'FAILURE_EVIDENCE',eid)
    return sig


def audit(db, refresh=False):
    claims=json.loads((nq.ROOT/'tools/nightly/fidelity/claims.json').read_text())
    refs={ref:nq.git('rev-parse',ref) for ref in ('lab/stable','origin/robot/stable')}
    out={'stamp':time.time(),'claims':claims,'git':refs,'qualification':'ENGINEERING_AUDIT_NOT_SIM_VERIFIED','fetches':[]}
    if refresh:
        class VendorRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,req,fp,code,msg,headers,newurl):
                if not newurl.startswith('https://x2-aimdk.agibot.com/'):
                    raise ValueError('non-vendor redirect forbidden')
                return super().redirect_request(req,fp,code,msg,headers,newurl)
        opener=urllib.request.build_opener(VendorRedirect())
        for claim in claims['claims']:
            item={'claim_id':claim['id'],'url':claim['url'],'retrieved_at':time.time()}
            try:
                # Only public vendor HTTPS; never an X2 address or arbitrary URLs from experiment output.
                if not claim['url'].startswith('https://x2-aimdk.agibot.com/'):
                    raise ValueError('non-vendor URL forbidden')
                req=urllib.request.Request(claim['url'],headers={'User-Agent':'QazBot-LAB-fidelity-audit'})
                with opener.open(req,timeout=10) as response:
                    if not response.url.startswith('https://x2-aimdk.agibot.com/'):
                        raise ValueError('unexpected documentation redirect')
                    raw=response.read(2*1024*1024)
                    version=re.search(r'AimDK_X2\s+([0-9.]+)',raw.decode('utf-8',errors='replace'))
                    item.update(status='FETCHED_NOT_SEMANTICALLY_REVIEWED',sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw),detected_document_version=version.group(1) if version else None)
            except Exception as exc:
                item.update(status='UNVERIFIED',error=type(exc).__name__+': '+str(exc)[:160])
            out['fetches'].append(item)
    sources=('digital_twin/ws/src/x2_twin_bringup/config/twin.yaml','tools/nightly/fixtures/x2_recorded_lidar.json',
             'docs/evidence/robot_20261030/versions.txt','digital_twin/vendor/manifest.sha256')
    out['source_hashes']={p:hashlib.sha256((nq.ROOT/p).read_bytes()).hexdigest() for p in sources if (nq.ROOT/p).is_file()}
    d=nq.RUN/'fidelity';d.mkdir(parents=True,exist_ok=True)
    key=hashlib.sha256(json.dumps(out,sort_keys=True).encode()).hexdigest()
    (d/(key+'.json')).write_text(json.dumps(out,indent=2)+'\n')
    # These are unresolved factual mismatches, not proof that 103 degrees should be set to 120.
    for component,error,hw in (('lidar_model','Planar model density/height/timestamps differ from recorded processed E1R clouds',False),
                               ('factory_calibration','Published unit-specific factory extrinsics insufficient for claimed physical sensor alignment',True),
                               ('mc_identity','Current physical PC1 MC binary identity not established by PC2 platform version',True)):
        ingest(db,dict(component=component,error=error,needs_hardware=hw,fidelity='AUDIT',experiment_id=key,
                       git_sha=refs['lab/stable'],map='d1_qazbot_20261029',seed=None,evidence=str(d/(key+'.json')),hypothesis_status='HYPOTHESIS'))
    # Consume published hardware feedback; git-only, no deployment or robot connection.
    import lab
    out['robot_sync']=lab.robot_sync(db)
    (d/(key+'.json')).write_text(json.dumps(out,indent=2)+'\n')
    return out


def daily(db):
    if execution.restricted(db):
        return []  # targeted scope has no daily/general task expansion
    if nq.kv_get(db,'frozen',False):
        return []
    ok,why=execution.allowed(db,nq.ROOT,execution.ENGINEERING)
    if not ok:
        raise SystemExit('timer refused: '+why)
    stamp=time.strftime('%Y%m%d');name='fidelity_audit_'+stamp
    nq.enqueue(db,name,'python3 tools/nightly/workflow.py audit --refresh',execution_tier=execution.ENGINEERING,
               priority=1,resource='engineering',timeout_s=300,heartbeat_s=240,requires_result=0)
    enqueue_engineering(db)
    mode=nq.kv_get(db,'execution_mode',execution.ENGINEERING)
    if mode==execution.FULL:
        import campaign
        campaign.enqueue_current(db,qualification_only=False)
    nq.checkpoint(db,'mode-aware daily enqueue')
    return [name]


def analyze(db,report):
    """Only present measurements are evaluated; missing values never become a zero/PASS."""
    problems=[]
    outcome=report.get('nav2_action_outcome')
    if outcome and outcome not in ('SUCCEEDED','SAFE_REJECT','NOT_RUN_STATIONARY_SCOPE'):
        problems.append(('nav2_action',str(outcome)))
    for field in ('tf_anomalies','costmap_failures','sensor_timestamp_errors','scan_coverage_failures','localization_jumps',
                  'odometry_resets_unexplained','p23_motion_mismatch','mc_timing_failures','collision_contacts',
                  'path_invalidations','replanning_oscillations','resource_degradation'):
        value=report.get(field)
        if value:
            problems.append((field,json.dumps(value,sort_keys=True)))
    if report.get('error'):
        problems.append((report.get('component','campaign'),str(report['error'])))
    if report.get('fidelity')=='L3':
        missing=[k for k in ('rtf','missed_mc_cycles','raw_contacts') if report.get(k) is None]
        if missing:problems.append(('qualification_instrumentation','missing measured '+','.join(missing)))
    return [ingest(db,dict(report,component=kind,error=error,hypothesis_status='HYPOTHESIS')) for kind,error in problems]


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=('audit','enqueue','feedback','capability-probe'));p.add_argument('--refresh',action='store_true');p.add_argument('--file',type=Path);p.add_argument('--provider',choices=('claude','codex'));p.add_argument('--model')
    a=p.parse_args();db=nq.connect()
    if a.command=='capability-probe':
        import lead
        if not a.provider:p.error('--provider required')
        report=lead.probe(nq,a.provider,a.model);print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
    if a.command=='enqueue': print(json.dumps(daily(db)))
    elif a.command=='feedback': print(analyze(db,json.loads(a.file.read_text())))
    else: print(json.dumps(audit(db,a.refresh),indent=2))


if __name__=='__main__': main()
