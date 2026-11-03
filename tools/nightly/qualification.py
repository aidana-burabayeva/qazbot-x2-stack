"""Fail-closed authorization of unattended LAB execution from stored, version-matched E2E evidence."""
import json
import sqlite3
from pathlib import Path


GATES = ('aimdk_alignment','sensor_parity','nav2_marking_clearing','official_mc_mujoco',
         'frame_anchor','raw_collision_contacts','short_d1_route','resource_profile',
         'stop_resume','claude_cli_task','codex_cli_task','runtime_e2e')
# These scopes are not prerequisites for the first bounded stationary run or baseline E2E.
SCOPED_GATES = {'dynamic_office':('dynamic_obstacle','blocked_route','missing_scan'),
                'reset_freshness':('odometry_reset','localization_failure','sensor_delays_dropouts'),
                'stress':('cpu_contention','collision','recovery')}


def check(db, receipt_path, root, integration_sha, robot_sha, map_sha, stack_digest, simulation_digest):
    path = Path(receipt_path)
    if not path.is_file() or path.is_symlink():
        return ['missing_runtime_e2e_receipt']
    try:
        receipt = json.loads(path.read_text())
        rid = receipt['archive_run_id']
        row = db.execute('SELECT payload FROM optimization_runs WHERE run_id=?', (rid,)).fetchone()
        if row is None:
            return ['qualification_archive_record_missing']
        record = json.loads(row['payload'])
        if not isinstance(record, dict):
            return ['invalid_qualification_record']
        import optimization_archive
        errors = optimization_archive.failures(db, record, Path(root))
        if record.get('fidelity') != 'L3':
            errors.append('official_l3_required')
        expected = {'git_sha':integration_sha, 'robot_sha':robot_sha, 'map_sha256':map_sha,
                    'stack_image_digest':stack_digest, 'simulation_image_digest':simulation_digest}
        errors.extend('stale_or_unknown:' + k for k,v in expected.items() if not v or record.get(k)!=v)
        gates = record.get('runtime_gates', {})
        errors.extend('gate:' + k for k in GATES if not isinstance(gates,dict) or gates.get(k) is not True)
        evidence = record.get('cli_results', {})
        for provider in ('claude','codex'):
            key = evidence.get(provider) if isinstance(evidence,dict) else None
            stored = db.execute("SELECT t.owner,t.phase,t.state,r.verdict,r.status,r.git_sha FROM results r JOIN tasks t ON t.name=r.task WHERE r.result_key=?", (key,)).fetchone()
            if (not stored or stored['owner']!=provider or stored['phase']!='VERIFIED' or stored['state']!='COMPLETED' or
                    stored['verdict']!='PASS' or stored['status']!='STORED' or stored['git_sha']!=integration_sha):
                errors.append('missing_verified_cli_task:' + provider)
        return sorted(set(errors))
    except (ValueError, KeyError, TypeError, OSError, sqlite3.Error) as exc:
        return ['invalid_runtime_e2e_receipt:' + type(exc).__name__]


def first_l3_errors(report):
    """Prerequisites for one bounded run, not a receipt for all future navigation scenarios."""
    required=('sim.official_image_id','tests.frame_anchor','baseline.integration_sha256','contract.robot_env_vs_snapshot','contract.lab_env','map.host_sha256',
              'sim.stack_lite_healthy','sim.image_patch_is_documented','tf.owners','sensor.costmap_contract',
              'isolation.network_internal','isolation.no_host_network','isolation.no_default_route')
    checks={r['name']:r['status'] for r in report.get('checks',[])}
    return ['preflight:'+k for k in required if checks.get(k)!='PASS']

# Per-fix continuation of the existing X2 queue; no worker loop lives here.
import hashlib
import os
import shlex
import subprocess
import time
import xml.etree.ElementTree as ET

X2_BLOCKERS = {'d05': 'x2_p0_d05_collision_monitor', 'd04': 'x2_p1_d04_cpp_d07', 'd07': 'x2_p1_d04_cpp_d07'}
X2_CASES = {
    'd05': {'normal_navigation','obstacle','missing_cloud','stale_cloud','tf_failure','stop','held_stop','sensor_recovery','cancel','untrusted_no_motion'},
    'd04': {'real_cloud','tf_odom_pairing','qos','publication_loss','python_parity','marking','clearing','data_recovery'},
    'd07': {'low_obstacle','floor_separation','marking','clearing','false_obstacles','missing_cloud','sensor_recovery'},
}
X2_ACTIVE = ('FIX_CHECKING','L3_PREFLIGHT','L3_READY','L3_RUNNING')
X2_JOBS = {}


def x2_init(db):
    db.execute('''CREATE TABLE IF NOT EXISTS x2_l3(
      key TEXT PRIMARY KEY, blocker TEXT NOT NULL, producer TEXT NOT NULL, fix_sha TEXT NOT NULL,
      source_sha TEXT NOT NULL, conditions TEXT NOT NULL, spec TEXT NOT NULL, state TEXT NOT NULL,
      detail TEXT, task TEXT UNIQUE, pid INTEGER, evidence TEXT, updated REAL)''')
    db.execute("CREATE UNIQUE INDEX IF NOT EXISTS x2_l3_single_worker ON x2_l3((1)) WHERE state IN ('FIX_CHECKING','L3_PREFLIGHT','L3_READY','L3_RUNNING')")


def x2_exists(db):
    return bool(db.execute("SELECT 1 FROM sqlite_master WHERE name='x2_l3'").fetchone())


def x2_busy(db):
    return x2_exists(db) and bool(db.execute("SELECT 1 FROM x2_l3 WHERE state IN ('FIX_CHECKING','L3_PREFLIGHT','L3_READY','L3_RUNNING')").fetchone())


def x2_task(db, name):
    return dict(row) if x2_exists(db) and (row := db.execute('SELECT * FROM x2_l3 WHERE task=?', (name,)).fetchone()) else None


def _x2_set(db, key, state, detail='', **extra):
    import nq
    values=dict(state=state,detail=detail,updated=time.time(),**extra)
    db.execute('UPDATE x2_l3 SET '+','.join(k+'=?' for k in values)+' WHERE key=?',(*values.values(),key))
    nq.ev(db,key,state,detail)


def _git(root,*args):
    return subprocess.run(['git','-C',str(root),*args],capture_output=True,timeout=60)


def _ancestor(root, a, b):
    return _git(root,'merge-base','--is-ancestor',a,b).returncode == 0


def _safe_file(path):
    p=Path(path)
    return not p.is_absolute() and '..' not in p.parts and str(p).startswith(('tests/','ws/src/'))


def _pytest(paths):
    if not paths or not all(_safe_file(p) and p.endswith('.py') for p in paths):
        raise ValueError('explicit repo-relative pytest files required')
    return ['python3','-m','pytest','-q','-p','no:cacheprovider',*paths]


def x2_conditions(root):
    """No timestamp/usage samples: identical source and physical test inputs deduplicate."""
    def digest(path):
        p=Path(root)/path
        return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
    images={}
    for kind in ('stack','sim'):
        r=subprocess.run(['docker','image','inspect',f'x2-twin-{kind}:nightly','--format','{{.Id}}'],capture_output=True,text=True,timeout=20)
        images[kind]=r.stdout.strip() if not r.returncode else None
    import shutil
    runtime=subprocess.run(['docker','ps','--format','{{.Image}} {{.Names}}'],capture_output=True,text=True,timeout=20)
    conflicts=sorted(line for line in runtime.stdout.splitlines() if 'x2-twin-' in line) if not runtime.returncode else ['UNKNOWN']
    return dict(images=images,disk_floor_met=shutil.disk_usage(root).free>=5*2**30,runtime_conflicts=conflicts,mc_receipt=digest('run/nightly/nightly_qualification.json'),map=digest('ws/src/maps/d1_qazbot_20261029/map.pgm'),
                map_yaml=digest('ws/src/maps/d1_qazbot_20261029/map.yaml'),
                parameters=digest('digital_twin/.env.lab'),contract=digest('digital_twin/scripts/contract/robot.env'),
                nav2=digest('/opt/ros/humble/share/nav2_controller/package.xml'))


def x2_case_conditions(root,spec):
    conditions=x2_conditions(root);inputs={}
    for item in spec.get('inputs',[]) if isinstance(spec.get('inputs',[]),list) else []:
        try:
            p=Path(item['path']);p=p if p.is_absolute() else Path(root)/p
            st=p.stat();inputs[str(p)]=[st.st_size,st.st_mtime_ns,st.st_ino,item['sha256']]
        except (KeyError,OSError):inputs[str(item)]=None
    conditions['inputs']=inputs
    return conditions


def x2_admission_errors(db, root):
    import execution,shutil
    errors=[]
    if execution.active_leases(db): errors.append('active engineering/runtime task lease')
    if execution.get(db,'delegated_open',[]): errors.append('unconfirmed delegation lease')
    if db.execute("SELECT 1 FROM merge_queue WHERE state IN ('QUEUED','GATING')").fetchone(): errors.append('merge queue busy')
    if shutil.disk_usage(root).free < 5*2**30: errors.append('disk below existing 5 GiB floor')
    r=subprocess.run(['docker','ps','--format','{{.Image}}'],capture_output=True,text=True,timeout=20)
    if r.returncode: errors.append('cannot inspect conflicting runtime containers')
    elif any('x2-twin-' in line for line in r.stdout.splitlines()): errors.append('conflicting existing twin container; retained')
    for p in Path('/proc').glob('[0-9]*/exe'):
        try:
            if p.readlink().name in ('aima-mc-app-main','aima-sim-app'): errors.append('conflicting host MC/simulator process; retained')
        except OSError: pass
    return sorted(set(errors))


def x2_discover(db, root, run):
    """Only stored, verified ENGINEERING results and a real CAS integration admit a plan."""
    import nq,mergeq
    x2_init(db)
    source=mergeq.integration_head(root)
    for blocker,producer in X2_BLOCKERS.items():
        task=db.execute("SELECT * FROM tasks WHERE name=? AND state='COMPLETED' AND phase='VERIFIED'",(producer,)).fetchone()
        if not task: continue
        verified=nq.kv_get(db,'verified_sha:'+producer)
        result=db.execute("SELECT * FROM results WHERE task=? AND verdict='PASS' AND status='STORED' ORDER BY id DESC LIMIT 1",(producer,)).fetchone()
        merge=db.execute("SELECT * FROM merge_queue WHERE task=? AND sha=? AND state='INTEGRATED' ORDER BY id DESC LIMIT 1",(producer,verified)).fetchone()
        if not verified or not result or not merge or not _ancestor(root,merge['merge_sha'],source): continue
        data_path=Path(result['evidence_path'])/'engineering_result.json'
        if not data_path.is_file():
            data_path=Path(run)/'work'/f"{producer}.{task['attempts']}"/'engineering_result.json'
        try: data=json.loads(data_path.read_text())
        except (ValueError,OSError): data={}
        plans=[p for p in data.get('l3_fixes',[]) if isinstance(p,dict) and p.get('blocker')==blocker]
        if not plans: plans=[dict(blocker=blocker,fix_sha=verified,blocked='missing per-fix l3_fixes recipe; completion alone is insufficient')]
        for spec in plans:
            fix=spec.get('fix_sha') or verified
            conditions=x2_case_conditions(root,spec)
            key=hashlib.sha256(json.dumps([blocker,fix,source,conditions,spec],sort_keys=True).encode()).hexdigest()
            if db.execute('SELECT 1 FROM x2_l3 WHERE key=? OR (blocker=? AND fix_sha=? AND source_sha=? AND conditions=? AND spec=?)',
                          (key,blocker,fix,source,json.dumps(conditions,sort_keys=True),json.dumps(spec,sort_keys=True))).fetchone():continue
            errors=[]
            if spec.get('blocked'): errors.append(str(spec['blocked']))
            if blocker in ('d04','d07') and not spec.get('blocked') and any(not p.get('blocked') and p.get('blocker') in ('d04','d07') and p.get('blocker')!=blocker and p.get('fix_sha')==fix for p in data.get('l3_fixes',[]) if isinstance(p,dict)):
                errors.append('D04 and D07 require independent fix commits')
            if not _ancestor(root,fix,verified) or not _ancestor(root,fix,source): errors.append('fix SHA is not inside verified integrated result')
            causes=db.execute("SELECT key FROM root_causes WHERE task=? AND status='VERIFIED' AND evidence IS NOT NULL",(producer,)).fetchall()
            if not spec.get('root_cause') or spec['root_cause'] not in [r['key'] for r in causes]: errors.append('no matching VERIFIED root cause')
            diagnosis=data.get('diagnosis',{})
            if not all(diagnosis.get(k) for k in ('hypotheses','reproduction','causal_evidence','original_fail_retest','regression')): errors.append('missing accepted source-fix diagnosis')
            try:
                _pytest(spec.get('tests',[]));_pytest(spec.get('runtime_tests',[]))
                if any(_git(root,'cat-file','-e',source+':'+path).returncode for path in spec['tests']+spec['runtime_tests']):errors.append('tests missing from pinned integrated source')
                if not _ancestor(root,spec['baseline_sha'],fix) or spec['baseline_sha']==fix: errors.append('invalid original FAIL baseline')
            except (ValueError,KeyError,TypeError): errors.append('missing/invalid baseline or bounded test recipe')
            if spec.get('safety_invariants') is not True: errors.append('safety invariants not established')
            if spec.get('baseline_sha'):
                import safety_guard
                errors.extend('safety:'+e for e in safety_guard.check(str(root),spec['baseline_sha'],source))
            inputs=spec.get('inputs',[])
            if not isinstance(inputs,list): errors.append('invalid input manifest')
            else:
                for item in inputs:
                    try:
                        path=Path(item['path']);path=path if path.is_absolute() else Path(root)/path
                        if not path.is_file() or len(item['sha256'])!=64:errors.append('unavailable/invalid input:'+str(path))
                    except (KeyError,OSError,TypeError):errors.append('invalid input manifest entry')
            if not spec.get('dependencies_available'): errors.append('required runtime dependencies not established')
            state='L3_BLOCKED' if errors else 'ENGINEERING_VERIFIED'
            db.execute('INSERT INTO x2_l3(key,blocker,producer,fix_sha,source_sha,conditions,spec,state,detail,updated) VALUES(?,?,?,?,?,?,?,?,?,?)',
                       (key,blocker,producer,fix,source,json.dumps(conditions,sort_keys=True),json.dumps(spec,sort_keys=True),state,'; '.join(errors),time.time()))


def _x2_poll(db, row, out, phase):
    """Durable child outcome, never launch again after uncertain recovery."""
    key=row['key'];job=X2_JOBS.get(key)
    if job:
        result=job.poll()
        if result is None:return None
        X2_JOBS.pop(key,None)
    elif row['pid']:
        try:
            cmd=Path(f"/proc/{row['pid']}/cmdline").read_bytes()
            if key.encode() in cmd:return None
        except OSError:pass
    path=out/(phase+'.json')
    if not path.is_file():
        _x2_set(db,key,'L3_BLOCKED',f'{phase} process ended/restarted without a durable outcome; no automatic retry')
        return None
    try:return json.loads(path.read_text())
    except (OSError,ValueError):
        _x2_set(db,key,'L3_BLOCKED','invalid durable '+phase+' outcome');return None


def x2_advance(db, root, run):
    """One finite transition, called only by the existing nq tick under its lease."""
    import nq,execution,mergeq,attempts
    if execution.get(db,'campaign_scope')!=execution.X2_CAMPAIGN or execution.get(db,'frozen',False):return
    x2_init(db)
    x2_recover(db,root,run)
    # Reconcile completed L3 jobs before looking for further ENGINEERING work.
    for raw in db.execute("SELECT * FROM x2_l3 WHERE state IN ('L3_READY','L3_RUNNING')").fetchall():
        row=dict(raw);task=db.execute('SELECT * FROM tasks WHERE name=?',(row['task'],)).fetchone()
        if task and task['state']=='PENDING' and row['source_sha']!=mergeq.integration_head(root):
            db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING',reason='stale L3 source authorization; no runtime started' WHERE name=?",(row['task'],))
            _x2_set(db,row['key'],'L3_BLOCKED','Git SHA changed before dispatch; qualification invalidated')
            continue
        if task and task['state']=='RUNNING':
            if row['state']!='L3_RUNNING':_x2_set(db,row['key'],'L3_RUNNING')
        elif task and task['state'] in ('COMPLETED','FAILED','NEEDS_ENGINEERING'):
            out=Path(run)/'work'/f"{task['name']}.{task['attempts']}"/'l3_outcome.json'
            try: result=json.loads(out.read_text())
            except (ValueError,OSError):result=dict(status='L3_BLOCKED',reason='no validated runtime outcome')
            if row['source_sha']!=mergeq.integration_head(root):result=dict(status='L3_BLOCKED',reason='Git SHA changed; stale qualification')
            _x2_set(db,row['key'],result['status'],result.get('reason',''),evidence=str(out))
            auth=execution.get(db,'execution_authorization',{});auth['mode']=execution.ENGINEERING
            execution.put(db,'execution_authorization',auth);execution.put(db,'execution_mode',execution.ENGINEERING)
    if execution.active_leases(db):return
    active=db.execute("SELECT * FROM x2_l3 WHERE state IN ('FIX_CHECKING','L3_PREFLIGHT') ORDER BY updated LIMIT 1").fetchone()
    if active:
        row=dict(active);out=Path(run)/'l3-fixes'/row['key'];phase='proof' if row['state']=='FIX_CHECKING' else 'preflight'
        result=_x2_poll(db,row,out,phase)
        if result is None:return
        if row['source_sha']!=mergeq.integration_head(root):
            _x2_set(db,row['key'],'L3_BLOCKED','Git SHA changed during '+phase);return
        if not result.get('ok'):
            _x2_set(db,row['key'],'L3_BLOCKED',result.get('reason',phase+' failed'),evidence=str(out));return
        if phase=='proof':
            _x2_set(db,row['key'],'FIX_VERIFIED','orchestrator original-FAIL and fixed regression reruns passed')
            _x2_set(db,row['key'],'INTEGRATED','CAS integration confirmed')
        else:
            report=result['report'];errors=first_l3_errors(report)
            if errors or report.get('integration')!=row['source_sha'] or report.get('verdict')!='PASS':
                _x2_set(db,row['key'],'L3_BLOCKED','; '.join(errors or ['stale/failed preflight']));return
            errors=x2_admission_errors(db,root)
            if errors:_x2_set(db,row['key'],'L3_BLOCKED','; '.join(errors));return
            _x2_set(db,row['key'],'L3_PREFLIGHT',conditions=json.dumps(result['conditions'],sort_keys=True))
            execution.authorize(db,execution.L3,report,root,row['source_sha'])  # new ordinary single-use capability per run
            name='x2_l3_'+row['blocker']+'_'+row['key'][:16]
            _x2_set(db,row['key'],'L3_READY',task=name,evidence=str(out))
            nq.enqueue(db,name,f'python3 tools/nightly/qualification.py run-fix --key {row["key"]}',
                       execution_tier=execution.L3,resource='sim',priority=-10,max_attempts=1,timeout_s=900,heartbeat_s=120)
        return
    if x2_busy(db):return
    if execution.get(db,'x2_auto_l3_enabled',False) is not True:return
    x2_discover(db,root,run)
    raw=db.execute("SELECT * FROM x2_l3 WHERE state IN ('ENGINEERING_VERIFIED','INTEGRATED') ORDER BY updated LIMIT 1").fetchone()
    if not raw:return
    row=dict(raw);out=Path(run)/'l3-fixes'/row['key'];out.mkdir(parents=True,exist_ok=True)
    errors=x2_admission_errors(db,root)
    if errors:_x2_set(db,row['key'],'L3_BLOCKED','; '.join(errors));return
    phase='proof' if row['state']=='ENGINEERING_VERIFIED' else 'preflight'
    job=attempts.VerifyJob(['python3',str(Path(root)/'tools/nightly/qualification.py'),phase+'-fix','--key',row['key']],str(root),str(out/(phase+'.log')),900,10)
    X2_JOBS[row['key']]=job
    _x2_set(db,row['key'],'FIX_CHECKING' if phase=='proof' else 'L3_PREFLIGHT',pid=job.p.pid,evidence=str(out))


def _atomic_json(path, data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2)+'\n');os.replace(temp,path)


def _junit(path):
    root=ET.parse(path).getroot();cases=root.findall('.//testcase')
    return dict(total=len(cases),failed=sum(c.find('failure') is not None for c in cases),
                errors=sum(c.find('error') is not None for c in cases),skipped=sum(c.find('skipped') is not None for c in cases),
                ids=sorted((c.get('classname'),c.get('name')) for c in cases))


def x2_proof_fix(db, row, root, run, out):
    import attempts,execution
    spec=json.loads(row['spec']);trees=[];measurements={}
    for item in spec.get('inputs',[]):
        path=Path(item['path']);path=path if path.is_absolute() else Path(root)/path
        h=hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
        if h.hexdigest()!=item['sha256']:raise ValueError('input SHA256 mismatch: '+str(path))
    try:
        for label,sha in [('before',spec['baseline_sha']),('after',row['source_sha'])]:
            wt,branch=attempts.prepare(str(root),run,'x2_proof_'+label+'_'+row['key'][:12],1,row['key'][:16],sha)
            trees.append(wt)
            if label=='before':
                paths=spec.get('test_files',spec['tests'])
                if not all(_safe_file(p) and (p.startswith('tests/') or '/test/' in p) for p in paths):raise ValueError('only test files may be overlaid on the baseline')
                p=_git(wt,'restore','--source='+row['source_sha'],'--',*paths)
                if p.returncode:raise ValueError('baseline test overlay failed')
            xml=out/(label+'.xml');argv=_pytest(spec['tests'])+['--junitxml='+str(xml)]
            cmd=execution.engineering_sandbox(['bash','-c','source /opt/ros/humble/setup.bash; exec '+shlex.join(argv)],wt,out)
            env=dict(os.environ,ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1',PYTHONDONTWRITEBYTECODE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
            with (out/(label+'.log')).open('w') as log:
                p=subprocess.run(cmd,cwd=wt,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
            measurements[label]=dict(rc=p.returncode,junit=_junit(xml))
        a,b=measurements['before'],measurements['after']
        ok=(a['rc']==1 and a['junit']['failed']>0 and not a['junit']['errors'] and b['rc']==0
            and b['junit']['total']>0 and not b['junit']['failed'] and not b['junit']['errors']
            and not b['junit']['skipped'] and a['junit']['ids']==b['junit']['ids'])
        return dict(ok=ok,reason='' if ok else 'original FAIL/fixed regression not reproduced on identical cases',measurements=measurements)
    finally:
        # These are helper-owned fresh worktrees, never engineering/foreign trees.
        for wt in trees:_git(root,'worktree','remove','--force',wt)


def x2_preflight_fix(db,row,root,run,out):
    import execution
    errors=x2_admission_errors(db,root)
    if errors:return dict(ok=False,reason='; '.join(errors))
    env=dict(os.environ,NIGHTLY_CAMPAIGN=execution.X2_CAMPAIGN)
    with (out/'standard_preflight.log').open('w') as log:
        p=subprocess.run(['python3',str(Path(root)/'tools/nightly/preflight.py'),'all','--mode',execution.L3],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
    report=json.loads((Path(run)/'preflight/latest.json').read_text())
    _atomic_json(out/'standard_preflight.json',report)
    errors=first_l3_errors(report)
    if p.returncode or report.get('verdict')!='PASS':errors.extend('preflight:'+r['name']+':'+r.get('detail','') for r in report.get('checks',[]) if r['status']=='FAIL')
    spec=json.loads(row['spec'])
    versions=[r.get('detail','') for r in report.get('checks',[]) if r['name']=='sim.nav2_version']
    if not spec.get('nav2_version') or not any('twin nav2_amcl '+spec['nav2_version']+' ' in v for v in versions):errors.append('missing/mismatched measured Nav2 version')
    conditions=x2_case_conditions(root,spec)
    if report.get('images')!=conditions['images']:errors.append('Docker image identities changed after preflight')
    if spec.get('requires_mc'):
        contract=__import__('preflight').env_file(Path(root)/'digital_twin/scripts/contract/robot.env')
        errors+=check(db,Path(run)/'nightly_qualification.json',root,row['source_sha'],
                      report.get('robot_sha'),contract['ROBOT_MAP_SHA256'],conditions['images']['stack'],conditions['images']['sim'])
    return dict(ok=not errors,reason='; '.join(errors),report=report,conditions=conditions)


def _x2_witness(out):
    # Passive graph observation, no command publications or hardware access.
    code = r'''import json,os,pathlib,time,rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
assert os.environ.get('QAZBOT_SIM')=='1' and os.environ.get('ROS_DOMAIN_ID')=='87'
out=pathlib.Path('/qualification');rclpy.init();n=rclpy.create_node('x2_l3_passive_witness')
names=set();topics={};subs={};counts={};gids=set();start=time.monotonic()
try:
 while not (out/'witness.stop').exists() and time.monotonic()-start<660:
  rclpy.spin_once(n,timeout_sec=.1)
  names.update(n.get_node_names())
  for topic,types in n.get_topic_names_and_types():
   if topic not in subs and any(t in types for t in ('geometry_msgs/msg/Twist','sensor_msgs/msg/PointCloud2','sensor_msgs/msg/LaserScan','tf2_msgs/msg/TFMessage')):
    typ=types[0];topics[topic]=typ
    def receive(msg,t=topic):counts[t]=counts.get(t,0)+1
    subs[topic]=n.create_subscription(get_message(typ),topic,receive,qos_profile_sensor_data)
   for info in n.get_publishers_info_by_topic(topic):gids.add(bytes(info.endpoint_gid).hex())
  (out/'graph_witness.json').write_text(json.dumps(dict(nodes=sorted(names),topics=topics,counts=counts,publisher_gids=sorted(gids))))
finally:
 n.destroy_node();rclpy.shutdown()
'''
    (out/'observe_graph.py').write_text(code)


def x2_runtime_fix(db,row,root,run,out):
    import execution,fcntl
    task=execution.require(execution.L3,root=root,run=run)
    if task['name']!=row['task']:raise ValueError('L3 task does not own this fix')
    spec=json.loads(row['spec'])
    if x2_case_conditions(root,spec)!=json.loads(row['conditions']):raise ValueError('test conditions changed after authorization')
    auth=execution.get(db,'execution_authorization',{})
    if auth.get('integration_sha')!=row['source_sha']:raise ValueError('stale source capability')
    lockdir=Path(run)/'l3-locks';lockdir.mkdir(exist_ok=True)
    lease=(lockdir/'host-worker.lock').open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # The test owns the real ROS nodes. It runs in the existing hardened internal-network service,
    # with pinned source read-only and evidence writable; no host network or Docker socket binding.
    project='x2fix-'+row['key'][:16]
    compose=['docker','compose','-p',project,'-f',str(Path(root)/'digital_twin/docker/compose.yaml'),'--env-file',str(Path(root)/'digital_twin/.env.lab')]
    env=dict(os.environ,**__import__('preflight').env_file(Path(root)/'digital_twin/.env.lab'),TWIN_VERSION='nightly',TWIN_BACKEND=str(root))
    source=Path(run)/'l3-fixes'/row['key']/'source'
    if not source.exists():
        p=_git(root,'worktree','add','--detach',str(source),row['source_sha'])
        if p.returncode:raise ValueError('cannot pin L3 runtime source')
    if _git(source,'rev-parse','HEAD').stdout.decode().strip()!=row['source_sha']:raise ValueError('runtime source worktree mismatch')
    _x2_witness(out)
    (out/'witness.stop').unlink(missing_ok=True)
    argv=_pytest(spec['runtime_tests'])+['--junitxml=/qualification/runtime.xml']
    command=compose+['run','--rm','--no-deps','-v',str(source)+':/source:ro','-v',str(out)+':/qualification',
                     '-w','/source','-e','QAZBOT_L3_SOURCE_SHA='+row['source_sha'],'-e','QAZBOT_L3_FIX_SHA='+row['fix_sha'],'-e','QAZBOT_L3_BLOCKER='+row['blocker'],'-e','QAZBOT_SIM=1','-e','ROS_DOMAIN_ID=87','-e','ROS_LOCALHOST_ONLY=1',
                     '-e','PYTHONDONTWRITEBYTECODE=1','-e','QAZBOT_L3_EVIDENCE=/qualification','stack-lite','bash','-c',
                     'source /opt/ros/humble/setup.bash; source /opt/x2twin/overlay/setup.bash; '
                     'python3 /qualification/observe_graph.py & witness=$!; '+shlex.join(argv)+
                     '; result=$?; touch /qualification/witness.stop; wait "$witness"; exit "$result"']
    error=None;rc=1
    try:
        with (out/'runtime.log').open('w') as log:
            rc=subprocess.run(command,env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=600).returncode
    finally:
        p=subprocess.run(compose+['down','--remove-orphans'],env=env,cwd=root,capture_output=True,text=True,timeout=90)
        (out/'teardown.log').write_text(p.stdout+p.stderr)
        if p.returncode:error='owned runtime teardown unverified'
        lease.close()
    try:
        junit=_junit(out/'runtime.xml');record=json.loads((out/'runtime_contract.json').read_text())
        witness=json.loads((out/'graph_witness.json').read_text())
    except (ValueError,OSError,ET.ParseError):return dict(status='L3_BLOCKED',reason=error or 'runtime dependencies/report unavailable; no qualified runtime result')
    required=X2_CASES[row['blocker']]
    valid=(rc==0 and junit['total']>0 and not any(junit[k] for k in ('failed','errors','skipped'))
           and record.get('fidelity')=='REAL_ROS2_NAV2' and record.get('source_sha')==row['source_sha']
           and record.get('nav2_version')==spec['nav2_version'] and record.get('safety_violations')==0
           and all(record.get('cases',{}).get(k) is True for k in required))
    observed=set(witness.get('nodes',[]))
    message_types={witness['topics'][t] for t,count in witness.get('counts',{}).items() if count>0}
    valid=valid and bool(witness.get('publisher_gids')) and 'sensor_msgs/msg/PointCloud2' in message_types
    if row['blocker']=='d05':
        valid=valid and observed >= {'controller_server','collision_monitor','goal_coordinator','motion_adapter'} and 'geometry_msgs/msg/Twist' in message_types
    else:
        valid=valid and 'local_costmap' in observed
        if row['blocker']=='d04':valid=valid and bool(observed & {'lidar_projector','qazbot_level_bridge'}) and 'sensor_msgs/msg/LaserScan' in message_types
    if _git(root,'rev-parse','lab/stable').stdout.decode().strip()!=row['source_sha']:return dict(status='L3_BLOCKED',reason='Git SHA changed; runtime outcome is stale')
    return dict(status='L3_PASS' if valid and not error else 'L3_FAIL',reason=error or ('' if valid else 'runtime assertions/coverage/safety contract failed'),
                hardware_qualification='NEEDS_HARDWARE: no measured X2 braking or 21/45 cm box verification',record=record,junit=junit)


def x2_retained_leases(db):
    import execution
    active=execution.active_leases(db)
    return bool(active) and all(x2_task(db,r['name']) for r in active)


def x2_recover(db,root,run):
    """Recover durable outcomes; never duplicate or kill a live/uncertain runtime."""
    import nq
    if not x2_exists(db):return
    for row in db.execute("SELECT t.* FROM tasks t JOIN x2_l3 q ON q.task=t.name WHERE t.state='RUNNING'").fetchall():
        record=x2_task(db,row['name']);out=Path(run)/'work'/f"{row['name']}.{row['attempts']}"/'l3_outcome.json'
        try:
            cmd=Path(f"/proc/{row['pid']}/cmdline").read_bytes() if row['pid'] else b''
            if record['key'].encode() in cmd:continue
        except OSError:pass
        try:data=json.loads(out.read_text())
        except (ValueError,OSError):
            project='x2fix-'+record['key'][:16]
            p=subprocess.run(['docker','ps','-q','--filter','label=com.docker.compose.project='+project],capture_output=True,text=True,timeout=20)
            if p.returncode or p.stdout.strip():
                _x2_set(db,record['key'],'L3_BLOCKED','orphan runtime still present/unknown; task lease retained')
                continue
            data=dict(status='L3_BLOCKED',reason='runtime process ended without durable result; containers absent; no retry')
            _atomic_json(out,data)
        rc=0 if data.get('status')=='L3_PASS' else 1
        from types import SimpleNamespace
        nq.finish(db,row,SimpleNamespace(returncode=rc),Path(row['log_path']),'ok' if rc==0 else 'fail')


def x2_stop(db):
    """Operator STOP owns these finite preflight/proof children only, never an engineer lease."""
    import signal
    if not x2_exists(db):return
    for r in db.execute("SELECT * FROM x2_l3 WHERE state IN ('FIX_CHECKING','L3_PREFLIGHT','L3_READY')").fetchall():
        try:
            pid=r['pid'];cmd=Path(f'/proc/{pid}/cmdline').read_bytes() if pid else b''
            if r['key'].encode() in cmd and os.getpgid(pid)==pid:os.killpg(pid,signal.SIGTERM)
        except (OSError,ProcessLookupError):pass
        _x2_set(db,r['key'],'L3_BLOCKED','operator STOP; evidence retained; no automatic repeat')


def _x2_main():
    import argparse,nq
    p=argparse.ArgumentParser();p.add_argument('command',choices=('proof-fix','preflight-fix','run-fix'));p.add_argument('--key',required=True);a=p.parse_args()
    db=nq.connect();row=dict(db.execute('SELECT * FROM x2_l3 WHERE key=?',(a.key,)).fetchone())
    root,run=nq.ROOT,nq.RUN
    if a.command=='run-fix':out=Path(os.environ['NIGHTLY_RESULT_DIR'])
    else:out=Path(run)/'l3-fixes'/a.key
    out.mkdir(parents=True,exist_ok=True)
    phase={'proof-fix':'proof','preflight-fix':'preflight','run-fix':'l3_outcome'}[a.command]
    try:result={'proof-fix':x2_proof_fix,'preflight-fix':x2_preflight_fix,'run-fix':x2_runtime_fix}[a.command](db,row,root,run,out)
    except (Exception,SystemExit) as exc:result=dict(ok=False,status='L3_BLOCKED',reason=type(exc).__name__+': '+str(exc))
    _atomic_json(out/(phase+'.json'),result)
    if a.command=='run-fix':
        _atomic_json(out/'result.json',dict(scenario_id='x2:'+row['blocker'],seed=row['key'],
                     verdict={'L3_PASS':'PASS','L3_FAIL':'FAIL'}.get(result.get('status'),'UNVERIFIED'),
                     metrics=dict(fidelity='L3',fix_sha=row['fix_sha'],git_sha=row['source_sha'],status=result.get('status')),
                     safety_violations=int(result.get('record',{}).get('safety_violations') or 0),notes=result.get('reason','')))
    return 0 if result.get('ok') or result.get('status')=='L3_PASS' else 1


if __name__=='__main__':
    raise SystemExit(_x2_main())
