#!/usr/bin/env python3
"""SHA-pinned current campaign, dispatched only by nq. First L3 scope is stationary, not navigation E2E."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import execution
import nq

PATH='digital_twin/campaigns/d1_p23_amcl_v1.json'


def load(sha):
    raw=subprocess.check_output(['git','-C',str(nq.ROOT),'show',sha+':'+PATH])
    d=json.loads(raw)
    if (d.get('map')!='d1_qazbot_20261029' or d.get('motion_adapter')!='P23' or d.get('localization')!='amcl' or
            d.get('scan_fov_deg')!=103 or d.get('mc')!='official' or d.get('l3_workers')!=1 or
            d.get('qualification_scope')!='stationary_official_mc' or d.get('optimization_enabled') is not False or
            not 0<d.get('duration_s',0)<=60 or not 0<d.get('max_wall_s',0)<=600):
        raise ValueError('campaign is not the current bounded D1/P23/AMCL qualification profile')
    return d,hashlib.sha256(raw).hexdigest()


def enqueue_current(db,qualification_only):
    sha=nq.mergeq.integration_head(nq.ROOT)
    d,digest=load(sha)
    tier=execution.L3 if qualification_only else execution.FULL
    name='d1_p23_'+time.strftime('%Y%m%d')+'_'+tier.lower()+'_'+sha[:9]+'_'+digest[:9]
    cmd=f'python3 tools/nightly/campaign.py run --source-sha {sha} --campaign-sha {digest} --tier {tier}'
    nq.enqueue(db,name,cmd,execution_tier=tier,resource='sim',priority=40,timeout_s=d['max_wall_s']+120,heartbeat_s=90)
    return name


def run(sha,digest,tier):
    task=execution.require(tier)
    d,actual=load(sha)
    if actual!=digest or sha!=nq.mergeq.integration_head(nq.ROOT):
        raise SystemExit('pinned campaign/source SHA mismatch')
    db=nq.connect();auth=nq.kv_get(db,'execution_authorization',{})
    out=Path(os.environ['NIGHTLY_RESULT_DIR']);out.mkdir(parents=True,exist_ok=True)
    lockdir=nq.RUN/'l3-locks';lockdir.mkdir(exist_ok=True)
    lock=(lockdir/'host-worker.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    project='x2tq-'+hashlib.sha256((task['name']+str(task['attempts'])).encode()).hexdigest()[:16]
    capsule=out/'worker_authorization.json'
    capsule.write_text(json.dumps(dict(mode=auth.get('mode'),scope=d['qualification_scope'],source_sha=sha,campaign_sha=digest,
                                      task=task['name'],expires=time.time()+d['max_wall_s'],l3_workers=1,map=d['map'],mc='official')))
    profile=__import__('preflight').env_file(nq.ROOT/'digital_twin/.env.lab')
    env=dict({**os.environ,**profile},TWIN_MAP_DIR=str(nq.ROOT/'ws/src/maps'/d['map']),TWIN_VERSION='nightly',TWIN_BACKEND=str(nq.ROOT),TWIN_MAP=d['map'],TWIN_LOCALIZATION='amcl',TWIN_MC='official',
             TWIN_SCAN_FOV_DEG='103',NIGHTLY_WORKER_AUTH=str(capsule),NIGHTLY_LOCK_DIR=str(lockdir))
    command=['docker','compose','-p',project,'-f',str(nq.ROOT/'digital_twin/docker/compose.yaml'),'--env-file',str(nq.ROOT/'digital_twin/.env.lab')]
    started=time.monotonic();free_before=__import__('shutil').disk_usage(out).free
    samples=[];error=None;images={};graph={}
    def call(args,timeout=60):
        r=subprocess.run(args,cwd=nq.ROOT/'digital_twin',env=env,capture_output=True,text=True,timeout=timeout)
        return r.returncode,r.stdout+r.stderr
    def stop_signal(_sig,_frame):
        raise InterruptedError('qualification interrupted; evidence preserved')
    previous=signal.signal(signal.SIGTERM,stop_signal)
    try:
        if __import__('preflight').vendor_identity()!=auth.get('vendor_identity'):
            raise ValueError('vendor MC/MuJoCo binaries differ from preflight')
        for kind in ('stack','sim'):
            rc,text=call(['docker','image','inspect',f'x2-twin-{kind}:nightly','--format','{{.Id}}'])
            images[kind]=text.strip()
            if rc or images[kind]!=auth.get('images',{}).get(kind):
                raise ValueError('image digest differs from preflight: '+kind)
        rc,text=call(command+['up','-d','--wait','--wait-timeout','240','sim','stack'],timeout=300)
        (out/'startup.log').write_text(text)
        if rc:raise RuntimeError('official MC/stack startup failed')
        probe="source /opt/ros/humble/setup.bash; source /opt/x2twin/overlay/setup.bash; export QAZBOT_SIM=1 TWIN_ODOM_OWNER=/hal_plant; /opt/x2twin/overlay/lib/qazbot_tf_probe/tf_owner_probe"
        # Same configured TWIN_START as the stack; no pose is guessed for physical hardware.
        profile=__import__('preflight').env_file(nq.ROOT/'digital_twin/.env.lab')
        probe='export TWIN_START='+__import__('shlex').quote(profile['TWIN_START'])+'; '+probe
        rc,text=call(command+['exec','-T','stack','bash','-c',probe],timeout=60)
        (out/'official_tf.log').write_text(text);graph['tf_rc']=rc
        if rc:raise RuntimeError('official plant/AMCL TF ownership failed')
        sensor=(nq.ROOT/'tools/nightly/costmap_sensor_probe.py').read_text()
        rc,text=call(command+['exec','-T','stack','bash','-c',"source /opt/ros/humble/setup.bash; source /opt/x2twin/overlay/setup.bash; export QAZBOT_SIM=1; python3 - <<'QAZBOT_SENSOR'\n"+sensor+'\nQAZBOT_SENSOR'],timeout=30)
        (out/'official_sensor.log').write_text(text);graph['sensor_rc']=rc
        if rc:raise RuntimeError('official costmap sensor graph failed')
        end=time.monotonic()+d['duration_s']
        while time.monotonic()<end:
            if __import__('shutil').disk_usage(out).free<5*2**30:
                raise RuntimeError('disk below existing 5 GiB floor; no evidence deleted')
            ids_rc,ids=call(['docker','ps','-q','--filter','label=com.docker.compose.project='+project])
            if ids_rc or not ids.strip():
                raise RuntimeError('qualification containers disappeared')
            rc,text=call(['docker','stats','--no-stream','--format','{{json .}}',*ids.split()],timeout=15)
            samples.append({'wall_s':time.monotonic()-started,'rc':rc,'docker_stats':text})
            (out/'resource_samples.json').write_text(json.dumps(samples,indent=2))
            print('qualification heartbeat',len(samples),flush=True)
            time.sleep(2)
    except Exception as exc:
        error=type(exc).__name__+': '+str(exc)
    finally:
        cleanup_errors=[]
        for name in ('mc.log','sim.log','world.json'):
            try:
                rc,text=call(command+['exec','-T','sim','cat','/var/log/twin/'+name],timeout=15)
                (out/name).write_text(text)
            except Exception as exc:
                cleanup_errors.append(name+': '+str(exc))
        try:
            rc,text=call(command+['logs','--no-color'],timeout=30)
            (out/'containers.log').write_text(text)
        except Exception as exc:
            cleanup_errors.append('logs: '+str(exc))
        # Teardown must still be attempted if any evidence collector timed out.
        try:
            down_rc,text=call(command+['down','--remove-orphans'],timeout=60)
            (out/'teardown.log').write_text(text)
            if down_rc:cleanup_errors.append('teardown unverified')
        except Exception as exc:
            cleanup_errors.append('teardown: '+str(exc))
        if cleanup_errors:error=(error or '')+' cleanup: '+'; '.join(cleanup_errors)
        signal.signal(signal.SIGTERM,previous)
        capsule.unlink(missing_ok=True)  # revokes this run, never deletes result/evidence/volumes
    result=dict(campaign_id=d['id'],campaign_sha=digest,git_sha=sha,robot_sha=d['robot_sha'],map_sha256=d['map_sha256'],seed=d['seeds'][0],
                scope=d['qualification_scope'],fidelity='L3',images=images,sensor_graph=graph,wall_s=time.monotonic()-started,
                disk_delta_bytes=free_before-__import__('shutil').disk_usage(out).free,
                evidence_bytes=sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),
                resource_samples=samples,rtf=None,missed_mc_cycles=None,raw_contacts=None,
                qualification_status='UNVERIFIED',nav2_action_outcome='NOT_RUN_STATIONARY_SCOPE',
                error=error or 'RTF/MC missed-cycle/contact instrumentation not yet qualified; stationary run is not route E2E')
    (out/'qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    import workflow
    workflow.analyze(db,dict(result,component='qualification_instrumentation' if error is None else 'qualification_runtime',experiment_id=task['name'],
                            evidence=str(out/'qualification.json'),hypothesis_status='HYPOTHESIS'))
    print(json.dumps(result,indent=2))
    return 1  # never manufactures PASS from missing real physics/contact/cycle measurements


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=('run',));p.add_argument('--source-sha',required=True);p.add_argument('--campaign-sha',required=True);p.add_argument('--tier',choices=(execution.L3,execution.FULL),required=True)
    a=p.parse_args();raise SystemExit(run(a.source_sha,a.campaign_sha,a.tier))


if __name__=='__main__': main()
