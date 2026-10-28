#!/usr/bin/env python3
"""Two isolated official-MC/MuJoCo workers, randomized SIM-only turn parameter grid.
Read-only large vendor assets; private mutable configs/caches/logs/IPC/network per worker.
Does not touch the RViz container or hardware. No result is overwritten.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import queue
import random
import shutil
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
MC=ROOT/'vendor/official/mc/mc-x86-v1.0.0-20260522'
SIM=ROOT/'vendor/official/sim/sim_mujoco-x86-v1.0.0-20260522'


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def write_json(path,value):
    tmp=path.with_name(path.name+'.tmp')
    with tmp.open('w') as stream:
        json.dump(value,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    os.replace(tmp,path)



def clone_runtime(src,dst,shared):
    """Copy mutable text/configs; point immutable assets at a read-only Docker mount."""
    dst.mkdir(parents=True,exist_ok=False)
    def walk(source,target):
        for p in source.iterdir():
            rel=p.relative_to(src);q=target/p.name
            if p.is_symlink():q.symlink_to(os.readlink(p));continue
            if p.is_dir():
                if p.name in ('log','cfg') and rel.parts[0]=='bin':q.mkdir();continue
                if p.name in ('lib','mc3th','meshes','resource','rl_models','animation','mujoco_plugin'):
                    q.symlink_to(shared+'/'+str(rel),target_is_directory=True);continue
                q.mkdir();walk(p,q)
            elif p.suffix in ('.so','.onnx','.stl','.STL','.mcap') or (p.stat().st_mode&0o111 and p.suffix!='.sh'):
                q.symlink_to(shared+'/'+str(rel))
            else:shutil.copy2(p,q)
    walk(src,dst)


def analyze(output):
    meta=json.loads((output/'meta.json').read_text());data=np.load(output/'data.npz');f=data['frames'];o=data['odom'];c=data['cmd']
    if len(f)<10:raise ValueError('missing joined frame telemetry')
    t=f[:,0];i=min(np.searchsorted(t,meta['turn_on']),len(t)-1);target=math.radians(meta['target_deg'])
    frames={}
    for k,name in enumerate(['pelvis','torso','left_ankle','right_ankle']):
        b=2+4*k;heading=np.unwrap(f[:,b+3]);angle=heading-heading[i]
        err=math.atan2(math.sin(angle[-1]-target),math.cos(angle[-1]-target))
        frames[name]=dict(turn_deg=math.degrees(float(angle[-1])),error_deg=math.degrees(err),
                          maximum_turn_deg=math.degrees(float(np.max(abs(angle[i:])))),
                          max_xy_m=float(np.linalg.norm(f[i:,b:b+2]-f[i,b:b+2],axis=1).max()))
    gaps=np.diff(c[:,0]);active=np.any(np.abs(c[:-1,1:])>.005,axis=1)
    active_gap=float(gaps[active].max()) if active.any() else math.inf
    min_turn_height=float(o[o[:,0]>=meta['turn_on'],3].min())
    valid=active_gap<.15 and f[:,1].max()<=.05
    stable=meta['aborted'] is None and min_turn_height>=.6
    passed=bool(valid and stable and meta['controller']['done'] and abs(frames['pelvis']['error_deg'])<=math.degrees(.1)
                and all(abs(frames[x]['error_deg'])<=10 for x in ['torso','left_ankle','right_ankle'])
                and frames['pelvis']['maximum_turn_deg']<=abs(meta['target_deg'])+20
                and frames['pelvis']['max_xy_m']<=.15)
    return dict(valid_measurement=bool(valid),stable_turn=bool(stable),passed=passed,controller=meta['controller'],frames=frames,
                min_height_m=min_turn_height,min_episode_height_m=float(o[:,3].min()),max_active_cmd_gap_s=active_gap,max_cmd_gap_s=float(np.diff(c[:,0]).max()),
                max_join_age_s=float(f[:,1].max()),aborted=meta['aborted'])



def analyze_walk(output):
    meta=json.loads((output/'meta.json').read_text());d=np.load(output/'data.npz');o=d['odom'];c=d['cmd'];f=d['frames']
    t=o[:,0];start=np.searchsorted(t,meta['walk_on']);off=np.searchsorted(t,meta['walk_off']);q=o[start,4:8]
    yaw=math.atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2));direction=np.array([math.cos(yaw),math.sin(yaw)])
    forward=(o[:,1:3]-o[start,1:3])@direction
    tail=(t>=meta['walk_off']-2)&(t<=meta['walk_off'])
    speed=float(np.polyfit(t[tail],forward[tail],1)[0]);gaps=np.diff(c[:,0]);active=np.abs(c[:-1,1])>.005
    gap=float(gaps[active].max());active_odom=o[(t>=meta['walk_on'])&(t<=meta['walk_off']+1.),0]
    valid=len(f)>10 and gap<.15 and f[:,1].max()<=.05 and meta['aborted'] is None and len(active_odom)>2 and np.diff(active_odom).max()<.15
    return dict(valid_measurement=bool(valid),passed=bool(valid and meta['aborted'] is None),
      commanded_speed=meta['walk_command'],measured_speed=speed,speed_gain=speed/meta['walk_command'],
      coast_m=float(forward[-1]-forward[off]),peak_coast_m=float(forward[off:].max()-forward[off]),
      distance_m=float(forward[-1]),max_active_cmd_gap_s=gap,min_height_m=float(o[start:,3].min()),aborted=meta['aborted'])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--walk-speeds',type=float,nargs='+');ap.add_argument('--walk-seconds',type=float,default=6.);ap.add_argument('--adaptive-finish',action='store_true');ap.add_argument('--adapter-zero-burst',action='store_true');ap.add_argument('--scratch-root',type=Path);ap.add_argument('--resume',type=Path);ap.add_argument('--name',default=time.strftime('turn_grid_%Y%m%d_%H%M%S'))
    ap.add_argument('--lead-times',type=float,nargs='+',default=[.12,.16,.20]);ap.add_argument('--angles',type=float,nargs='+',default=[-180,-90,90,180]);ap.add_argument('--tolerance',type=float,default=.06);ap.add_argument('--turn-seconds',type=float,default=12.);ap.add_argument('--image',default='qazbot-turn-worker:20261026');ap.add_argument('--repeats',type=int,default=1);a=ap.parse_args()
    if a.repeats<1:raise ValueError('positive repeat count required')
    archive=ROOT/'logs/twin'/a.name
    if archive.exists():raise ValueError('archive name already used')
    if a.scratch_root:
        run=a.scratch_root/a.name;run.mkdir(parents=True,exist_ok=False)
        out=run/'results';out.mkdir();runtime=run/'workers';runtime.mkdir()
        (run/'startup_logs').mkdir()
    else:
        out=archive;out.mkdir();runtime=ROOT/'runtime_x86/turn_grid_workers'/a.name;runtime.mkdir(parents=True,exist_ok=False)
    sources={str(p.relative_to(ROOT)):digest(p) for p in [ROOT/'tools/twin/mc_turn_after_walk.py',ROOT/'tools/twin/turn_regulator.py']}
    resumed=[]
    if a.resume:
        prior=json.loads((a.resume/'manifest.json').read_text())
        if prior['source_sha256']!=sources:raise ValueError('probe source changed: cannot merge resumed evidence')
        a.walk_seconds=prior.get('walk_seconds',a.walk_seconds)
        cases=prior['cases'];a.tolerance=prior['tolerance'];a.turn_seconds=prior['turn_seconds'];a.adaptive_finish=prior.get('adaptive_finish',False);a.adapter_zero_burst=prior.get('adapter_zero_burst',False)
        for case in cases:
            record=a.resume/(case['id']+'_result.json')
            try:
                result=json.loads(record.read_text())
                # A completed measurement, including a failed accuracy result, is retained.
                json.loads((a.resume/case['id']/'meta.json').read_text())
                if result.get('exit_code')!=0 or 'frames' not in result:continue
                shutil.copytree(a.resume/case['id'],out/case['id'])
                result['resumed_from']=str(a.resume);write_json(out/record.name,result);resumed.append(case['id'])
            except (OSError,ValueError,KeyError):continue
    else:
        cases=([dict(walk_speed=v,repeat=rep) for v,rep in itertools.product(a.walk_speeds,range(a.repeats))] if a.walk_speeds else [dict(lead_time=lead,target_deg=target,repeat=rep) for lead,target,rep in itertools.product(a.lead_times,a.angles,range(a.repeats))])
        random.Random(261026).shuffle(cases)
        for i,case in enumerate(cases):case['id']=f'case_{i:02d}'
    tasks=queue.Queue()
    for case in cases:
        if case['id'] not in resumed:tasks.put(case)
    manifest=dict(walk_seconds=a.walk_seconds,cases=cases,seed=261026,workers=2,backend='official MC v1.0.0 + MuJoCo + Xvfb; flat floor',
                  source_sha256=sources,tolerance=a.tolerance,turn_seconds=a.turn_seconds,adaptive_finish=a.adaptive_finish,adapter_zero_burst=a.adapter_zero_burst,resumed_cases=resumed,
                  scope='direct SDK turn after walking; sequential state within each worker; resumed cases keep original runtime state',
                  physics_timebase='host monotonic; real-time factor not measured',cpu_sets=['0-2,6-8','3-5,9-11'])
    write_json(out/'manifest.json',manifest)
    def worker(index):
        name=f'qazbot-turn-{a.name.lower().replace("_","-")}-w{index}'
        work=runtime/f'w{index}';work.mkdir()
        clone_runtime(MC,work/'mc','/workspace/shared_mc');clone_runtime(SIM,work/'sim','/workspace/shared_sim')
        (work/'agibot_data/var').mkdir(parents=True)
        cmd=['docker','run','-d','--init','--network','none','--ipc','private','--shm-size','256m','--memory','8g','--tmpfs','/tmp:rw,size=512m,mode=1777',
             '--ulimit','core=0','--cpuset-cpus',manifest['cpu_sets'][index],'--user',f'{os.getuid()}:{os.getgid()}',
             '--label','qazbot.turn_sweep='+a.name,'--name',name]
        for env in ['HOME=/tmp/home','QAZBOT_SIM=1','ROS_DOMAIN_ID=87','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1']:
            cmd+=['-e',env]
        for host,container,mode in [(ROOT,'/workspace/control_backend','rw'),(ROOT.parent.parent/'control_backend','/control_backend','rw'),(MC,'/workspace/shared_mc','ro'),(SIM,'/workspace/shared_sim','ro'),
            (work/'mc','/workspace/mc','rw'),(work/'sim','/workspace/sim_mujoco','rw'),
            (ROOT/'runtime_x86/run/aimdk','/workspace/aimdk','ro'),(work/'agibot_data','/agibot/data','rw')]:
            cmd+=['-v',f'{host}:{container}:{mode}']
        if a.scratch_root:cmd+=['-v',f'{run}:/workspace/turn_sweep:rw']
        cmd += [a.image,'sleep','infinity']
        subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL)
        try:
            env=dict(os.environ,QAZBOT_TWIN_CONTAINER=name,QAZBOT_HEADLESS='1',QAZBOT_TWIN_MODEL='official')
            if a.scratch_root:
                env.update(QAZBOT_TWIN_LOG_DIR='/workspace/turn_sweep/startup_logs',QAZBOT_TWIN_HOST_LOG_DIR=str(run/'startup_logs'))
            with (out/f'worker{index}_startup.log').open('w') as log:
                ready=subprocess.run([str(ROOT/'tools/twin/docker/sim_restart.sh'),f'{a.name}_w{index}'],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
            if ready.returncode:raise RuntimeError('MC/sim startup failed')
            print(f'worker {index} READY ({name})',flush=True)
            while True:
                try:case=tasks.get_nowait()
                except queue.Empty:break
                result=dict(case,worker=index,container=name)
                destination=out/case['id']
                inside='/workspace/turn_sweep/results/'+case['id'] if a.scratch_root else '/workspace/control_backend/'+str(destination.relative_to(ROOT))
                argv=['docker','exec',name,'/workspace/control_backend/tools/twin/docker/env_container.sh','python3',
                      '/workspace/control_backend/tools/twin/mc_turn_after_walk.py',inside,'--angular','.8','--target-deg',str(case.get('target_deg',0)),
                      '--lead-time',str(case.get('lead_time',.2)),'--tolerance',str(a.tolerance),'--turn-seconds',str(a.turn_seconds)]
                if 'walk_speed' in case:argv+=['--walk-only','--walk-speed',str(case['walk_speed']),'--walk-seconds',str(a.walk_seconds)]
                if a.adaptive_finish:argv+=['--adaptive-finish']
                if a.adapter_zero_burst:argv+=['--adapter-zero-burst']
                try:
                    with (out/(case['id']+'.log')).open('w') as log:
                        trial=subprocess.run(argv,stdout=log,stderr=subprocess.STDOUT,timeout=65)
                    result['exit_code']=trial.returncode
                    if trial.returncode==0:result.update(analyze_walk(destination) if 'walk_speed' in case else analyze(destination))
                    else:result.update(valid_measurement=False,passed=False,error='probe exited; inspect log')
                except Exception as error:result.update(valid_measurement=False,passed=False,error=str(error))
                write_json(out/(case['id']+'_result.json'),result)
                print(json.dumps({k:v for k,v in result.items() if k not in ('frames','controller')}),flush=True)
        finally:subprocess.run(['docker','stop','-t','8',name],stdout=subprocess.DEVNULL,check=False)
    failures=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures=[pool.submit(worker,i) for i in range(2)]
        for i,f in enumerate(futures):
            try:f.result()
            except Exception as error:failures.append(dict(worker=i,error=str(error)))
    changed=[p for p,h in sources.items() if digest(ROOT/p)!=h]
    results=[json.loads(p.read_text()) for p in sorted(out.glob('case_*_result.json'))]
    summary=dict(manifest=manifest,results=results,worker_failures=failures,missing_cases=len(cases)-len(results),sources_changed=changed,
                 total=len(results),valid=sum(r['valid_measurement'] for r in results),passed=sum(r['passed'] for r in results),
                 qualification='SIM parameter screening only; not independently repeated hardware validation')
    write_json(out/'summary.json',summary)
    if a.scratch_root:
        print('RAM_RESULTS',out,flush=True)
        shutil.copytree(out,archive)
        (archive/'ARCHIVE_COMPLETE').write_text(str(out)+'\n')
        print('ARCHIVED',archive,flush=True)
    print('SUMMARY',json.dumps({k:v for k,v in summary.items() if k not in ('manifest','results')}),flush=True)
    return 0 if not failures and not changed and len(results)==len(cases) else 1


if __name__=='__main__':raise SystemExit(main())
