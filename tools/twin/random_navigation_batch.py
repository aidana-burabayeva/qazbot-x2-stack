#!/usr/bin/env python3
"""Host-side independent trials for a saved random target set; isolated containers.

Every attempt starts at the known virtual spawn. Failures remain failures; no retry,
no hidden correction of localization. Parallelism is ordinary test processes only.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from analyze_navigation_accuracy import analyze

ROOT=Path(__file__).resolve().parents[2]
ap=argparse.ArgumentParser();ap.add_argument('targets',type=Path);ap.add_argument('output',type=Path)
ap.add_argument('--containers',nargs='+',required=True);ap.add_argument('--prefix',required=True)
ap.add_argument('--transform-tolerance',type=float)
ap.add_argument('--laser-model',choices=['likelihood_field','likelihood_field_prob']);ap.add_argument('--max-beams',type=int)
ap.add_argument('--pose-refine',action='store_true')
ap.add_argument('--exact-goal',action='store_true')
ap.add_argument('--test-timeout',type=float,default=180.)
a=ap.parse_args();a.output.mkdir()
manifest=json.loads(a.targets.read_text());results=[]
for container in a.containers:
    network=subprocess.check_output(['docker','inspect','-f','{{.HostConfig.NetworkMode}}',container],text=True).strip()
    if network!='none':raise RuntimeError('network must be none: '+container)
def run_group(pair):
    worker,targets=pair;completed=[]
    for target in targets:
        name=f"{a.prefix}_{target['index']:02d}";session=ROOT/'tests/integration_results'/name
        if session.exists():raise RuntimeError('refusing overwrite '+str(session))
        log=a.output/(name+'.log');command=[str(ROOT/'tools/twin/run_interactive.sh'),'--no-rviz','--name',name,'--test-timeout',str(a.test_timeout),'--test-pose',*[str(v) for v in target['goal']]]
        if a.transform_tolerance is not None:command+=['--amcl-transform-tolerance',str(a.transform_tolerance)]
        if a.laser_model is not None:command+=['--amcl-laser-model',a.laser_model]
        if a.max_beams is not None:command+=['--amcl-max-beams',str(a.max_beams)]
        env=dict(os.environ,QAZBOT_TWIN_CONTAINER=worker,QAZBOT_SIM_MC_TURN='0',QAZBOT_SIM_MC_WALK='0',QAZBOT_SIM_POSE_REFINE='1' if a.pose_refine else '0',QAZBOT_SIM_EXACT_GOAL='1' if a.exact_goal else '0')
        env.pop('QAZBOT_SIM_MC_LINEAR_DECEL', None)
        source_files=['tools/twin/dds_sim_udp_loopback.xml' if env.get('QAZBOT_SIM_DDS_UDP_ONLY')=='1' else 'tools/twin/dds_sim_loopback.xml','tools/twin/docker/env_container.sh','tools/twin/interactive_navigation.py','ws/src/navigation/nav_full.launch.py','ws/src/navigation/nav_control.launch.py','ws/src/navigation/bt/navigate_to_pose.xml','tools/twin/amcl_sim.launch.py','ws/src/navigation/goal_coordinator.py','ws/src/navigation/goal_alignment_watch.py','ws/src/navigation/localization_trust_node.py','ws/src/navigation/source_stamp.py']
        if a.pose_refine:source_files+=['candidates/nav2_pose_refine/controller.cpp','runtime_x86/pose_refine_ws/install/qazbot_pose_refine/lib/libqazbot_pose_refine.so']
        if a.exact_goal:source_files+=['candidates/nav2_exact_goal/goal_endpoint.patch','runtime_x86/exact_goal_ws/install/nav2_smac_planner/lib/libnav2_smac_planner_2d.so']
        hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_files}
        print('START '+json.dumps(dict(worker=worker,**target)),flush=True);started=time.time()
        with log.open('w') as stream:
            process=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
        result=dict(**target,worker=worker,name=name,duration_s=time.time()-started,returncode=process.returncode,reset_to_known_spawn=True,source_hashes=hashes)
        try:
            metadata=json.loads((session/'session.json').read_text());journey=json.loads((session/'journey.json').read_text());terminal=next(e for e in reversed(journey['events']) if 'state' in e)
            result.update(state=metadata['state'],passed=metadata['within_ground_truth_tolerance'],workflow_succeeded=metadata['workflow_succeeded'],shutdown=metadata['shutdown'],endpoint=terminal)
            result['accuracy']=analyze(session)
            (session/'accuracy.json').write_text(json.dumps(result['accuracy'],indent=2))
        except (OSError,ValueError,KeyError,StopIteration) as exc:result.update(state='setup_or_record_error',passed=False,error=str(exc))
        (a.output/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n');completed.append(result);print('RESULT '+json.dumps({k:v for k,v in result.items() if k not in ('accuracy','source_hashes','endpoint')}),flush=True)
    return completed
with ThreadPoolExecutor(max_workers=len(a.containers)) as pool:
    futures=[pool.submit(run_group,(worker,manifest['targets'][i::len(a.containers)])) for i,worker in enumerate(a.containers)]
    for future in as_completed(futures):
        results+=future.result()
        summary=dict(targets=manifest,results=sorted(results,key=lambda r:r['index']),passed=sum(bool(r.get('passed')) for r in results),completed=len(results))
        (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(dict(passed=sum(bool(r.get('passed')) for r in results),total=len(results))),flush=True)
