#!/usr/bin/env python3
"""Run whole official Nav2 journeys in explicitly selected private SIM containers.
No container is inferred or replaced. Results and failures remain separate per case.
"""
import argparse,concurrent.futures,hashlib,itertools,json,os,queue,subprocess,time
from pathlib import Path
from parallel_turn_sweep import write_json
ROOT=Path(__file__).resolve().parents[2]

def main():
    p=argparse.ArgumentParser();p.add_argument('--exact-goal',action='store_true');p.add_argument('--decelerations',type=float,nargs='+',default=[.6]);p.add_argument('--cpu-set');p.add_argument('--name',required=True);p.add_argument('--containers',nargs='+',required=True);p.add_argument('--leads',nargs='+',type=float,default=[.45,.65]);p.add_argument('--distances',nargs='+',type=float,default=[1.5]);p.add_argument('--repeats',type=int,default=1);p.add_argument('--motion-noise',type=float,default=0.);p.add_argument('--xy-tol',type=float,default=.1);p.add_argument('--yaw-tol',type=float,default=.1);a=p.parse_args()
    if len(set(a.containers))!=len(a.containers) or len(a.containers)>2:raise ValueError('one or two distinct private containers')
    if a.repeats<1 or any(not .1<=v<=4. for v in a.decelerations) or any(not 0<=v<=1.5 for v in a.leads):raise ValueError('invalid grid')
    for c in a.containers:
        info=json.loads(subprocess.check_output(['docker','inspect',c]))[0]
        if info['State']['Running'] or info['HostConfig']['NetworkMode']!='none' or info['HostConfig']['IpcMode']!='private' or not info['Config']['Labels'].get('qazbot.turn_sweep'):
            raise RuntimeError('requires stopped private sweep worker: '+c)
    if a.cpu_set and len(a.containers)!=1:raise ValueError('CPU override is for one full chain at a time')
    out=ROOT/'logs/twin'/a.name;out.mkdir(exist_ok=False)
    paths=['tools/twin/process_exit_audit.py','tools/twin/docker/env_container.sh','tools/twin/run_interactive.sh','ws/src/navigation/nav_full.launch.py','ws/src/navigation/localization_trust_node.py','tools/twin/mc_plant_bridge.py','tests/nav_e2e/loc_model_fixture.py','runtime_x86/turn_ws/install/qazbot_mc_turn/lib/libqazbot_mc_turn.so','tools/twin/mc_turn_controller/src/controllers.cpp','tools/twin/mc_turn_controller/src/walk_brake.hpp','ws/src/navigation/nav_control.launch.py','tools/twin/interactive_navigation.py']
    hashes=lambda:{s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in paths}
    if a.exact_goal:paths+=['candidates/nav2_exact_goal/goal_endpoint.patch','runtime_x86/exact_goal_ws/install/nav2_smac_planner/lib/libnav2_smac_planner_2d.so']
    before=hashes();tasks=queue.Queue();cases=[]
    for i,(lead,distance,rep,decel) in enumerate(itertools.product(a.leads,a.distances,range(a.repeats),a.decelerations)):
        case=dict(id=f'{a.name}_{i:02d}',lead=lead,distance=distance,repeat=rep,linear_decel=decel);tasks.put(case);cases.append(case)
    write_json(out/'manifest.json',dict(cases=cases,exact_goal=a.exact_goal,cpu_set=a.cpu_set,containers=a.containers,source_sha256=before,motion_noise=a.motion_noise,xy_tol=a.xy_tol,yaw_tol=a.yaw_tol))
    def worker(c):
        results=[]
        if a.cpu_set:subprocess.run(['docker','update','--cpuset-cpus',a.cpu_set,c],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(['docker','start',c],check=True,stdout=subprocess.DEVNULL)
        try:
            while not tasks.empty():
                try:case=tasks.get_nowait()
                except queue.Empty:break
                env=dict(os.environ,QAZBOT_TWIN_CONTAINER=c,QAZBOT_HEADLESS='1',QAZBOT_SIM_MC_TURN='1',QAZBOT_SIM_MC_WALK='1',QAZBOT_SIM_MC_WALK_LEAD=str(case['lead']),QAZBOT_SIM_POSE_REFINE='0',QAZBOT_SIM_MC_LINEAR_DECEL=str(case['linear_decel']),QAZBOT_SIM_EXACT_GOAL='1' if a.exact_goal else '0')
                cmd=[str(ROOT/'tools/twin/run_interactive.sh'),'--plant','official','--no-rviz','--test-goal',str(case['distance']),'--xy-tol',str(a.xy_tol),'--yaw-tol',str(a.yaw_tol),'--amcl-motion-noise',str(a.motion_noise),'--name',case['id']]
                with (out/(case['id']+'.log')).open('w') as log:
                    try:rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=480).returncode
                    except subprocess.TimeoutExpired:rc=124
                capture="import glob,pathlib,shutil,sys;d=pathlib.Path(sys.argv[1]);d.mkdir(exist_ok=True);[shutil.copy2(p,d) for p in glob.glob('/tmp/home/.ros/log/qazbot_*.yaml')]"
                captured=subprocess.run(['docker','exec',c,'python3','-c',capture,'/workspace/control_backend/tests/integration_results/'+case['id']+'/launch_parameters'],capture_output=True,text=True)
                result=dict(case,container=c,exit_code=rc,parameter_capture_exit=captured.returncode)
                session=ROOT/'tests/integration_results'/case['id']/'session.json'
                if session.exists():result['session']=json.loads(session.read_text())
                write_json(out/(case['id']+'_result.json'),result);results.append(result)
                print(json.dumps(result),flush=True)
        finally:subprocess.run(['docker','stop','-t','8',c],stdout=subprocess.DEVNULL,check=False)
        return results
    results=[];errors=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(a.containers)) as pool:
        for f in [pool.submit(worker,c) for c in a.containers]:
            try:results.extend(f.result())
            except Exception as e:errors.append(str(e))
    after=hashes();write_json(out/'summary.json',dict(results=results,errors=errors,sources_changed=[s for s in before if before[s]!=after[s]],completed=len(results),passed=sum(r.get('session',{}).get('within_ground_truth_tolerance',False) and r.get('session',{}).get('clean_shutdown',False) for r in results)))
    return int(bool(errors) or len(results)!=len(cases))
if __name__=='__main__':raise SystemExit(main())
