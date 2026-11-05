#!/usr/bin/env python3
"""Create a NEW private official-MC worker after the research relocation; never replaces a container."""
import argparse,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'ws/tools/twin'))
from parallel_turn_sweep import clone_runtime,MC,SIM
p=argparse.ArgumentParser();p.add_argument('name');a=p.parse_args()
if not a.name.startswith('qazbot-') or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-_' for c in a.name):p.error('use a simple qazbot- name')
if subprocess.run(['docker','inspect',a.name],capture_output=True).returncode==0:raise RuntimeError('container exists; nothing replaced')
work=ROOT/'runtime_x86/research_workers'/a.name;work.mkdir(parents=True,exist_ok=False)
clone_runtime(MC,work/'mc','/workspace/shared_mc');clone_runtime(SIM,work/'sim','/workspace/shared_sim');(work/'agibot_data/var').mkdir(parents=True)
cmd=['docker','create','--init','--network','none','--ipc','private','--shm-size','256m','--memory','8g','--tmpfs','/tmp:rw,size=512m,mode=1777','--ulimit','core=0','--user',f'{os.getuid()}:{os.getgid()}','--label','qazbot.turn_sweep=research_20261027','--name',a.name]
for e in ['HOME=/tmp/home','QAZBOT_SIM=1','ROS_DOMAIN_ID=87','ROS_LOCALHOST_ONLY=1']:cmd+=['-e',e]
for host,dest,mode in [(ROOT,'/workspace/control_backend','rw'),(ROOT.parent.parent/'control_backend','/control_backend','rw'),(MC,'/workspace/shared_mc','ro'),(SIM,'/workspace/shared_sim','ro'),(work/'mc','/workspace/mc','rw'),(work/'sim','/workspace/sim_mujoco','rw'),(ROOT/'runtime_x86/run/aimdk','/workspace/aimdk','ro'),(work/'agibot_data','/agibot/data','rw')]:cmd+=['-v',f'{host}:{dest}:{mode}']
cmd+=['qazbot-turn-worker:20261026','sleep','infinity'];subprocess.run(cmd,check=True)
(work/'manifest.json').write_text(json.dumps(dict(command=cmd,scope='private SIM worker; original containers retained'),indent=2)+'\n')
