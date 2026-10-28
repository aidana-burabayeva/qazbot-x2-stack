"""Physical X2 scheduler A/B. Dry run by default; never commands MC/restarts ROS.

Execute only after operator approval of B1-experiment-card.md. Existing node PID
and start ticks stay fixed. A and B use one long-lived observer. Always restore
affinity, including new threads, on completion/abort. Artifacts under robot docs.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ap=argparse.ArgumentParser()
ap.add_argument('--out',required=True,type=Path);ap.add_argument('--lio-pid',type=int,default=86463)
ap.add_argument('--bridge-pid',type=int,default=86488);ap.add_argument('--trust-pid',type=int,default=86641)
ap.add_argument('--phase-seconds',type=float,default=290);ap.add_argument('--warmup-seconds',type=float,default=30)
ap.add_argument('--execute',action='store_true');ap.add_argument('--operator-confirmed-standing',action='store_true')
a=ap.parse_args();tool=Path(__file__).with_name('burst_probe_v13.py')
if a.phase_seconds<250 or a.warmup_seconds<20:raise SystemExit('preserve original 40/100/160/220-second joins and warmup')
plan=dict(pid=a.lio_pid,phases=['A_pinned','B_unpinned'],joins=[[40,'bare'],[100,'sub'],[160,'bare'],[220,'sub']],
 phase_seconds=a.phase_seconds,warmup_seconds=a.warmup_seconds,no_restart=True,no_MC_commands=True,restore_at_end=True)
if not a.execute:print(json.dumps(plan,indent=2));raise SystemExit(0)
if not a.operator_confirmed_standing:raise SystemExit('explicit operator standing confirmation required')
root=(Path.home()/'control_backend/docs').resolve();out=a.out.resolve()
if root not in out.parents:raise SystemExit('output must be inside robot control_backend/docs')
out.mkdir(parents=True,exist_ok=False)
proc=Path('/proc')/str(a.lio_pid)
def identity():
    text=(proc/'stat').read_text();tail=text[text.rfind(')')+2:].split()
    cmd=(proc/'cmdline').read_bytes().decode().split('\0')
    if not cmd or not cmd[0].endswith('/fastlio_mapping'):raise RuntimeError('wrong process')
    return tail[19]  # starttime, field22; tail starts at field3
start_ticks=identity()
def tasks():return [int(p.name) for p in (proc/'task').iterdir()]
original={tid:sorted(os.sched_getaffinity(tid)) for tid in tasks()}
base=original[a.lio_pid]
if base!=[5,6] or any(mask!=base for mask in original.values()):raise RuntimeError('unexpected initial affinity')
all_cpus=sorted(os.sched_getaffinity(0))
if all_cpus!=list(range(8)):raise RuntimeError('expected this host eight-CPU allowed set')
children=[];phases=[];stopping=[False];result=dict(plan=plan,original_affinity=original,start_ticks=start_ticks)
signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
def launch(label,args):
    log=(out/(label+'.log')).open('w')
    p=subprocess.Popen([sys.executable,str(tool),*args],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((p,log));return p
def safety():
    if stopping[0]:raise RuntimeError('operator interrupt')
    if identity()!=start_ticks:raise RuntimeError('PID/start time changed')
    d=json.loads((out/'safety_AB.json').read_text());now=time.monotonic_ns()
    for key in ['mc','adapter','coordinator']:
        if now-d[key]['received_mono_ns']>3e9:raise RuntimeError('safety status stale: '+key)
    mc=d['mc'];adapter=d['adapter']['data'];coord=d['coordinator']['data']
    if not mc.get('source_real_ns') or not 0<=time.time_ns()-mc['source_real_ns']<=500000000:
        raise RuntimeError('MC DDS publication is stale/future, not just callback receipt')
    if mc['source']!='rc' or mc['action']!='STAND_DEFAULT' or mc['body']!=1 or mc['fsm']!=4:
        raise RuntimeError('robot not confirmed RC/STAND/FSM4: '+json.dumps(mc))
    if adapter.get('live') is not False or not adapter.get('dry_run') or adapter.get('counters',{}).get('published')!=0:
        raise RuntimeError('adapter must remain dry-run with zero publications')
    if coord.get('busy') or coord.get('gate'):raise RuntimeError('active coordinator/goal')
    if d.get('zero_stamps'):raise RuntimeError('zero/invalid sensor stamp observed; separate issue')
    for p,_ in children[:2]:
        if p.poll() is not None:raise RuntimeError('observer/sampler exited')
def wait(seconds,guard=True):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if guard:safety()
        elif stopping[0]:raise RuntimeError('interrupted during warmup')
        time.sleep(min(.25,max(0.,deadline-time.monotonic())))
def affinity(cpus):
    if identity()!=start_ticks:raise RuntimeError('PID changed before affinity')
    for tid in tasks():
        try:os.sched_setaffinity(tid,cpus)
        except ProcessLookupError:pass
    actual={tid:sorted(os.sched_getaffinity(tid)) for tid in tasks()}
    if any(v!=list(cpus) for v in actual.values()):raise RuntimeError('thread affinity verification failed')
    return actual
try:
    total=a.warmup_seconds+2*a.phase_seconds+15
    obs=launch('observer',['observe','--duration',str(total),'--out',str(out),'--tag','AB','--safety-sidecar'])
    launch('sampler',['sched','--duration',str(total+5),'--out',str(out),'--tag','AB','--period','.1','--pids',
        f'fastlio={a.lio_pid}',f'bridge={a.bridge_pid}',f'trust={a.trust_pid}',f'observer={obs.pid}'])
    wait(a.warmup_seconds,guard=False);safety()
    for label,mask in [('A_pinned',base),('B_unpinned',all_cpus)]:
        safety();before=affinity(mask);phase=dict(label=label,cpus=mask,start_real_ns=time.time_ns(),start_mono_ns=time.monotonic_ns(),threads=before)
        phases.append(phase);(out/'phases.json').write_text(json.dumps(phases,indent=2));begin=time.monotonic()
        for offset,variant in plan['joins']:
            wait(max(0.,begin+offset-time.monotonic()));safety()
            join=launch(f'join_{label}_{offset}',['join','--variant',variant,'--duration','8','--out',str(out),'--tag',f'{label}_{offset}'])
            while join.poll() is None:wait(.25)
            if join.returncode!=0:raise RuntimeError('join process failed')
        wait(max(0.,begin+a.phase_seconds-time.monotonic()))
        phase.update(end_real_ns=time.time_ns(),end_mono_ns=time.monotonic_ns())
        (out/'phases.json').write_text(json.dumps(phases,indent=2))
    result['outcome']='measured_not_yet_analyzed'
except BaseException as exc:
    result['outcome']='aborted';result['error']=repr(exc)
finally:
    try:result['restored_affinity']=affinity(base)
    except Exception as exc:result['restore_error']=repr(exc)
    for p,log in reversed(children):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
        log.close()
    result['phases']=phases;(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if result.get('error') or result.get('restore_error'):raise SystemExit(1)
