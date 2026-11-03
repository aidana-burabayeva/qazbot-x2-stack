#!/usr/bin/env python3
"""Reproduce missing-log-path shutdown and verify the rebuilt binary.

No sensor messages, motion publishers or MC calls. Isolated DDS domain only.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True)
p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--baseline',action='store_true');p.add_argument('--gdb',action='store_true');args=p.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('Isolated localhost domain87 required')
args.output.mkdir(exist_ok=False)

def run(name,enabled,gdb=False):
    path=args.output/(name+'.log');log=path.open('w')
    cmd=[str(args.binary),'--ros-args','--params-file',str(args.config),'-p','runtime_pos_log_enable:='+str(enabled).lower()]
    if gdb:cmd=['gdb','--batch','-ex','handle SIGINT nostop noprint pass','-ex','run','-ex','bt','-ex','info registers x0','--args']+cmd
    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    ready=False;injected=False
    try:
        deadline=time.monotonic()+20
        while proc.poll() is None:
            text=path.read_text()
            if 'Node init finished.' in text:
                ready=True;break
            if time.monotonic()>deadline:raise RuntimeError('Initialization observation deadline exceeded')
            time.sleep(.05)
        if ready:
            target=proc.pid
            if gdb:
                rows=subprocess.check_output(['ps','-eo','pid=,ppid='],text=True).splitlines()
                children=[int(c) for c,parent in (row.split() for row in rows) if int(parent)==proc.pid]
                target=next(c for c in children if str(args.binary).encode() in Path('/proc/'+str(c)+'/cmdline').read_bytes().split(b'\0'))
            os.kill(target,signal.SIGINT);injected=True
        rc=proc.wait(timeout=20)
        text=path.read_text()
        return {'exit_code':rc,'node_initialized':ready,'sigint_injected':injected,
                'explicit_log_open_error':'Cannot open runtime pose log' in text,
                'segfault_in_fclose':'SIGSEGV' in text and 'fclose' in text,
                'log':str(path)}
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=20)
        log.close()

result={'binary':str(args.binary),'qualification':'Optional logging and shutdown only; no estimator performance claim.'}
result['disabled']=run('disabled',False)
if args.gdb:result['gdb_disabled']=run('gdb_disabled',False,True)
if args.baseline:
    result['passed']=result['disabled']['exit_code']==-signal.SIGSEGV
else:
    result['enabled_missing_path']=run('enabled_missing_path',True)
    result['passed']=result['disabled']['exit_code']==0 and result['disabled']['node_initialized'] and result['enabled_missing_path']['exit_code']==1 and result['enabled_missing_path']['explicit_log_open_error']
(args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit('Optional logging regression failed')
