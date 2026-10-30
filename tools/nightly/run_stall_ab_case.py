#!/usr/bin/env python3
"""One s1_0046 experiment arm; invoke ONLY via nq --resource sim, never directly.

This calls the existing L2 runner, never docker itself, and preserves each arm's evidence.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--arm',choices=['A','B'],required=True);ap.add_argument('--evidence',required=True)
    args=ap.parse_args();os.chdir(ROOT)
    out=ROOT/args.evidence/args.arm;out.mkdir(parents=True,exist_ok=False)
    epsilon=.02 if args.arm=='A' else .10
    tsv=ROOT/'run/nightly/scenarios/seed1.tsv'
    line=next(x for x in tsv.read_text().splitlines() if x.startswith('s1_0046\t'))
    paths=['ws/src/navigation/x2_motion_adapter.py','ws/src/navigation/nav_control.launch.py','tests/p20_sim/nav2.compose.yaml',
           'tests/p20_sim/run_nav2_trace.sh','tools/nightly/run_l2.sh','ws/src/maps/main_office/map.pgm']
    metadata={'arm':args.arm,'yaw_lin_eps':epsilon,'scenario':line,'started':time.time(),
      'git_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
      'git_diff':subprocess.check_output(['git','diff','--','ws/src/navigation','tests/p20_sim/nav2.compose.yaml'],text=True),
      'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}}
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    before=set((ROOT/'run/nightly/logs').glob('l2_s1_0046_*.out'))
    env=dict(os.environ,P20_YAW_LIN_EPS=str(epsilon))
    p=subprocess.run(['bash','tools/nightly/run_l2.sh','s1_0046',str(tsv.relative_to(ROOT))],env=env)
    created=set((ROOT/'run/nightly/logs').glob('l2_s1_0046_*.out'))-before
    if len(created)!=1:raise RuntimeError('expected one new L2 run log, got '+repr(created))
    run_log=created.pop();run_name=run_log.stem
    runtime=Path('/home/adn/QazBot/digital_twin/results')/run_name
    metadata.update(finished=time.time(),runner_returncode=p.returncode,run_name=run_name,runtime_dir=str(runtime))
    shutil.copyfile(run_log,out/'runner.log')
    for source,target in [('result.json','result.json'),('stack.log','stack.log'),('recorder/status.jsonl','status.jsonl')]:
        if (runtime/source).is_file():shutil.copyfile(runtime/source,out/target)
    for source,target in [('trace.json','trace.json.gz'),('recorder/motion.csv','motion.csv.gz')]:
        if (runtime/source).is_file():
            with (runtime/source).open('rb') as i,gzip.open(out/target,'wb') as o:shutil.copyfileobj(i,o)
    shutil.copyfile(ROOT/'run/nightly/l2_s1_0046.json',out/'l2_summary.json')
    if not (out/'result.json').is_file():raise RuntimeError('no navigation result; infrastructure failed')
    result=json.loads((out/'result.json').read_text());metadata['navigation_result']=result.get('result')
    seen=set()
    if (out/'status.jsonl').exists():
        for row in (out/'status.jsonl').read_text().splitlines():
            d=json.loads(row)
            if d.get('kind')=='adapter' and 'yaw_lin_eps' in d['data']:seen.add(d['data']['yaw_lin_eps'])
    metadata['observed_yaw_lin_eps']=sorted(seen)
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    manifest={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
    (out/'SHA256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(metadata,indent=2),flush=True)
    if seen!={epsilon}:raise RuntimeError('adapter did not acknowledge exactly the requested epsilon: '+repr(seen))
    # A measured navigation FAIL is still a completed experiment; no retry policy here.
    return 0

if __name__=='__main__':raise SystemExit(main())
