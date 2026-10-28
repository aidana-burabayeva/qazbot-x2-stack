#!/usr/bin/env python3
"""Build/run a probe against installed Nav2 progress/goal-checker libraries (SIM only)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):
    raise RuntimeError('isolated SIM only')
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);args=ap.parse_args()
args.output.mkdir(parents=True)
root=Path(__file__).resolve().parents[2];base=Path('/opt/ros/humble');source=root/'tests/twin/navigation_checker_probe.cpp'
binary=args.output/'navigation_checker_probe'
command=['g++','-std=c++17',str(source),'-o',str(binary),f'-L{base}/lib',f'-Wl,-rpath,{base}/lib','-lpose_progress_checker','-lsimple_progress_checker','-lsimple_goal_checker','-lstopped_goal_checker','-lrclcpp_lifecycle','-lrclcpp','-lrcutils','-lrcl','-pthread']
command+=['-I'+str(p) for p in sorted((base/'include').iterdir()) if p.is_dir()]+['-I'+str(base/'include')]
subprocess.run(command,check=True)
result=subprocess.run([str(binary)],text=True,capture_output=True)
evidence=dict(passed=result.returncode==0,stdout=result.stdout,stderr=result.stderr,
              source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              libraries={name:hashlib.sha256((base/'lib'/name).read_bytes()).hexdigest() for name in ('libpose_progress_checker.so','libsimple_goal_checker.so','libstopped_goal_checker.so')})
(args.output/'result.json').write_text(json.dumps(evidence,indent=2)+'\n');print(json.dumps(evidence,indent=2))
raise SystemExit(result.returncode)
