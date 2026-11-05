#!/usr/bin/env python3
"""Fingerprint the simulation-only refinement plugin and its source."""
import argparse,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
ap=argparse.ArgumentParser();ap.add_argument('--write',action='store_true');a=ap.parse_args()
prefix=ROOT/'runtime_x86/pose_refine_ws/install/qazbot_pose_refine'
library=prefix/'lib/libqazbot_pose_refine.so';manifest=prefix/'qazbot_pose_refine.json'
files=['candidates/nav2_pose_refine/'+name for name in ('controller.cpp','CMakeLists.txt','package.xml','plugins.xml')]
try:
    value=dict(schema=1,simulation_only=True,library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
               sources={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files})
    if a.write:manifest.write_text(json.dumps(value,indent=2)+'\n')
    else:
        if json.loads(manifest.read_text())!=value:raise ValueError('refinement build differs from source')
except (OSError,ValueError) as exc:
    if a.write:raise
    print(str(exc));raise SystemExit(1)
