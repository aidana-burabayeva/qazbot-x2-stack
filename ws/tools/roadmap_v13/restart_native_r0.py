"""Approved R0: hash-guarded C1/C2 promotion + QazBot-only dry-run restart.

No MC service calls, no velocity publisher, no initialpose/goal publication.
Requires explicit command flags after operator permission. Never runs a twin.
"""
import argparse
import base64
import hashlib
import json
import os
import shutil
import subprocess
from datetime import datetime,timezone
from pathlib import Path

ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--operator-confirmed-standing',action='store_true')
a=ap.parse_args()
if not a.execute or not a.operator_confirmed_standing:raise SystemExit('requires approved --execute --operator-confirmed-standing')
root=(Path.home()/'control_backend').resolve();staging=Path(__file__).resolve().parent
freeze=root/'docs/roadmap_v13/evidence/B0_20261027_1030Z'
disk=json.loads((freeze/'disk_manifest.json').read_text());bundle=json.loads((staging/'R0_promotion_bundle.json').read_text())
sha=lambda b:hashlib.sha256(b).hexdigest()
critical=['ws/src/navigation/localization.launch.py','ws/src/navigation/localization_trust_node.py',
 'ws/src/navigation/nav_full.launch.py','ws/src/navigation/nav_control.launch.py','ws/src/navigation/run_nav_full.sh',
 'ws/src/navigation/x2_motion_adapter.py','ws/src/navigation/goal_coordinator.py',
 'ws/src/lio/native_odom_bridge.py','ws/src/lio/level_bridge.py','ws/src/lio/x2.yaml',
 'tools/stack/stack_up.sh','tools/stack/stack_down.sh','ws/src/maps/first_map_nav2_v3/map.yaml','ws/src/maps/first_map_nav2_v3/map.pgm']
for rel in critical:
 if sha((root/rel).read_bytes())!=disk[rel]['sha256']:raise SystemExit('freeze drift: '+rel)
for row in bundle:
 rel=Path(row['path']);target=root/rel
 if rel.is_absolute() or '..' in rel.parts:raise SystemExit('invalid path')
 current=sha(target.read_bytes()) if target.exists() else None
 if current!=row['before_sha256']:raise SystemExit('promotion precondition failed: '+str(rel))
 if sha(base64.b64decode(row['data_b64']))!=row['after_sha256']:raise SystemExit('bundle integrity failed')
oldtag=(root/'logs/stack_last_tag').read_text().strip()
expected={'localization':('localization.launch.py',86415),'trust':('localization_trust_node.py',86641),
          'adapter':('x2_motion_adapter.py',86794),'nav_full':('nav_full.launch.py',92489)}
for stage,(name,pid) in expected.items():
 actual=int((root/f'logs/{stage}_{oldtag}.pid').read_text());cmd=(Path('/proc')/str(actual)/'cmdline').read_bytes().decode()
 if actual!=pid or name not in cmd:raise SystemExit('unexpected running role/PID: '+stage)
 if stage=='adapter' and 'live:=false' not in cmd:raise SystemExit('existing adapter is not dry-run')
stamp=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%SZ');tag='rviz_native_v13_'+stamp
out=root/'docs/roadmap_v13/evidence'/('R0_'+stamp);out.mkdir(parents=True,exist_ok=False)
journal=dict(permission='operator approved R0 and confirmed standing in chat',old_tag=oldtag,new_tag=tag,
 state='preflight_passed',odom_source='native_imu',adapter_live=False,
 first_goal_parameters=dict(xy_tol=.20,yaw_tol=.17453292519943295,goal_checker_stateful=True),changes=[])
def save():
 (out/'journal.json').write_text(json.dumps(journal,indent=2));print(json.dumps(dict(state=journal['state'],tag=tag,out=str(out))),flush=True)
save()
try:
 with (out/'shutdown_old.log').open('w') as log:
  r=subprocess.run(['bash',str(root/'tools/stack/stack_down.sh'),oldtag],stdout=log,stderr=subprocess.STDOUT,timeout=190)
 if r.returncode:raise RuntimeError('old stack not fully stopped; no promotion applied')
 journal['state']='old_stack_stopped';save()
 for row in bundle:
  rel=Path(row['path']);target=root/rel
  if (sha(target.read_bytes()) if target.exists() else None)!=row['before_sha256']:raise RuntimeError('file changed during shutdown: '+str(rel))
  backup=out/'before'/rel;backup.parent.mkdir(parents=True,exist_ok=True)
  mode=target.stat().st_mode&0o777 if target.exists() else 0o664
  if target.exists():shutil.copy2(target,backup)
  target.parent.mkdir(parents=True,exist_ok=True);tmp=target.with_name(target.name+'.r0_tmp')
  with tmp.open('xb') as f:f.write(base64.b64decode(row['data_b64']));f.flush();os.fsync(f.fileno())
  tmp.chmod(mode);os.replace(tmp,target)
  if sha(target.read_bytes())!=row['after_sha256']:raise RuntimeError('post-write integrity failed')
  journal['changes'].append({k:v for k,v in row.items() if k!='data_b64'})
 journal['state']='C1_C2_promoted';save()
 env=dict(os.environ,QAZBOT_TRUST_POLICY='go2',QAZBOT_ODOM_SOURCE='native_imu')
 with (out/'startup_native.log').open('w') as log:
  r=subprocess.run(['bash',str(staging/'pilot_stack_up.sh'),tag],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=480)
 if r.returncode:raise RuntimeError('native dry-run startup failed; inspect exact stage logs')
 journal['state']='native_dry_run_up_needs_operator_seed';save()
except BaseException as exc:
 journal['state']='failed_no_motion';journal['error']=repr(exc);save()
 raise
