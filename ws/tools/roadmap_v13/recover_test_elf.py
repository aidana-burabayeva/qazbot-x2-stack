"""Repair only 11 positively prefix-matched, non-runtime test executables.

Arguments: evidence directory containing recovery_prefix_checks.json and payload.
Validates every input before writes; no ROS, services, reloads or restarts.
"""
import hashlib
import json
import os
import shutil
import stat
import struct
import sys
import tarfile
from pathlib import Path

root=(Path.home()/'control_backend').resolve();evidence=Path(sys.argv[1]).resolve()
if root/'docs' not in evidence.parents:raise ValueError('robot docs evidence directory required')
checks=json.loads((evidence/'recovery_prefix_checks.json').read_text());assert len(checks)==11
backup=evidence/'test_elf_recovery_backup';backup.mkdir(exist_ok=False)
sha=lambda b:hashlib.sha256(b).hexdigest()
prepared=[]
with tarfile.open(evidence/'verified_test_elf_payload.tar') as tf:
 for r in checks:
  rel=Path(r['path']);target=root/rel
  assert str(rel).startswith('candidates/nav2_smac_bounds/build/nav2_smac_planner/test/test_')
  assert not target.is_symlink() and root in target.resolve().parents
  old=target.read_bytes();full=tf.extractfile(str(rel)).read()
  assert sha(old)==r['damaged_sha256'] and sha(full)==r['reference_sha256']
  assert r['exact_prefix_match'] and full[:len(old)]==old and len(full)>len(old)
  assert full[:4]==b'\x7fELF' and full[4:6]==bytes([2,1])
  assert struct.unpack_from('<H',full,18)[0]==183
  ph,sh=struct.unpack_from('<QQ',full,32);psz,pn,ssz,sn=struct.unpack_from('<HHHH',full,54)
  assert ph+psz*pn<=len(full) and sh+ssz*sn<=len(full)
  prepared.append((r,target,full))
 for p in Path('/proc').glob('[0-9]*/maps'):
  try:maps=p.read_text()
  except OSError:continue
  if any(str(t) in maps for _,t,_ in prepared):raise RuntimeError('test executable mapped in a running process')
result=[]
for r,target,full in prepared:
 b=backup/r['path'];b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(target,b)
 tmp=target.with_name(target.name+'.v13_recovery_tmp')
 with tmp.open('xb') as f:f.write(full);f.flush();os.fsync(f.fileno())
 tmp.chmod(int(r['reference_mode'],8));os.replace(tmp,target)
 assert sha(target.read_bytes())==r['reference_sha256']
 result.append(dict(path=r['path'],before=r['damaged_sha256'],after=r['reference_sha256'],
                    backup=str(b),mode=oct(stat.S_IMODE(target.stat().st_mode))))
(evidence/'test_elf_recovery_result.json').write_text(json.dumps(result,indent=2))
print(json.dumps(dict(repaired=len(result),runtime_changes=False,backup=str(backup)),indent=2))
