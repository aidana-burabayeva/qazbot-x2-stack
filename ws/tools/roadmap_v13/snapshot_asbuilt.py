"""Read-only as-built snapshot. Writes only a new directory under robot docs.

No ROS participant, no attach/debugger, no service calls or runtime changes.
"""
import hashlib
import json
import os
import platform
import stat
import struct
import subprocess
import sys
import tarfile
from datetime import datetime,timezone
from pathlib import Path

root=(Path.home()/'control_backend').resolve()
out=Path(sys.argv[1]).resolve()
if root/'docs' not in out.parents:raise ValueError('output must be inside robot control_backend/docs')
out.mkdir(parents=True,exist_ok=False)
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
trees=['ws/src','candidates','tests','tools/stack','tools/diagnostics']
files={};errors=[]
for tree in trees:
 base=root/tree
 if not base.exists():continue
 for p in sorted(base.rglob('*')):
  if p.is_dir() and not p.is_symlink():continue
  rel=str(p.relative_to(root))
  try:
   s=p.lstat();d=dict(size=s.st_size,mode=oct(stat.S_IMODE(s.st_mode)),mtime_ns=s.st_mtime_ns,inode=s.st_ino)
   if p.is_symlink():d['symlink']=os.readlink(p)
   elif p.is_file():
    d['sha256']=sha(p)
    with p.open('rb') as f:hdr=f.read(64)
    if hdr[:4]==b'\x7fELF':
     end='<' if hdr[5]==1 else '>';d['elf_class']=hdr[4]
     d['elf_machine']=struct.unpack_from(end+'H',hdr,18)[0] if len(hdr)>=20 else None
     if hdr[4]==2 and len(hdr)>=64:
      phoff,shoff=struct.unpack_from(end+'QQ',hdr,32)
      phsz,phn,shsz,shn=struct.unpack_from(end+'HHHH',hdr,54)
      d['elf_table_beyond_eof']=(phoff+phsz*phn>s.st_size or shoff+shsz*shn>s.st_size)
   files[rel]=d
  except (OSError,ValueError,struct.error) as exc:errors.append([rel,repr(exc)])
(out/'disk_manifest.json').write_text(json.dumps(files,indent=2))
proc={};loaded={}
want=['fastlio_mapping','level_bridge.py','native_odom_bridge.py','localization_trust_node.py',
      'goal_coordinator.py','x2_motion_adapter.py','planner_server','controller_server','/amcl ',
      'bt_navigator','velocity_smoother','ros2 launch']
env_keys={'ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE',
          'CYCLONEDDS_URI','QAZBOT_SMAC_PREFIX','QAZBOT_LIFECYCLE_PREFIX','ROS_LOG_DIR','AMENT_PREFIX_PATH',
          'PYTHONPATH','LD_LIBRARY_PATH','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS'}
for p in Path('/proc').glob('[0-9]*'):
 try:
  args=(p/'cmdline').read_bytes().decode(errors='replace').split('\0');cmd=' '.join(args)
  if not any(k in cmd for k in want):continue
  if 'snapshot_asbuilt' in cmd:continue
  d=dict(args=args,cwd=os.readlink(p/'cwd'),exe=os.readlink(p/'exe'),
         affinity=sorted(os.sched_getaffinity(int(p.name))),stat=(p/'stat').read_text(),schedstat=(p/'schedstat').read_text())
  env=(p/'environ').read_bytes().decode(errors='replace').split('\0')
  d['environment']={s.split('=',1)[0]:s.split('=',1)[1] for s in env if '=' in s and s.split('=',1)[0] in env_keys}
  d['parameter_files']={}
  for i,a in enumerate(args[:-1]):
   if a=='--params-file':
    f=Path(args[i+1]);d['parameter_files'][str(f)]=dict(sha256=sha(f),content=f.read_text())
  mappings=[]
  for line in (p/'maps').read_text().splitlines():
   parts=line.split(maxsplit=5)
   if len(parts)<6 or not parts[5].startswith('/'):continue
   path=parts[5];mappings.append(dict(mapping=line,path=path))
   if path in loaded:continue
   f=Path(path.removesuffix(' (deleted)'))
   try:loaded[path]=dict(disk_sha256=sha(f),disk_inode=f.stat().st_ino,mapped_inode=int(parts[4]),size=f.stat().st_size,deleted=path.endswith(' (deleted)'))
   except OSError as exc:loaded[path]=dict(error=repr(exc))
  d['mappings']=mappings;proc[p.name]=d
 except (OSError,ProcessLookupError,PermissionError) as exc:errors.append([str(p),repr(exc)])
(out/'process_manifest.json').write_text(json.dumps(proc,indent=2))
(out/'mapped_files.json').write_text(json.dumps(loaded,indent=2))
env=dict(utc=datetime.now(timezone.utc).isoformat(),host=platform.node(),machine=platform.machine(),
 python=sys.version,clock_ticks=os.sysconf('SC_CLK_TCK'),errors=errors,
 loaded_python_source_hash='not observable without process introspection; disk hash is not a loaded-code hash')
(out/'environment.json').write_text(json.dumps(env,indent=2))
archive=out/'asbuilt.tar.gz'
with tarfile.open(archive,'w:gz',compresslevel=1,dereference=False) as tf:
 for tree in trees:
  if (root/tree).exists():tf.add(root/tree,arcname=tree,recursive=True)
(out/'snapshot_sha256.json').write_text(json.dumps({p.name:sha(p) for p in out.iterdir() if p.is_file()},indent=2))
print(json.dumps(dict(output=str(out),files=len(files),processes=len(proc),mapped_files=len(loaded),
 archive_bytes=archive.stat().st_size,errors=len(errors)),indent=2))
