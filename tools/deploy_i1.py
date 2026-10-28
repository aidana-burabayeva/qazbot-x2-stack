#!/usr/bin/env python3
"""Explicit, operator-approved I1 file installation or verified rollback.

Does not start ROS, MC, adapters or goals. Run only after approval. The robot
deployed directory need not contain .git; a source SHA and file hashes are
required in the resulting deployment manifest.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

BASE = {
    'ws/src/lio/level_bridge.py': '86c434ce0ee5a48de1a324f7a3eb5e2136f8d9d3f1b91b582f6fa522707be686',
    'ws/src/navigation/nav_control.launch.py': 'ebea6637578530f0b30be6dd5d720ef1011270ce2a4df902d2bd1e1b5c266bec',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def no_stack():
    patterns = ('fastlio_mapping', '/lio/level_bridge.py', '/lio/native_odom_bridge.py',
                'localization_trust_node.py', 'nav2_costmap_2d/nav2_costmap_2d',
                'nav2_amcl/amcl', 'nav2_controller/controller_server',
                'nav2_bt_navigator/bt_navigator', 'x2_motion_adapter.py', 'goal_coordinator.py')
    rows = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True).splitlines()
    running = [row for row in rows if any(p in row for p in patterns)]
    if running:
        raise RuntimeError('ROS owner still running; refusing file changes: '+repr(running))


def replace(path, data):
    mode = path.stat().st_mode & 0o777
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
        staged = Path(f.name)
    staged.chmod(mode)
    os.replace(staged, path)


def restore(root, backup, manifest):
    for rel, item in manifest['files'].items():
        target, old = root/rel, backup/'original'/rel
        if sha(old) != item['before']:
            raise RuntimeError('Backup integrity failed: '+rel)
        if sha(target) not in (item['before'], item['after']):
            raise RuntimeError('Unexpected third-party file change; refusing overwrite: '+rel)
    for rel, item in manifest['files'].items():
        replace(root/rel, (backup/'original'/rel).read_bytes())
        if sha(root/rel) != item['before']:
            raise RuntimeError('Rollback verification failed: '+rel)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=Path('/home/agi/control_backend'))
    p.add_argument('--backup', type=Path, required=True)
    p.add_argument('--source', type=Path)
    p.add_argument('--source-commit')
    p.add_argument('--restore', action='store_true')
    args = p.parse_args()
    if not args.backup.resolve().is_relative_to((args.root/'docs').resolve()):
        p.error('Backup must be under the operator-approved control_backend/docs')
    no_stack()
    if args.restore:
        manifest = json.loads((args.backup/'deployment.json').read_text())
        restore(args.root, args.backup, manifest)
        manifest['status'] = 'ROLLED_BACK'
    else:
        if not args.source or not re.fullmatch('[0-9a-f]{40}', args.source_commit or ''):
            p.error('--source and exact --source-commit are required')
        payload = {rel: (args.source/rel).read_bytes() for rel in BASE}
        for rel, expected in BASE.items():
            if sha(args.root/rel) != expected:
                raise RuntimeError('Robot baseline changed; refusing overwrite: '+rel)
            compile(payload[rel], rel, 'exec')
        profile = args.source/'tools/i1_nav_udp_only.xml'
        tree = ET.parse(profile)
        ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        if [x.text for x in tree.findall('.//d:transport_descriptor/d:type', ns)] != ['UDPv4']:
            raise RuntimeError('Unexpected capture transport; not UDP-only')
        args.backup.mkdir(parents=True, exist_ok=False)
        manifest = {'status': 'PREPARED', 'source_commit': args.source_commit, 'files': {}}
        for rel in BASE:
            old = args.backup/'original'/rel
            old.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.root/rel, old)
            if sha(old) != BASE[rel]:
                raise RuntimeError('Backup verification failed: '+rel)
            manifest['files'][rel] = {'before': BASE[rel], 'after': hashlib.sha256(payload[rel]).hexdigest()}
        shutil.copy2(profile, args.backup/'dds_udp_only.xml')
        manifest['profile_sha256'] = sha(profile)
        (args.backup/'deployment.json').write_text(json.dumps(manifest, indent=2)+'\n')
        try:
            no_stack()
            for rel in BASE:
                if sha(args.root/rel) != BASE[rel]:
                    raise RuntimeError('Robot file changed after backup: '+rel)
                replace(args.root/rel, payload[rel])
                if sha(args.root/rel) != manifest['files'][rel]['after']:
                    raise RuntimeError('Installed hash mismatch: '+rel)
            manifest['status'] = 'INSTALLED_NOT_STARTED'
        except Exception as exc:
            restore(args.root, args.backup, manifest)
            manifest.update(status='ROLLED_BACK_AFTER_INSTALL_ERROR', error=repr(exc))
            (args.backup/'deployment.json').write_text(json.dumps(manifest, indent=2)+'\n')
            raise
    (args.backup/'deployment.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
