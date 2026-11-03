import hashlib, json, os, pathlib, subprocess, datetime

home = pathlib.Path('/home/agi')
deploy = home / 'control_backend'
repo = deploy / 'qazbot-x2-stack'
out = pathlib.Path('/tmp/x2_preservation_20261103')
out.mkdir(exist_ok=True)

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''):
            h.update(b)
    return h.hexdigest()

roots = ['control_backend/ws/src', 'control_backend/tools', 'control_backend/tests',
         'control_backend/candidates', 'control_backend/candidates_v13',
         'control_backend/backups', 'control_backend/run', 'control_backend/runtime',
         'control_backend/vendor_snapshots', 'x2_slam_stage1', 'qazbot-speech',
         'aimdk', 'linkcraft_player.py', '.orchestrator']
rows = []
for root in roots:
    p = home / root
    paths = [p] if p.is_file() else sorted(p.rglob('*'))
    for f in paths:
        if '.git' in f.parts or not (f.is_file() or f.is_symlink()):
            continue
        try:
            row = {'path': str(f.relative_to(home)), 'size': f.stat().st_size,
                   'sha256': digest(f), 'mode': oct(f.stat().st_mode & 0o777)}
            if f.is_symlink():
                row['symlink'] = os.readlink(f)
            rows.append(row)
        except (OSError, RuntimeError) as e:
            rows.append({'path': str(f.relative_to(home)), 'error': str(e)})
(out / 'files.json').write_text(json.dumps(rows, indent=2))

comparisons = {}
for branch in ['robot/wip-20261103', 'robot/feedback-d04-d05-20261102']:
    hashes = {}
    tracked = subprocess.check_output(['git', '-C', str(repo), 'ls-tree', '-r', branch], text=True)
    for line in tracked.splitlines():
        meta, path = line.split('\t', 1)
        mode, typ, oid = meta.split()
        if typ == 'blob':
            hashes[path] = oid
    result = {'identical': [], 'different': [], 'absent_from_git': []}
    for row in rows:
        path = row['path']
        if not path.startswith('control_backend/') or 'sha256' not in row:
            continue
        rel = path[len('control_backend/'):]
        if not rel.startswith(('ws/src/', 'tools/', 'tests/')):
            continue
        if '__pycache__' in pathlib.Path(rel).parts or rel.endswith('.pyc'):
            continue
        if rel not in hashes:
            result['absent_from_git'].append(rel)
        else:
            oid = subprocess.check_output(['git', '-C', str(repo), 'hash-object', str(home/path)], text=True).strip()
            result['identical' if oid == hashes[rel] else 'different'].append(rel)
    comparisons[branch] = result
(out/'comparison.json').write_text(json.dumps(comparisons, indent=2))
summary = {b: {k:len(v) for k,v in r.items()} for b,r in comparisons.items()}
(out/'summary.json').write_text(json.dumps({'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(), 'files':len(rows), 'comparisons':summary}, indent=2))
print((out/'summary.json').read_text())
