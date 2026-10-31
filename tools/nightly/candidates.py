#!/usr/bin/env python3
"""X2-bound candidates recorded by the merge queue in docs/hardware/CANDIDATES.json on lab/stable (SIM ONLY, stdlib only).

A candidate exists only for a VERIFIED engineering attempt that changed PRODUCTION files (ws/src/**, never the simulator, tests or lab tools).
Sim-only fixes are integrated into lab/stable but never offered to the robot. Each entry carries what `/robot-sync-validate` needs:
task id, root cause, base/result SHA, production files, SIM_VERIFIED evidence, regression result, dependencies, hardware validation plan, rollback.
"""
import json
import subprocess
import time
from pathlib import Path

FILE = 'docs/hardware/CANDIDATES.json'
PRODUCTION_PREFIX = 'ws/src/'
NOT_PRODUCTION = ('ws/src/maps/main_office/',)      # simulator-only map variant
FORBIDDEN = ('digital_twin/', 'tests/', 'tools/nightly/', 'run/')


def _git(wt, *a):
    r = subprocess.run(['git', '-C', str(wt), *a], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else ''


def _blob_sha(wt, ref, path):
    import hashlib
    return hashlib.sha256(subprocess.run(['git', '-C', str(wt), 'show', f'{ref}:{path}'], capture_output=True).stdout).hexdigest()


def production_files(wt, base, sha):
    paths = _git(wt, 'diff', '--name-only', f'{base}..{sha}').split()
    return [p for p in paths if p.startswith(PRODUCTION_PREFIX) and not p.startswith(NOT_PRODUCTION) and not p.startswith(FORBIDDEN)]


def plan_for(files):
    areas = sorted({p.split('/')[2] for p in files if p.count('/') >= 2})
    return ('Operator approval and a clear area are REQUIRED before any step that moves the robot or restarts a service. '
            f'1) Read-only: record installed SHA256 of {len(files)} file(s), robot mode, dirty state, running parameters (areas: {", ".join(areas)}). '
            '2) Deploy ONLY the listed files from the candidate branch, one subsystem at a time, with the rollback below prepared. '
            '3) Re-run the original reproduction and the regression checks of this candidate; compare with the simulation prediction; PASS needs measured evidence. '
            '4) Report HW_VERIFIED with the installed-state SHA256 manifest, or HW_FAILED with evidence.')


def build(db, task_name, wt, base_sha, result_sha, merge_sha=None, gate=''):
    """Entry dict, or None if the attempt touches no production file. Raises ValueError when a production fix lacks mandatory data."""
    files = production_files(wt, base_sha, result_sha)
    if not files:
        return None
    t = db.execute('SELECT * FROM tasks WHERE name=?', (task_name,)).fetchone()
    res = db.execute("SELECT evidence_path, manifest_sha256, verdict, metrics FROM results WHERE task=? AND verdict='PASS' AND status='STORED' ORDER BY id DESC LIMIT 1", (task_name,)).fetchone()
    rc = db.execute("SELECT key, summary, status, evidence FROM root_causes WHERE task=? AND status='VERIFIED' ORDER BY ts DESC LIMIT 1", (task_name,)).fetchone()
    if not res:
        raise ValueError('no stored PASS result: not a SIM_VERIFIED production fix')
    if not rc:
        raise ValueError('production fix without a VERIFIED root cause (record it with `nq.py root-cause --status VERIFIED`)')
    metrics = json.loads(res['metrics'] or '{}')
    return {
        'id': f'cand-{task_name}-{result_sha[:7]}', 'task_id': task_name, 'title': (t['repro'] or task_name)[:120] if t else task_name,
        'root_cause': {'key': rc['key'], 'summary': rc['summary'], 'evidence': rc['evidence']},
        'base_sha': base_sha, 'result_sha': result_sha, 'merge_sha': merge_sha,
        'production_files': [{'path': p, 'sha256_after': _blob_sha(wt, result_sha, p)} for p in files],
        'sim_verified': {'verdict': res['verdict'], 'evidence_path': res['evidence_path'], 'manifest_sha256': res['manifest_sha256'], 'tests_cmd': metrics.get('tests_cmd')},
        'regression': {'gate': gate, 'result': 'PASS'},
        'dependencies': [],
        'hardware_validation_plan': plan_for(files),
        'rollback': {'procedure': 'Restore each listed file from robot/stable (the version recorded as sha256_before when the candidate branch was prepared), restart only the owning service with operator approval, re-verify SHA256.',
                     'restore_from': 'robot/stable'},
        'status': 'SIM_VERIFIED', 'created': time.strftime('%Y-%m-%dT%H:%M:%S%z')}


def add(wt, entry):
    """Append to docs/hardware/CANDIDATES.json inside the (throw-away) merge worktree; dependencies = earlier candidates touching the same files."""
    p = Path(wt) / FILE
    data = json.loads(p.read_text()) if p.exists() else {'schema': 1, 'candidates': []}
    if any(c['id'] == entry['id'] for c in data['candidates']):
        return False
    mine = {f['path'] for f in entry['production_files']}
    entry['dependencies'] = [c['id'] for c in data['candidates'] if mine & {f['path'] for f in c['production_files']}]
    data['candidates'].append(entry)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=1) + '\n')
    return True
