#!/usr/bin/env python3
"""Robot-side Git workflow for the two permanent branches (stdlib only, self-contained so it works in a robot checkout without the lab tools).

  robot/stable   what is actually INSTALLED on the X2 (+ hardware evidence + open blockers). Moves only by compare-and-swap, never by force.
  lab/stable     what the autonomous lab has verified in simulation (SIM_VERIFIED). Its docs/hardware/CANDIDATES.json lists X2-bound fixes.

  list      candidates in origin/lab/stable that the robot has not handled yet
  make      robot/candidate/<id> cut from robot/stable carrying ONLY the production files of one candidate (no sim runtime, no test stubs)
  result    record a hardware outcome for a candidate: HW_VERIFIED integrates the verified files into robot/stable (after the installed-state check);
            HW_FAILED only records the failure + evidence (robot/stable files unchanged, the failure stays in lab history)
  snapshot  integrate a captured installed-state snapshot (integration branch) into robot/stable after SHA256 + forbidden-path checks

No command here talks to the robot, deploys, restarts anything or moves it. They only read/write Git.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROBOT_STABLE, LAB_STABLE = 'robot/stable', 'lab/stable'
CANDIDATES_FILE = 'docs/hardware/CANDIDATES.json'
RESULTS_DIR = 'docs/hardware/candidate_results'
PRODUCTION_PREFIX = 'ws/src/'
# never part of anything bound for the X2: simulator, test stubs, lab orchestration, local runtime
FORBIDDEN = ('digital_twin/', 'tests/', 'tools/nightly/', 'run/', '.claude/worktrees/')
FORBIDDEN_SUFFIX = ('.mcap', '.db3', '.sqlite', '.db', '.log')
SNAPSHOT_ALLOWED = ('ws/src/', 'docs/', 'tools/twin/', 'tools/operator_skills/', '.claude/skills/')
REQUIRED = ('id', 'task_id', 'root_cause', 'base_sha', 'result_sha', 'production_files', 'sim_verified', 'regression', 'dependencies', 'hardware_validation_plan', 'rollback')


class StableError(Exception):
    pass


def git(repo, *a, check=True, text=True, input=None):
    r = subprocess.run(['git', '-C', str(repo), '-c', 'user.name=qazbot-stable', '-c', 'user.email=stable@localhost', *a], capture_output=True, text=text, input=input)
    if check and r.returncode != 0:
        raise StableError(f"git {' '.join(a[:3])}: {(r.stderr or r.stdout).strip()[-300:]}")
    return r.stdout.strip() if text else r.stdout


def rev(repo, ref):
    r = subprocess.run(['git', '-C', str(repo), 'rev-parse', '--verify', '-q', ref + '^{commit}'], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else None


def blob_sha256(repo, ref, path):
    r = subprocess.run(['git', '-C', str(repo), 'show', f'{ref}:{path}'], capture_output=True)
    return hashlib.sha256(r.stdout).hexdigest() if r.returncode == 0 else None


def forbidden(path):
    return path.startswith(FORBIDDEN) or path.endswith(FORBIDDEN_SUFFIX) or '/__pycache__/' in path


def cas(repo, branch, new, old):
    """compare-and-swap of refs/heads/<branch>; refused if the branch moved. No force."""
    try:
        git(repo, 'update-ref', '-m', f'stable: {branch}', f'refs/heads/{branch}', new, old)
    except StableError as e:
        raise StableError(f'{branch} moved during the operation (CAS refused): {e}')


def load_candidates(repo, lab_ref=f'origin/{LAB_STABLE}'):
    r = subprocess.run(['git', '-C', str(repo), 'show', f'{lab_ref}:{CANDIDATES_FILE}'], capture_output=True, text=True)
    return json.loads(r.stdout).get('candidates', []) if r.returncode == 0 else []


def load_results(repo, robot_ref=f'origin/{ROBOT_STABLE}'):
    out = {}
    rc = subprocess.run(['git', '-C', str(repo), 'ls-tree', '-r', '--name-only', robot_ref, RESULTS_DIR], capture_output=True, text=True)
    for p in rc.stdout.split():
        d = json.loads(git(repo, 'show', f'{robot_ref}:{p}'))
        out[d['id']] = d
    return out


def status_of(c, results):
    r = results.get(c['id'])
    return r['status'] if r else c.get('status', 'SIM_VERIFIED')


def list_candidates(repo, lab_ref=f'origin/{LAB_STABLE}', robot_ref=f'origin/{ROBOT_STABLE}'):
    res = load_results(repo, robot_ref)
    return [dict(c, effective_status=status_of(c, res)) for c in load_candidates(repo, lab_ref)]


def validate_entry(c):
    miss = [k for k in REQUIRED if c.get(k) in (None, '', [])]
    miss = [k for k in miss if k != 'dependencies']
    if miss:
        raise StableError(f"candidate {c.get('id')} incomplete, missing: {miss}")
    files = [f['path'] if isinstance(f, dict) else f for f in c['production_files']]
    bad = [f for f in files if not f.startswith(PRODUCTION_PREFIX) or forbidden(f)]
    if bad:
        raise StableError(f"candidate {c['id']} lists non-production or simulator files: {bad}")
    if not c['sim_verified'].get('evidence_path') or c['sim_verified'].get('verdict') != 'PASS':
        raise StableError(f"candidate {c['id']} has no PASS simulation evidence")
    return files


def make_candidate(repo, cid, lab_ref=f'origin/{LAB_STABLE}', robot_ref=f'origin/{ROBOT_STABLE}'):
    cands = {c['id']: c for c in load_candidates(repo, lab_ref)}
    if cid not in cands:
        raise StableError(f'unknown candidate {cid}')
    c = cands[cid]
    files = validate_entry(c)
    results = load_results(repo, robot_ref)
    for dep in c.get('dependencies', []):
        if status_of(cands.get(dep, {'id': dep}), results) != 'HW_VERIFIED':
            raise StableError(f'dependency {dep} is not HW_VERIFIED: candidate {cid} cannot be prepared yet')
    base = rev(repo, robot_ref)
    if not base:
        raise StableError(f'{robot_ref} not found')
    if not rev(repo, c['result_sha']):
        raise StableError(f"result_sha {c['result_sha'][:12]} is not available locally: fetch {lab_ref}")
    branch = f'robot/candidate/{cid}'
    if rev(repo, f'refs/heads/{branch}'):
        raise StableError(f'{branch} already exists (never overwritten)')
    wt = tempfile.mkdtemp(prefix='cand_')
    try:
        git(repo, 'worktree', 'add', '-q', '-b', branch, wt, base)
        rollback = []
        for f in files:
            before = blob_sha256(repo, base, f)
            rollback.append({'path': f, 'sha256_before': before})
            git(wt, 'checkout', c['result_sha'], '--', f)
        plan = dict(c, status='HW_CANDIDATE', robot_base_sha=base, rollback_files=rollback, prepared=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        p = Path(wt) / 'docs' / 'hardware' / 'candidates' / f'{cid}.json'
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(plan, indent=1))
        git(wt, 'add', '-A')
        staged = git(wt, 'diff', '--cached', '--name-only').split()
        bad = [s for s in staged if not (s.startswith(PRODUCTION_PREFIX) or s.startswith('docs/hardware/candidates/')) or forbidden(s)]
        if bad:
            raise StableError(f'candidate would carry non-production files: {bad}')
        git(wt, 'commit', '-q', '-m', f'candidate {cid}: {c.get("title", c["task_id"])} (SIM_VERIFIED, awaiting hardware validation)')
        sha = git(wt, 'rev-parse', 'HEAD')
    except Exception:
        git(repo, 'worktree', 'remove', '--force', wt, check=False)
        git(repo, 'branch', '-D', branch, check=False)  # only the branch THIS call just created
        raise
    git(repo, 'worktree', 'remove', '--force', wt, check=False)
    return dict(branch=branch, sha=sha, files=files, robot_base=base)


def integrate_branch_into_stable(repo, source, robot_branch=ROBOT_STABLE):
    """ff when possible, else a no-ff merge in a throw-away worktree; then CAS."""
    head = rev(repo, f'refs/heads/{robot_branch}')
    tip = rev(repo, source)
    if not head or not tip:
        raise StableError('missing branch')
    if git(repo, 'merge-base', '--is-ancestor', head, tip, check=False) == '' and subprocess.run(['git', '-C', str(repo), 'merge-base', '--is-ancestor', head, tip]).returncode == 0:
        cas(repo, robot_branch, tip, head)
        return tip
    wt = tempfile.mkdtemp(prefix='stable_')
    try:
        git(repo, 'worktree', 'add', '--detach', '-q', wt, head)
        try:
            git(wt, 'merge', '--no-ff', '--no-edit', '-m', f'integrate {source} into {robot_branch}', tip)
        except StableError:
            git(wt, 'merge', '--abort', check=False)
            raise StableError(f'merge conflict integrating {source}: kept for a dedicated integration task')
        new = git(wt, 'rev-parse', 'HEAD')
    finally:
        git(repo, 'worktree', 'remove', '--force', wt, check=False)
    cas(repo, robot_branch, new, head)
    return new


def record_result(repo, cid, status, evidence, installed_manifest=None, lab_ref=f'origin/{LAB_STABLE}'):
    """HW_VERIFIED needs the installed-state manifest (path -> sha256 measured on the X2 AFTER the permitted deployment) to equal the candidate files."""
    if status not in ('HW_VERIFIED', 'HW_FAILED'):
        raise StableError('status must be HW_VERIFIED or HW_FAILED')
    if not evidence:
        raise StableError('hardware evidence reference is required')
    branch = f'robot/candidate/{cid}'
    cand = rev(repo, f'refs/heads/{branch}')
    if not cand:
        raise StableError(f'{branch} does not exist: prepare the candidate first')
    plan = json.loads(git(repo, 'show', f'{cand}:docs/hardware/candidates/{cid}.json'))
    record = {'id': cid, 'status': status, 'evidence': evidence, 'candidate_branch': branch, 'candidate_sha': cand, 'recorded': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
              'task_id': plan['task_id'], 'result_sha': plan['result_sha']}
    head = rev(repo, f'refs/heads/{ROBOT_STABLE}')
    wt = tempfile.mkdtemp(prefix='res_')
    try:
        git(repo, 'worktree', 'add', '--detach', '-q', wt, head)
        if status == 'HW_VERIFIED':
            if not installed_manifest:
                raise StableError('HW_VERIFIED requires the installed-state manifest (path -> sha256 measured on the robot after deployment)')
            files = [f['path'] for f in plan['rollback_files']]
            wrong = {f: (installed_manifest.get(f), blob_sha256(repo, cand, f)) for f in files if installed_manifest.get(f) != blob_sha256(repo, cand, f)}
            if wrong:
                raise StableError(f'installed state does not match the candidate (not verified): {wrong}')
            git(wt, 'merge', '--no-ff', '--no-edit', '-m', f'hardware verified: candidate {cid}', cand)
            record['installed_sha256'] = {f: installed_manifest[f] for f in files}
        p = Path(wt) / RESULTS_DIR / f'{cid}.json'
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(record, indent=1))
        git(wt, 'add', '-A')
        git(wt, 'commit', '-q', '-m', f'candidate {cid}: {status}')
        new = git(wt, 'rev-parse', 'HEAD')
    finally:
        git(repo, 'worktree', 'remove', '--force', wt, check=False)
    cas(repo, ROBOT_STABLE, new, head)
    return new


def diff_manifests(prev, new):
    """SHA256 comparison with the previous robot baseline. A file absent from the new manifest is `removed` ONLY when the new manifest declares
    scope=="full"; a partial manifest (e.g. tooling only) reports it as `not_covered` - it says nothing about the robot's files."""
    a, b = {f['path']: f['sha256'] for f in prev['files']}, {f['path']: f['sha256'] for f in new['files']}
    gone = sorted(set(a) - set(b))
    full = new.get('scope') == 'full'
    return {'added': sorted(set(b) - set(a)), 'removed': gone if full else [], 'not_covered': [] if full else gone, 'changed': sorted(p for p in set(a) & set(b) if a[p] != b[p]), 'scope': new.get('scope', 'partial')}


def integrate_snapshot(repo, integration_branch, manifest_path):
    """The robot-side session captured the installed state into `integration_branch`; verify then move robot/stable (ff-only or merge, CAS)."""
    manifest = json.loads(Path(manifest_path).read_text())
    tip = rev(repo, integration_branch)
    if not tip:
        raise StableError(f'{integration_branch} not found')
    base = rev(repo, f'refs/heads/{ROBOT_STABLE}')
    changed = git(repo, 'diff', '--name-only', base, tip).split() if base else []
    bad = [p for p in changed if forbidden(p) or not p.startswith(SNAPSHOT_ALLOWED)]
    if bad:
        raise StableError(f'snapshot branch carries files that do not belong on robot/stable: {bad[:8]}')
    mism = [f['path'] for f in manifest['files'] if blob_sha256(repo, tip, f['path']) != f['sha256']]
    if mism:
        raise StableError(f'{len(mism)} installed file(s) do not match the snapshot branch SHA256: {mism[:5]}')
    prev = None
    r = subprocess.run(['git', '-C', str(repo), 'show', f'{base}:docs/evidence/x2_twin_20261030/robot_baseline_manifest.json'], capture_output=True, text=True)
    if r.returncode == 0:
        prev = json.loads(r.stdout)
    new = integrate_branch_into_stable(repo, integration_branch)
    return dict(sha=new, verified_files=len(manifest['files']), diff_vs_previous=diff_manifests(prev, manifest) if prev else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--repo', default='.')
    sp = ap.add_subparsers(dest='c', required=True)
    sp.add_parser('list')
    m = sp.add_parser('make'); m.add_argument('id')
    r = sp.add_parser('result'); r.add_argument('id'); r.add_argument('status'); r.add_argument('--evidence', required=True); r.add_argument('--installed-manifest')
    s = sp.add_parser('snapshot'); s.add_argument('branch'); s.add_argument('--manifest', required=True)
    a = ap.parse_args()
    try:
        if a.c == 'list':
            print(json.dumps([{k: c[k] for k in ('id', 'task_id', 'effective_status')} for c in list_candidates(a.repo)], indent=1))
        elif a.c == 'make':
            print(json.dumps(make_candidate(a.repo, a.id), indent=1))
        elif a.c == 'result':
            im = json.loads(Path(a.installed_manifest).read_text()) if a.installed_manifest else None
            print(record_result(a.repo, a.id, a.status, a.evidence, im))
        else:
            print(json.dumps(integrate_snapshot(a.repo, a.branch, a.manifest), indent=1))
    except StableError as e:
        sys.exit(f'refused: {e}')


if __name__ == '__main__':
    main()
