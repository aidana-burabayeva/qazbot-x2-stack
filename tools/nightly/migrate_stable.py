#!/usr/bin/env python3
"""One-time migration to the two permanent branches (robot/stable, lab/stable). Idempotent; plain pushes only; nothing is deleted or reset.

  1 create robot/stable from the published robot baseline and lab/stable from the lab branch (only when absent - existing refs never move)
  2 integrate the verified lab history into lab/stable THROUGH THE MERGE QUEUE (merge in a throw-away worktree, regression gate, CAS)
  3 audit all remote branches, tag every non-permanent tip `archive/<branch>`, commit docs/BRANCH_MIGRATION.md through the queue
  4 push both stable branches and the archive tags, verify the remote SHAs
usage: migrate_stable.py --repo . --state DIR [--robot-base origin/baseline/x2-robot-20261030] [--lab-base origin/test/p20-twin02-sim-20261029]
       [--integrate BRANCH ...] [--gate "cmd"] [--no-push]
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def git(repo, *a, check=True):
    r = subprocess.run(['git', '-C', str(repo), '-c', 'user.name=qazbot-migrate', '-c', 'user.email=migrate@localhost', *a], capture_output=True, text=True)
    if check and r.returncode != 0:
        raise SystemExit(f'git {" ".join(a[:3])} failed: {r.stderr.strip()[-250:]}')
    return r.stdout.strip()


def rev(repo, ref):
    r = subprocess.run(['git', '-C', str(repo), 'rev-parse', '--verify', '-q', ref + '^{commit}'], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else None


def ensure_branch(repo, name, base):
    """Create refs/heads/<name> at <base> only if neither the local nor the remote branch exists."""
    if rev(repo, f'refs/heads/{name}') or rev(repo, f'refs/remotes/origin/{name}'):
        return 'exists', rev(repo, f'refs/heads/{name}') or rev(repo, f'refs/remotes/origin/{name}')
    b = rev(repo, base)
    if not b:
        raise SystemExit(f'base {base} not found')
    git(repo, 'branch', name, b)
    return 'created', b


def drain(nq, mq, db, limit=2400):
    for _ in range(limit):
        if not mq.step(db, nq.RUN, lambda t, e, d='': nq.ev(db, t, e, d)):
            return
        time.sleep(0.5)
    raise SystemExit('merge queue did not drain')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default='.'); ap.add_argument('--state', required=True)
    ap.add_argument('--robot-base', default='origin/baseline/x2-robot-20261030'); ap.add_argument('--lab-base', default='origin/test/p20-twin02-sim-20261029')
    ap.add_argument('--integrate', nargs='*', default=[]); ap.add_argument('--gate'); ap.add_argument('--no-push', action='store_true')
    a = ap.parse_args()
    repo = str(Path(a.repo).resolve())
    os.environ['NIGHTLY_ROOT'], os.environ['NIGHTLY_RUN'] = repo, str(Path(a.state).resolve())
    if a.gate:
        os.environ['NIGHTLY_INTEGRATION_GATE'] = a.gate
    sys.path.insert(0, str(HERE))
    import nq, mergeq as mq, branch_audit
    db = nq.connect()
    report = {'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    git(repo, 'fetch', '--prune', '-q')
    report['robot/stable'] = ensure_branch(repo, 'robot/stable', a.robot_base)
    report['lab/stable'] = ensure_branch(repo, 'lab/stable', a.lab_base)
    for b in a.integrate:
        sha = mq.enqueue_manual(db, repo, f'migrate_{b.replace("/", "_")}', b, 'migration of verified lab history')
        report.setdefault('queued', []).append((b, sha[:12]))
    drain(nq, mq, db)
    report['merge_queue'] = [dict(r) for r in db.execute('SELECT task,state,detail,substr(merge_sha,1,12) merge_sha FROM merge_queue ORDER BY id')]
    if any(r['state'] not in ('INTEGRATED',) for r in report['merge_queue']):
        print(json.dumps(report, indent=1)); raise SystemExit('migration stopped: a candidate did not integrate (kept, see merge_queue)')
    if not a.no_push:
        for b in ('robot/stable', 'lab/stable'):
            r = subprocess.run(['git', '-C', repo, 'push', 'origin', f'refs/heads/{b}:refs/heads/{b}'], capture_output=True, text=True)
            report[f'push {b}'] = 'ok' if r.returncode == 0 else r.stderr.strip()[-200:]
        git(repo, 'fetch', '--prune', '-q')
    # audit + archive tags (non-destructive) + docs through the queue
    rows = branch_audit.audit(repo)
    for r in rows:
        if r['branch'] not in branch_audit.STABLE:
            tag = f"archive/{r['branch'].replace('/', '-')}"
            if not rev(repo, f'refs/tags/{tag}'):
                git(repo, 'tag', tag, git(repo, 'rev-parse', f"origin/{r['branch']}"))
    docs_branch = f"docs/branch-migration-{time.strftime('%Y%m%d')}"
    if not rev(repo, f'refs/heads/{docs_branch}'):
        wt = Path(a.state) / 'docs_wt'
        git(repo, 'worktree', 'add', '-q', '-b', docs_branch, str(wt), 'lab/stable')
        (wt / 'docs').mkdir(exist_ok=True)
        (wt / 'docs' / 'BRANCH_MIGRATION.md').write_text(branch_audit.render_md(rows))
        (wt / 'docs' / 'BRANCH_MIGRATION.json').write_text(json.dumps(rows, indent=1))
        git(wt, 'add', '-A'); git(wt, 'commit', '-q', '-m', 'docs: branch migration table (audit of all remote branches)')
        git(repo, 'worktree', 'remove', '--force', str(wt))
        mq.enqueue_manual(db, repo, 'migrate_docs', docs_branch, 'branch migration table')
        drain(nq, mq, db)
    report['final'] = {b: (rev(repo, f'refs/heads/{b}') or '')[:12] for b in ('robot/stable', 'lab/stable')}
    if not a.no_push:
        for b in ('lab/stable',):
            r = subprocess.run(['git', '-C', repo, 'push', 'origin', f'refs/heads/{b}:refs/heads/{b}'], capture_output=True, text=True)
            report[f'push2 {b}'] = 'ok' if r.returncode == 0 else r.stderr.strip()[-200:]
        r = subprocess.run(['git', '-C', repo, 'push', 'origin', 'refs/tags/archive/*:refs/tags/archive/*'], capture_output=True, text=True)
        report['push tags'] = 'ok' if r.returncode == 0 else r.stderr.strip()[-200:]
        report['remote'] = {b: subprocess.run(['git', '-C', repo, 'ls-remote', 'origin', f'refs/heads/{b}'], capture_output=True, text=True).stdout.split()[:1] for b in ('robot/stable', 'lab/stable')}
    report['audit_counts'] = {c: sum(1 for r in rows if r['cls'] == c) for c in ('ACTIVE', 'INTEGRATED', 'SUPERSEDED', 'NEEDS_REVIEW', 'ARCHIVED')}
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
