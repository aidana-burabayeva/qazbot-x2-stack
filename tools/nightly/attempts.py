#!/usr/bin/env python3
"""Attempt isolation for engineering tasks (SIM ONLY, stdlib only).

Every engineering attempt gets its OWN git worktree + branch cut from a known SHA. Consequences:
 * single writer by construction: an old/unknown agent that wakes up later can only touch its old worktree;
 * a worktree is never reused for a new attempt (so "lease released" is never taken as proof that a process stopped);
 * the orchestrator, not the agent, commits the attempt and computes SHAs/hashes (the agent needs no shell for it);
 * nothing is merged anywhere here: a verified attempt is only an INTEGRATION_READY branch.
"""
import os
import re
import signal
import subprocess
import time
from pathlib import Path

IDENT = ['-c', 'user.name=qazbot-nightly', '-c', 'user.email=nightly@localhost']


def _git(cwd, *a, timeout=60):
    r = subprocess.run(['git', '-C', str(cwd), *a], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def is_git_repo(cwd):
    return _git(cwd, 'rev-parse', '--git-dir')[0] == 0 and _git(cwd, 'rev-parse', '--verify', 'HEAD')[0] == 0


def head(cwd):
    return _git(cwd, 'rev-parse', 'HEAD')[1]


def prepare(base_cwd, run_dir, name, attempt, attempt_id, base_sha):
    """New worktree + branch for one attempt. Never reuses an existing path."""
    safe = re.sub(r'[^A-Za-z0-9._-]', '_', name)
    wt = Path(run_dir) / 'attempts' / f'{safe}.{attempt}.{attempt_id[:6]}'
    if wt.exists():
        raise FileExistsError(f'attempt worktree already exists: {wt}')
    wt.parent.mkdir(parents=True, exist_ok=True)
    branch = f'attempt/{safe}-{attempt}-{attempt_id[:6]}'
    rc, out = _git(base_cwd, 'worktree', 'add', '-q', '-b', branch, str(wt), base_sha)
    if rc != 0:
        raise RuntimeError(f'worktree add failed: {out[-200:]}')
    excl = _git(wt, 'rev-parse', '--git-path', 'info/exclude')[1]
    exp = Path(excl) if os.path.isabs(excl) else wt / excl
    exp.parent.mkdir(parents=True, exist_ok=True)
    with open(exp, 'a') as f:
        f.write('\n.qazbot/\n')
    (wt / '.qazbot' / 'results').mkdir(parents=True, exist_ok=True)
    return str(wt), branch


def commit(wt, name, attempt):
    """Commit whatever the agent left (except .qazbot/). Returns (tip_sha, changed)."""
    before = head(wt)
    _git(wt, 'add', '-A')
    if _git(wt, 'diff', '--cached', '--quiet')[0] == 0:
        return before, False
    rc, out = _git(wt, *IDENT, 'commit', '-q', '-m', f'nightly attempt {name}#{attempt} (committed by the orchestrator, not by the agent)')
    return head(wt), rc == 0


def survivors(wt):
    """Any process whose cwd is inside the attempt worktree."""
    root = os.path.realpath(wt)
    found = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) == os.getpid():
            continue
        try:
            cwd = os.path.realpath(os.readlink(f'/proc/{d}/cwd'))
        except OSError:
            continue
        if cwd == root or cwd.startswith(root + os.sep):
            found.append(int(d))
    return found


def reap(wt, grace=1.0):
    """Kill leftovers of an ended attempt and report them. Absence of survivors is the only accepted proof that the writer is gone."""
    pids = survivors(wt)
    for p in pids:
        try:
            os.kill(p, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(grace if pids else 0)
    return pids, survivors(wt)


class VerifyJob:
    """Test re-run in a separate, niced process group with a wall-clock limit. The caller polls; nothing blocks the queue."""

    def __init__(self, argv, cwd, log, timeout, nice=10):
        self.deadline = time.time() + timeout
        self.log = log
        env = dict(os.environ, OMP_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
        fh = open(log, 'wb')
        self.p = subprocess.Popen(['nice', '-n', str(nice), *argv], cwd=cwd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True, env=env)

    def poll(self):
        """None while running; (ok, reason) when finished."""
        rc = self.p.poll()
        if rc is None:
            if time.time() > self.deadline:
                try:
                    os.killpg(self.p.pid, signal.SIGKILL)
                except OSError:
                    pass
                self.p.wait()
                return False, 'verification timeout'
            return None
        return rc == 0, f'tests rc={rc}'

    def kill(self):
        try:
            os.killpg(self.p.pid, signal.SIGKILL)
        except OSError:
            pass


def commit_components(wt, name, attempt, plans):
    """Commit explicitly disjoint D04/D07 groups in the existing private attempt only."""
    if not isinstance(plans,list) or not all(isinstance(p,dict) for p in plans):raise ValueError('invalid component plans')
    selected=[p for p in plans if p.get('blocker') in ('d04','d07') and not p.get('blocked') and p.get('files')]
    if len(selected)<2:return {}
    if len(selected)!=2 or {p['blocker'] for p in selected}!={'d04','d07'}:raise ValueError('exactly one D04 and one D07 group required')
    _git(wt,'add','-A')
    changed=set(_git(wt,'diff','--cached','--name-only')[1].splitlines())
    used=set()
    for plan in selected:
        paths=plan['files']
        if (not isinstance(paths,list) or not paths or not all(isinstance(p,str) and p in changed and not Path(p).is_absolute()
                and '..' not in Path(p).parts and not p.startswith('.qazbot/') for p in paths)
                or (used.intersection(paths) and not all(isinstance(p.get('patch'),str) and p['patch'] for p in selected))):
            raise ValueError('independent D04/D07 commit groups must be explicit, changed and disjoint')
        used.update(paths)
    _git(wt,'restore','--staged','--source=HEAD','--','.')
    results={}
    for plan in selected:
        before=head(wt)
        if plan.get('patch'):
            r=subprocess.run(['git','-C',str(wt),'apply','--cached','--whitespace=error','-'],input=plan['patch'],text=True,capture_output=True,timeout=60)
            rc,out=r.returncode,r.stdout+r.stderr
        else:
            rc,out=_git(wt,'--literal-pathspecs','add','--',*plan['files'])
        if rc:raise ValueError('cannot stage component group: '+out[-120:])
        staged=set(_git(wt,'diff','--cached','--name-only')[1].splitlines())
        if not staged or not staged.issubset(plan['files']):raise ValueError('component patch changes undeclared files')
        rc,out=_git(wt,*IDENT,'commit','-q','-m',f"nightly {name}#{attempt}: independent {plan['blocker']} fix")
        if rc:raise ValueError('cannot commit component group: '+out[-120:])
        results[plan['blocker']]=dict(fix_sha=head(wt),baseline_sha=before)
    return results
