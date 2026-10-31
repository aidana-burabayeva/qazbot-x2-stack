#!/usr/bin/env python3
"""Minimal merge queue for VERIFIED engineering attempts (SIM ONLY, stdlib only; no new CI platform).

Integrates INTEGRATION_READY attempt branches, one at a time, into the permanent lab branch (default `lab/stable`, never main / a
robot branch / a branch that is checked out anywhere). For each candidate:
  ownership   the attempt branch tip must still be the SHA that was verified, the task must be COMPLETED+VERIFIED with a stored PASS result;
  merge       `git merge --no-ff` in a throw-away detached worktree cut from the CURRENT integration head (no force, no reset, no rebase);
  gate        the regression command is re-run on the MERGED tree (non-blocking job);
  publish     compare-and-swap `git update-ref <branch> <new> <old>`: if the integration head moved meanwhile the update is refused and the
              candidate is re-gated on the new head (so a stale gate result can never be integrated);
  state       run/nightly/integration_state.json is replaced atomically after every transition.
Conflicts and failed gates keep the branch and evidence, classify the candidate (CONFLICT / REGRESSION_FAILED / STALE) and the queue goes on.
"""
import json
import os
import shlex
import shutil
import subprocess
import time
from pathlib import Path

import attempts
import candidates

SCHEMA = """CREATE TABLE IF NOT EXISTS merge_queue(
  id INTEGER PRIMARY KEY, task TEXT NOT NULL, repo TEXT NOT NULL, branch TEXT NOT NULL, sha TEXT NOT NULL, base_sha TEXT,
  state TEXT NOT NULL, detail TEXT, enq_ts REAL, ts REAL, merge_sha TEXT, wt TEXT, UNIQUE(task, sha))"""
JOBS = {}
IDENT = ['-c', 'user.name=qazbot-nightly', '-c', 'user.email=nightly@localhost']
FORBIDDEN_BRANCHES = ('main', 'master')


def init(db):
    db.execute(SCHEMA)


def branch_name():
    b = os.environ.get('NIGHTLY_INTEGRATION_BRANCH', 'lab/stable')
    if not b.startswith('lab/') or b in FORBIDDEN_BRANCHES:
        raise SystemExit(f'refused: integration branch must be under lab/ (got {b}); production and robot branches are never touched')
    return b


def gate_argv():
    return shlex.split(os.environ.get('NIGHTLY_INTEGRATION_GATE', 'python3 -m pytest -q tests/nightly'))


def git(repo, *a, timeout=120):
    r = subprocess.run(['git', '-C', str(repo), *a], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def rev(repo, ref):
    rc, o = git(repo, 'rev-parse', '--verify', '-q', ref)
    return o if rc == 0 else None


def checked_out_at(repo, ref):
    """Worktree path that has `ref` (refs/heads/...) checked out, or None. update-ref on a checked-out branch moves HEAD but not the tree."""
    out = git(repo, 'worktree', 'list', '--porcelain')[1]
    path = None
    for ln in out.splitlines():
        if ln.startswith('worktree '):
            path = ln[9:]
        elif ln.startswith('branch ') and ln[7:] == ref:
            return path
    return None


def integration_head(repo):
    """Fresh integration SHA that attempts must start from (the branch ref, never the control checkout's HEAD); HEAD only if the branch does not exist yet."""
    return rev(repo, f'refs/heads/{branch_name()}') or rev(repo, 'HEAD')


def ensure_branch(repo):
    b = branch_name()
    if rev(repo, f'refs/heads/{b}'):
        return b
    base = os.environ.get('NIGHTLY_INTEGRATION_BASE') or rev(repo, 'HEAD')
    git(repo, 'branch', b, base)  # a new ref, never a move of an existing one
    return b


def state_file(run):
    return Path(run) / 'integration_state.json'


def write_state(db, run, repo):
    rows = [dict(r) for r in db.execute('SELECT task,sha,state,detail,merge_sha,ts FROM merge_queue ORDER BY id')]
    b = branch_name()
    st = dict(branch=b, head=rev(repo, f'refs/heads/{b}'), repo=str(repo), updated=time.time(), items=rows)
    tmp = state_file(run).with_suffix('.tmp')
    tmp.write_text(json.dumps(st, indent=1))
    os.replace(tmp, state_file(run))  # atomic


def enqueue(db, run, kv_get, task):
    """Only a COMPLETED + VERIFIED task whose verified SHA is still its branch tip with a stored PASS result may be queued."""
    if os.environ.get('NIGHTLY_MERGEQ', '1') == '0' or not task['branch'] or not task['attempt_wt']:
        return None
    sha = kv_get(db, f"verified_sha:{task['name']}")
    ok = db.execute("SELECT 1 FROM results WHERE task=? AND attempt=? AND verdict='PASS' AND status='STORED' LIMIT 1", (task['name'], task['attempts'])).fetchone()
    if not sha or not ok or task['state'] != 'COMPLETED' or task['phase'] != 'VERIFIED':
        return None
    base = (kv_get(db, f"base:{task['name']}:{task['attempts']}", {}) or {}).get('git_sha')
    try:
        db.execute("INSERT OR IGNORE INTO merge_queue(task,repo,branch,sha,base_sha,state,enq_ts,ts) VALUES(?,?,?,?,?,'QUEUED',?,?)",
                   (task['name'], task['cwd'], task['branch'], sha, base, time.time(), time.time()))
    except Exception:
        return None
    return sha


def enqueue_manual(db, repo, name, branch, note='operator migration'):
    """Operator-driven integration of an existing branch (migration). Still merged in a throw-away worktree, gated and CAS-published;
    no candidate is created (there is no stored attempt result)."""
    sha = rev(repo, f'refs/heads/{branch}')
    if not sha:
        raise SystemExit(f'unknown branch {branch}')
    db.execute("INSERT OR IGNORE INTO merge_queue(task,repo,branch,sha,base_sha,state,detail,enq_ts,ts) VALUES(?,?,?,?,?,'QUEUED',?,?,?)", (name, str(repo), branch, sha, None, note, time.time(), time.time()))
    return sha


def _set(db, mid, state, detail=None, **kw):
    cols = {'state': state, 'detail': detail, 'ts': time.time(), **kw}
    db.execute(f"UPDATE merge_queue SET {', '.join(k + '=?' for k in cols)} WHERE id=?", (*cols.values(), mid))


def _cleanup(repo, wt):
    if wt and Path(wt).exists():
        git(repo, 'worktree', 'remove', '--force', wt)
        shutil.rmtree(wt, ignore_errors=True)
        git(repo, 'worktree', 'prune')


def step(db, run, ev=None):
    """One non-blocking step. Returns True while there is still work (QUEUED or GATING rows)."""
    ev = ev or (lambda *a: None)
    gating = db.execute("SELECT * FROM merge_queue WHERE state='GATING'").fetchall()
    for row in gating:
        job = JOBS.get(row['id'])
        if job is None:  # orchestrator restarted: drop the half-done attempt and gate again from the current head
            _cleanup(row['repo'], row['wt'])
            _set(db, row['id'], 'QUEUED', 'orchestrator restarted during gate: re-gating', wt=None)
            continue
        out = job.poll()
        if out is None:
            continue
        JOBS.pop(row['id'], None)
        ok, why = out
        repo, b = row['repo'], branch_name()
        mlog = Path(run) / 'merge' / f"{row['id']}.gate.log"
        mlog.parent.mkdir(parents=True, exist_ok=True)
        if Path(job.log).exists():
            shutil.copy2(job.log, mlog)
        if not ok:
            _cleanup(repo, row['wt'])
            _set(db, row['id'], 'REGRESSION_FAILED', f'gate failed after merge ({why}); branch and log kept: {mlog}', wt=None)
            ev(row['task'], 'MERGE_REGRESSION_FAILED', why)
        else:
            merge_sha = attempts.head(row['wt'])
            old = row['merge_sha']  # integration head the merge was built on (stored when gating started)
            co = checked_out_at(repo, f'refs/heads/{b}')
            if co:  # a checkout of the integration branch would be left with a stale tree/index: refuse, keep the gated merge for a retry
                _cleanup(repo, row['wt'])
                _set(db, row['id'], 'QUEUED', f'{b} is checked out in {co}: use a separate control checkout (lab/control); retry', wt=None)
                ev(row['task'], 'MERGE_REFUSED_CHECKED_OUT', co)
                write_state(db, run, repo)
                continue
            rc, o = git(repo, 'update-ref', '-m', f"integrate {row['task']}", f'refs/heads/{b}', merge_sha, old)
            _cleanup(repo, row['wt'])
            if rc != 0:  # head moved while the gate was running: the gate result is stale, never integrate it
                _set(db, row['id'], 'QUEUED', 'integration head moved during gate (CAS refused): re-gating', wt=None)
                ev(row['task'], 'MERGE_CAS_REFUSED', o[-120:])
            else:
                _set(db, row['id'], 'INTEGRATED', f'merged as {merge_sha[:12]}', merge_sha=merge_sha, wt=None)
                ev(row['task'], 'INTEGRATED', f'{b} -> {merge_sha}')
        write_state(db, run, repo)
    if db.execute("SELECT 1 FROM merge_queue WHERE state='GATING' LIMIT 1").fetchone():
        return True  # strictly one candidate at a time
    row = db.execute("SELECT * FROM merge_queue WHERE state='QUEUED' ORDER BY id LIMIT 1").fetchone()
    if row is None:
        return False
    repo, b = row['repo'], ensure_branch(row['repo'])
    tip = rev(repo, f"refs/heads/{row['branch']}")
    if tip != row['sha']:
        _set(db, row['id'], 'STALE', f"branch tip {str(tip)[:12]} != verified sha {row['sha'][:12]}: changed after verification, not integrated")
        ev(row['task'], 'MERGE_STALE', 'branch moved after verification')
        write_state(db, run, repo)
        return True
    if row['base_sha'] and git(repo, 'merge-base', '--is-ancestor', row['base_sha'], row['sha'])[0] != 0:
        _set(db, row['id'], 'STALE', 'verified sha does not descend from its recorded base')
        write_state(db, run, repo)
        return True
    head = rev(repo, f'refs/heads/{b}')
    if git(repo, 'merge-base', '--is-ancestor', row['sha'], head)[0] == 0:
        _set(db, row['id'], 'INTEGRATED', 'already contained in the integration branch', merge_sha=head)
        write_state(db, run, repo)
        return True
    wt = Path(run) / 'merge' / f"wt.{row['id']}.{int(time.time())}"
    wt.parent.mkdir(parents=True, exist_ok=True)
    rc, o = git(repo, 'worktree', 'add', '--detach', '-q', str(wt), head)
    if rc != 0:
        _set(db, row['id'], 'QUEUED', f'cannot prepare merge worktree: {o[-120:]}')
        return True
    rc, o = git(wt, *IDENT, 'merge', '--no-ff', '--no-edit', '-m', f"integrate {row['task']} ({row['sha'][:12]}) into {b}", row['sha'])
    if rc != 0:
        files = git(wt, 'diff', '--name-only', '--diff-filter=U')[1].split()
        git(wt, 'merge', '--abort')
        _cleanup(repo, str(wt))
        _set(db, row['id'], 'CONFLICT', f"merge conflict in {files}; branch {row['branch']} kept for a dedicated integration task", wt=None)
        ev(row['task'], 'MERGE_CONFLICT', json.dumps(files))
        write_state(db, run, repo)
        return True
    try:  # X2-bound candidate record (production files only); sim-only fixes produce none
        base = row['base_sha'] or git(repo, 'merge-base', head, row['sha'])[1]
        ent = candidates.build(db, row['task'], wt, base, row['sha'], gate=' '.join(gate_argv()))
        if ent and candidates.add(wt, ent):
            git(wt, 'add', candidates.FILE)
            git(wt, *IDENT, 'commit', '-q', '-m', f"candidates: {ent['id']} (SIM_VERIFIED, awaiting hardware validation)")
            ev(row['task'], 'CANDIDATE_RECORDED', ent['id'])
    except ValueError as e:  # a production fix lacking mandatory data is integrated for the lab but NOT offered to the robot
        ev(row['task'], 'CANDIDATE_REFUSED', str(e))
    log = Path(run) / 'merge' / f"{row['id']}.gate.log.work"
    JOBS[row['id']] = attempts.VerifyJob(gate_argv(), str(wt), str(log), float(os.environ.get('NIGHTLY_INTEGRATION_GATE_TIMEOUT_S', 1800)),
                                         int(os.environ.get('NIGHTLY_VERIFY_NICE', 10)))
    _set(db, row['id'], 'GATING', f'merged onto {head[:12]}; gate running', merge_sha=head, wt=str(wt))
    write_state(db, run, repo)
    return True


def publish(repo):
    """Plain push of the integration branch (no force). Separate, explicit step."""
    b = branch_name()
    rc, o = git(repo, 'push', 'origin', f'refs/heads/{b}:refs/heads/{b}', timeout=180)
    return rc == 0, o[-200:]
