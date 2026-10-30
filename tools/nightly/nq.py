#!/usr/bin/env python3
"""QazBot nightly: persistent task queue + orchestrator (SIM ONLY, stdlib only).

Infrastructure, not a navigation hot path: it never talks to ROS and never touches the physical X2.
State lives in SQLite (WAL) under run/nightly/, so a dead terminal, a dead AI session or a restart of
this process loses nothing. `daemon` is the long-running mode (systemd --user); everything else is a CLI.

States: PENDING RUNNING COMPLETED FAILED INTERRUPTED NEEDS_ENGINEERING NEEDS_APPROVAL NEEDS_HARDWARE PAUSED_AGENT
Kinds : mechanical (shell command, no AI needed) | engineering (needs an agent: codex/claude CLI)
"""
import argparse
import fcntl
import hashlib
import json
import os
import re
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import store  # noqa: E402
import agents  # noqa: E402
import attempts  # noqa: E402
import router  # noqa: E402
import mergeq  # noqa: E402

ROOT = Path(os.environ.get('NIGHTLY_ROOT', Path(__file__).resolve().parents[2]))
RUN = Path(os.environ.get('NIGHTLY_RUN', ROOT / 'run' / 'nightly'))
DB = RUN / 'queue.db'
LOGS = RUN / 'logs'
TERMINAL = ('COMPLETED', 'FAILED', 'NEEDS_ENGINEERING', 'NEEDS_APPROVAL', 'NEEDS_HARDWARE')
LIMIT_RE = re.compile(r'usage limit|rate limit|quota|limit reached|try again at', re.I)
FORBIDDEN = re.compile(r'\b(ssh|scp|rsync)\b.*(10\.|192\.168|x2|agibot)|ros2 (topic pub|service call).*x2', re.I)

SCHEMA = """
CREATE TABLE IF NOT EXISTS tasks(
  id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL, kind TEXT NOT NULL DEFAULT 'mechanical',
  cmd TEXT NOT NULL, cwd TEXT, state TEXT NOT NULL DEFAULT 'PENDING', priority INTEGER NOT NULL DEFAULT 50,
  deps TEXT NOT NULL DEFAULT '[]', resource TEXT, attempts INTEGER NOT NULL DEFAULT 0, max_attempts INTEGER NOT NULL DEFAULT 2,
  timeout_s INTEGER NOT NULL DEFAULT 3600, heartbeat_s INTEGER NOT NULL DEFAULT 900,
  signature TEXT, reason TEXT, repro TEXT, log_path TEXT, exit_code INTEGER, pid INTEGER,
  created REAL, started REAL, finished REAL, not_before REAL NOT NULL DEFAULT 0);
CREATE TABLE IF NOT EXISTS events(ts REAL, task TEXT, event TEXT, detail TEXT);
CREATE TABLE IF NOT EXISTS kv(k TEXT PRIMARY KEY, v TEXT);
CREATE TABLE IF NOT EXISTS failures(signature TEXT PRIMARY KEY, first_task TEXT, count INTEGER, first_ts REAL, last_ts REAL, detail TEXT);
"""


def connect():
    RUN.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(DB, timeout=30, isolation_level=None)
    db.row_factory = sqlite3.Row
    db.execute('PRAGMA journal_mode=WAL')
    db.execute('PRAGMA synchronous=FULL')
    db.executescript(SCHEMA)
    store.init(db)
    mergeq.init(db)
    db.execute('CREATE TABLE IF NOT EXISTS router_stats(id INTEGER PRIMARY KEY, ts REAL, task TEXT, attempt INTEGER, cls TEXT, agent TEXT, model TEXT, effort TEXT, escalation INTEGER, outcome TEXT, seconds REAL, quota_before TEXT, quota_after TEXT)')
    cols = [r[1] for r in db.execute('PRAGMA table_info(tasks)')]
    if 'requires_result' not in cols:
        db.execute('ALTER TABLE tasks ADD COLUMN requires_result INTEGER NOT NULL DEFAULT 0')  # 1: no result.json -> never COMPLETED
    for c_ in ('phase', 'attempt_id', 'dispatch_ref'):
        if c_ not in cols:
            db.execute(f'ALTER TABLE tasks ADD COLUMN {c_} TEXT')  # delegated-task contract: DISPATCHED..VERIFIED, bound to attempt_id
    for c_ in ('attempt_wt', 'branch', 'model', 'effort', 'base_from', 'task_type', 'complexity', 'risk', 'pin_model', 'pin_effort'):
        if c_ not in cols:
            db.execute(f'ALTER TABLE tasks ADD COLUMN {c_} TEXT')
    if 'dispatched_at' not in cols:
        db.execute('ALTER TABLE tasks ADD COLUMN dispatched_at REAL')
    if 'owner' not in cols:
        db.execute('ALTER TABLE tasks ADD COLUMN owner TEXT')  # agent that currently holds the task (durable ownership)
    if 'klass' not in cols:
        db.execute('ALTER TABLE tasks ADD COLUMN klass TEXT')  # SIM_ONLY | SIM_THEN_HARDWARE | HARDWARE_ONLY | NEEDS_APPROVAL
    return db


def ev(db, task, event, detail=''):
    db.execute('INSERT INTO events VALUES(?,?,?,?)', (time.time(), task, event, str(detail)[:2000]))


def git(*a):
    try:
        return subprocess.run(['git', '-C', str(ROOT), *a], capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:
        return ''


def kv_set(db, k, v):
    db.execute('INSERT INTO kv VALUES(?,?) ON CONFLICT(k) DO UPDATE SET v=excluded.v', (k, json.dumps(v)))


def kv_get(db, k, default=None):
    r = db.execute('SELECT v FROM kv WHERE k=?', (k,)).fetchone()
    return json.loads(r['v']) if r else default


def checkpoint(db, why):
    """Machine-readable snapshot, rewritten atomically after every significant queue event."""
    counts = {r['state']: r['n'] for r in db.execute('SELECT state, COUNT(*) n FROM tasks GROUP BY state')}
    cp = {'ts': time.time(), 'why': why, 'git_sha': git('rev-parse', 'HEAD'), 'git_branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
          'dirty': git('status', '--porcelain').count('\n') + (1 if git('status', '--porcelain') else 0),
          'queue': counts, 'agents': kv_get(db, 'agents', {}),
          'results': {'total': db.execute('SELECT COUNT(*) FROM results').fetchone()[0],
                      'by_verdict': {r['verdict']: r['n'] for r in db.execute('SELECT verdict, COUNT(*) n FROM results GROUP BY verdict')},
                      'evidence_missing': db.execute("SELECT COUNT(*) FROM results WHERE status!='STORED'").fetchone()[0]},
          'last_engineering_checkpoint': (lambda r: dict(r) if r else None)(db.execute('SELECT task,ts,source,verified,next_step FROM eng_checkpoints ORDER BY id DESC LIMIT 1').fetchone()),
          'running': [dict(name=r['name'], pid=r['pid'], started=r['started']) for r in db.execute("SELECT * FROM tasks WHERE state='RUNNING'")],
          'blocked': [dict(name=r['name'], state=r['state'], reason=r['reason'], repro=r['repro'])
                      for r in db.execute("SELECT * FROM tasks WHERE state IN ('NEEDS_APPROVAL','NEEDS_HARDWARE','NEEDS_ENGINEERING','PAUSED_AGENT','FAILED')")]}
    tmp = RUN / 'checkpoint.json.tmp'
    tmp.write_text(json.dumps(cp, indent=1))
    os.replace(tmp, RUN / 'checkpoint.json')


def enqueue(db, name, cmd, kind='mechanical', **kw):
    if FORBIDDEN.search(cmd):
        raise SystemExit(f'refused: command looks like it targets the physical robot: {cmd}')
    cols = dict(cwd=str(ROOT), priority=50, deps='[]', resource=None, max_attempts=2, timeout_s=3600, heartbeat_s=900, state='PENDING', repro=None, reason=None)
    cols.update({k: v for k, v in kw.items() if v is not None})
    if isinstance(cols['deps'], list):
        cols['deps'] = json.dumps(cols['deps'])
    if kind == 'engineering' and not cols['resource']:
        cols['resource'] = 'wt:' + str(cols['cwd'])  # one writer per worktree: ready() never starts two tasks on one resource
    row = db.execute('SELECT state FROM tasks WHERE name=?', (name,)).fetchone()
    if row:
        return False  # never overwrite history; a re-run needs a new name
    db.execute('INSERT INTO tasks(name,kind,cmd,cwd,state,priority,deps,resource,max_attempts,timeout_s,heartbeat_s,repro,reason,created) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
               (name, kind, cmd, cols['cwd'], cols['state'], cols['priority'], cols['deps'], cols['resource'], cols['max_attempts'],
                cols['timeout_s'], cols['heartbeat_s'], cols['repro'] or cmd, cols['reason'], time.time()))
    ev(db, name, 'ENQUEUED', cmd)
    return True


def signature(task, log_tail):
    """Group failures: the last decisive line with digits/hex/paths normalised."""
    lines = [ln for ln in log_tail.splitlines() if re.search(r'error|fail|assert|timeout|traceback|exception', ln, re.I)]
    key = (lines[-1] if lines else log_tail.strip().splitlines()[-1:] or ['no-output'])
    key = key if isinstance(key, str) else key[0]
    key = re.sub(r'0x[0-9a-f]+|\d+(\.\d+)?|/[\w./-]+', '#', key)
    return hashlib.sha1(f"{task}|{key}".encode()).hexdigest()[:12]


def recover(db):
    """After any restart: finish what a crash left half-done (evidence <-> index), then RUNNING rows are orphans:
    kill survivors of our process groups, then requeue or park."""
    for r_ in db.execute("SELECT id FROM tasks WHERE state='VERIFYING'").fetchall():
        vp = kv_get(db, f"verifpid:{r_['id']}")
        if vp:
            try:
                os.killpg(vp, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    rep = store.adopt_orphans(db, RUN)
    if any(rep.values()):
        ev(db, '*', 'STORE_RECOVER', json.dumps(rep)[:1500])
    for r in db.execute("SELECT * FROM tasks WHERE state='RUNNING'").fetchall():
        if r['pid']:
            try:
                os.killpg(r['pid'], signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        nxt = 'PENDING' if r['attempts'] < r['max_attempts'] else 'FAILED'
        if r['kind'] == 'engineering':
            eng_autocheckpoint(db, r, 'orchestrator-recover', 'agent/orchestrator died without a final message')
        db.execute('UPDATE tasks SET state=?, pid=NULL, reason=? WHERE id=?', (nxt, 'recovered after orchestrator restart (INTERRUPTED)', r['id']))
        ev(db, r['name'], 'INTERRUPTED', f'-> {nxt}')
    checkpoint(db, 'recover')


def ready(db):
    if kv_get(db, 'frozen', False):  # operator STOP: nothing new starts until `lab.py resume`
        return
    done = {r['name'] for r in db.execute("SELECT name FROM tasks WHERE state='COMPLETED'")}
    now = time.time()
    for r in db.execute("SELECT * FROM tasks WHERE state='PENDING' AND not_before<=? ORDER BY priority, id", (now,)):
        if not all(d in done for d in json.loads(r['deps'])):
            continue
        # recomputed for every candidate: a dispatch earlier in the same tick must already hold its lease
        if r['resource'] and db.execute("SELECT 1 FROM tasks WHERE resource=? AND state IN ('RUNNING','DELEGATED','VERIFYING') LIMIT 1", (r['resource'],)).fetchone():
            continue
        yield r


def acwd(row):
    """Directory that holds the attempt's artefacts: its own worktree when isolated, else the task cwd."""
    return (row['attempt_wt'] if 'attempt_wt' in row.keys() else None) or row['cwd'] or str(ROOT)


def result_dir(name, attempt):
    return RUN / 'work' / f'{name}.{attempt}'


def run_mechanical(db, r):
    name = r['name']
    attempt = r['attempts'] + 1
    log = LOGS / f"{name}.{attempt}.log"
    rdir = result_dir(name, attempt)
    rdir.mkdir(parents=True, exist_ok=True)
    dirty = bool(git('status', '--porcelain', '--untracked-files=no'))
    base = dict(task=name, attempt=attempt, git_sha=git('rev-parse', 'HEAD'), code_dirty=dirty,
                config_sha=hashlib.sha256(r['cmd'].encode()).hexdigest()[:16])
    db.execute("UPDATE tasks SET state='RUNNING', attempts=?, started=?, log_path=?, finished=NULL, exit_code=NULL WHERE id=?", (attempt, time.time(), str(log), r['id']))
    if kv_get(db, f'base:{name}:{attempt}') is None:  # an isolated attempt already recorded its own base SHA
        kv_set(db, f'base:{name}:{attempt}', base)
    ev(db, name, 'START', r['cmd'])
    checkpoint(db, f'start {name}')
    with open(log, 'wb') as fh:
        p = subprocess.Popen(['bash', '-lc', r['cmd']], cwd=dict(r).get('run_cwd') or r['cwd'], stdin=subprocess.DEVNULL, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True,
                             env=dict(os.environ, NIGHTLY_RESULT_DIR=str(rdir), NIGHTLY_TASK=name, NIGHTLY_ATTEMPT=str(attempt), NIGHTLY_ATTEMPT_ID=str(dict(r).get('attempt_id') or '')))
    db.execute('UPDATE tasks SET pid=? WHERE id=?', (p.pid, r['id']))
    return p, log


def eng_autocheckpoint(db, r, source, why):
    """Engineering checkpoint written by ordinary code when an agent ends without writing its own. Never 'verified'."""
    cwd = acwd(r)

    def g(*a):
        try:
            return subprocess.run(['git', '-C', cwd, *a], capture_output=True, text=True, timeout=20).stdout.strip()
        except Exception:
            return ''
    tail = ''
    if r['log_path'] and Path(r['log_path']).exists():
        tail = Path(r['log_path']).read_text(errors='replace')[-1500:]
    store.save_engineering_checkpoint(db, r['name'], source, git_sha=g('rev-parse', 'HEAD'), worktree=cwd,
                                      files_changed=g('diff', '--name-only', 'HEAD').splitlines(),
                                      unfinished=g('status', '--porcelain').splitlines()[:50], verified=False,
                                      next_step='new agent: re-check git state and evidence; treat uncommitted changes as UNVERIFIED', detail=f'{why}\n{tail}')


def harvest(db, r, rc, log, state, sig, reason):
    """Persist everything the task produced BEFORE the queue is allowed to say COMPLETED (failed attempts too).
    Returns None when the task row was written inside the commit transaction, else (state, reason) for a plain update."""
    attempt = r['attempts']
    rdir = result_dir(r['name'], attempt)
    rj = rdir / 'result.json'
    has_files = rdir.exists() and any(rdir.rglob('*'))
    req = bool(r['requires_result'])
    if not has_files and not req:
        return state, reason  # legacy task without a result contract
    items, errs = [], []
    if rj.exists():
        try:
            items, errs = store.validate(json.loads(rj.read_text()))
        except json.JSONDecodeError as e:
            errs = [f'result.json is not valid JSON: {e}']
    elif req:
        errs = ['result.json missing']
    if errs:
        state, reason = 'NEEDS_ENGINEERING', ('invalid/missing result: ' + '; '.join(errs))[:300]
        items = []
    rdir.mkdir(parents=True, exist_ok=True)
    if log.exists():
        (rdir / 'task.log').write_bytes(log.read_bytes())
    base = kv_get(db, f"base:{r['name']}:{attempt}", None) or dict(task=r['name'], attempt=attempt, git_sha=git('rev-parse', 'HEAD'), code_dirty=False)
    meta = dict(base=base, items=items, final_state=state, exit_code=rc, signature=sig)
    ev_path, man_sha = store.publish(RUN, r['name'], attempt, rdir, meta)
    store.commit(db, r['id'], items, ev_path, man_sha, base, state, rc, reason, sig)
    return None


def finish(db, r, p, log, verdict):
    name = r['name']
    tail = log.read_text(errors='replace')[-4000:] if log.exists() else ''
    rc = p.returncode
    if verdict == 'ok':
        state, reason, sig = 'COMPLETED', None, None
    else:
        sig = signature(name, tail)
        reason = verdict if verdict != 'fail' else f'exit {rc}'
        seen = db.execute('SELECT count FROM failures WHERE signature=?', (sig,)).fetchone()
        a = db.execute('SELECT attempts, max_attempts FROM tasks WHERE id=?', (r['id'],)).fetchone()
        # no blind retries: only infra problems (hang/interrupt) get another try; a deterministic failure goes to engineering
        retry = verdict in ('hang', 'timeout') and a['attempts'] < a['max_attempts'] and not seen
        state = 'PENDING' if retry else ('NEEDS_ENGINEERING' if not seen else 'FAILED')
        db.execute('INSERT INTO failures VALUES(?,?,?,?,?,?) ON CONFLICT(signature) DO UPDATE SET count=count+1, last_ts=excluded.last_ts',
                   (sig, name, 1, time.time(), time.time(), tail[-500:]))
    full = db.execute('SELECT * FROM tasks WHERE id=?', (r['id'],)).fetchone()
    if full['kind'] == 'engineering':
        eng_autocheckpoint(db, full, 'orchestrator-finish', f'agent command ended rc={rc} verdict={verdict}')
    if full['kind'] == 'engineering' and full['attempt_wt']:
        left, still = attempts.reap(full['attempt_wt'])  # the attempt's process is over; anything left in its worktree is killed, and reported
        if left:
            ev(db, name, 'ATTEMPT_SURVIVORS_KILLED', f'{left} still_alive={still}')
    if full['kind'] == 'engineering' and verdict == 'ok':  # exit code 0 of an agent is not completion either
        why = 'no result file'
        if full['attempt_wt']:
            attempts.commit(full['attempt_wt'], name, full['attempts'])  # the orchestrator, not the agent, commits and hashes
        f = find_result_file(full)
        if f:
            ok_, res, why = verify_engineering_result(db, full, f)
            if ok_:
                start_verification(db, full, f, res)  # state VERIFYING, lease kept, tests re-run by a separate job
                checkpoint(db, f'finish {name} -> VERIFYING')
                return
        state, reason = 'NEEDS_ENGINEERING', f'agent exited {rc} without a verified result: {why}'[:300]
        route_log(db, full, 'NO_RESULT', time.time() - (full['started'] or time.time()))
        kv_set(db, f"esc:{name}", kv_get(db, f"esc:{name}", 0) + 1)
    if full['kind'] == 'engineering' and verdict != 'ok':
        hd = handoff_decision(db, full, rc, verdict, tail)
        if hd:
            state, reason = hd
            ev(db, name, 'HANDOFF', reason)
    plain = harvest(db, full, rc, log, state, sig, reason)
    if plain is not None:  # nothing to persist (legacy task)
        db.execute('UPDATE tasks SET state=?, exit_code=?, pid=NULL, finished=?, signature=?, reason=? WHERE id=?', (state, rc, time.time(), sig, reason, r['id']))
    else:
        state = db.execute('SELECT state FROM tasks WHERE id=?', (r['id'],)).fetchone()['state']
    ev(db, name, state, f'rc={rc} {reason or ""} sig={sig}')
    checkpoint(db, f'finish {name} {state}')


def agent_probe(db):
    """Facts about agents, from the real CLIs. Never assume availability; a recorded limit pauses engineering tasks only."""
    st = kv_get(db, 'agents', {})
    for a in ('claude', 'codex'):
        v = agents.ADAPTERS[a].version()
        cur = st.get(a, {})
        cur.update(version=v, installed=bool(v))
        cur.setdefault('status', 'unknown')
        st[a] = cur
    kv_set(db, 'agents', st)
    return st


def is_limited(s, now=None):
    now = now or time.time()
    return s.get('status') == 'usage_limited' and max(s.get('retry_after') or 0, s.get('recheck_after') or 0) > now


def refresh_usage(db, force=False):
    """Measured availability. Codex: provider API (official reset). Claude: no source -> UNKNOWN, left to log evidence."""
    st = kv_get(db, 'agents', {})
    ttl = int(os.environ.get('NIGHTLY_USAGE_TTL_S', 300))
    c = st.setdefault('codex', {})
    if force or time.time() - (c.get('usage') or {}).get('measured_at', 0) > ttl:
        u = agents.codex_usage()
        c['usage'] = u
        if u['status'] == 'LIMITED':
            c.update(status='usage_limited', retry_after=u.get('reset_at'), reset_known=bool(u.get('reset_at')),
                     recheck_after=None if u.get('reset_at') else time.time() + int(os.environ.get('NIGHTLY_LIMIT_PROBE_S', 1800)))
        elif u['status'] == 'AVAILABLE' and c.get('status') == 'usage_limited':
            c.update(status='available', retry_after=None, recheck_after=None)  # provider says usable: stronger than an old log line
    st.setdefault('claude', {})['usage'] = agents.claude_usage()
    kv_set(db, 'agents', st)
    return st


WAIT_RESET_MAX_S = 3600


def select_agent(db, r):
    """Pick provider+model+effort for an engineering task. Order: the task's own --agents list (default codex,claude); router fills model/effort.
    A limited provider with an OFFICIAL reset within 60 minutes keeps the task (bounded wait, queue goes on) unless it is critical;
    a longer or unknown reset hands off. Returns dict(agent, model, effort, route) | dict(wait_until, reason) | None."""
    st = agent_probe(db)
    st = refresh_usage(db) or st
    now = time.time()
    tried = kv_get(db, f"tried:{r['name']}", [])
    allowed = db_get_agents(r)
    esc = kv_get(db, f"esc:{r['name']}", 0)
    routed = router.route(r['task_type'], r['complexity'], r['risk'], escalation=esc, claude_verified=kv_get(db, 'claude_verified', {}), allowed_agents=set(allowed))
    cands = []
    for ag in allowed:
        pick = next((c for c in routed if c['agent'] == ag), {'agent': ag, 'model': None, 'effort': None, 'cls': router.classify(r['task_type'], r['complexity'], r['risk']), 'verified': False, 'reason': 'no router candidate'})
        if r['pin_model'] and ag == allowed[0]:  # explicit operator choice wins (tasks.model/effort only record what was chosen last time)
            pick = dict(pick, model=r['pin_model'], effort=r['pin_effort'] or pick['effort'], reason='pinned by operator')
        cands.append(pick)
    limited_first, waits = None, []
    for i, c in enumerate(cands):
        a = c['agent']
        if a in tried:
            continue
        s_ = st.get(a, {})
        if not s_.get('installed'):
            continue
        if is_limited(s_, now):
            until = s_.get('retry_after') or s_.get('recheck_after')
            waits.append(until)
            if i == 0 or limited_first is None:
                limited_first = (a, s_.get('retry_after'), until)
            continue
        if limited_first and limited_first[1] and limited_first[1] - now <= WAIT_RESET_MAX_S and r['priority'] > 10:
            # the preferred agent returns soon (official reset <= 60 min) and the task is not critical: wait, do not pay a handoff
            return dict(wait_until=limited_first[1] + 5, reason=f"{limited_first[0]} limited, official reset in {int(limited_first[1] - now)}s (<=60 min): waiting instead of handoff")
        return dict(agent=a, model=c['model'], effort=c['effort'], route=c)
    if waits:
        until = min(w for w in waits if w)
        return dict(wait_until=until + 5, reason='all candidate agents limited; waiting for the earliest reset/recheck')
    return None


def route_log(db, row, outcome, seconds=None):
    """Quality/cost evidence per routed attempt: outcome of the regression gate, duration, Codex quota before/after when measurable."""
    try:
        db.execute('INSERT INTO router_stats(ts,task,attempt,cls,agent,model,effort,escalation,outcome,seconds,quota_before,quota_after) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)',
                   (time.time(), row['name'], row['attempts'], router.classify(row['task_type'], row['complexity'], row['risk']), row['owner'], row['model'], row['effort'],
                    kv_get(db, f"esc:{row['name']}", 0), outcome, seconds, json.dumps(kv_get(db, f"quota_before:{row['name']}:{row['attempts']}")),
                    json.dumps((agents.codex_usage() if row['owner'] == 'codex' else agents.claude_usage()).get('used_pct'))))
    except Exception as e:
        ev(db, row['name'], 'ROUTE_LOG_ERROR', str(e)[:120])


def db_get_agents(r):
    m = re.search(r'--agents=(\S+)', r['repro'] or '')
    return m.group(1).split(',') if m else ['codex', 'claude']


def own_sessions(db):
    return {r['pid'] for r in db.execute("SELECT pid FROM tasks WHERE state='RUNNING' AND pid IS NOT NULL")}


def result_contract(name, attempt, aid, rdir, cwd, isolated=False):
    token = f'QAZBOT-TASK:{name}:{attempt}:{aid}'
    if isolated:
        return token, (f'[{token}] You work ONLY in your own git worktree {cwd} (a branch cut for this attempt). Do not commit: the orchestrator commits and hashes for you. '
                       f'When finished write the file {cwd}/.qazbot/results/{name}.{attempt}.json containing JSON with: task="{name}", attempt={attempt}, attempt_id="{aid}", '
                       'summary, tests={cmd} where cmd is the pytest command that proves the work (the orchestrator re-runs it; allowed: `python3 -m pytest ...`), '
                       'files=[{path}] listing every file you changed or created (relative paths, no symlinks), root_causes=[{key, summary, status, evidence}] (optional). '
                       'Without that file the work is not accepted. Never contact any robot or network service.')
    return token, (f'[{token}] Delivery of this message is not completion. Work in your OWN worktree. When finished, write engineering_result.json to {rdir} '
                   f'(or, if that path is not writable, to {cwd}/.qazbot/results/{name}.{attempt}.json) with: task="{name}", attempt={attempt}, attempt_id="{aid}", '
                   'git_sha=<your commit>, summary, tests={cmd, passed:int, failed:int} (cmd is re-run by the orchestrator, must be an allow-listed pytest command), '
                   'files=[{path, sha256}] (non-empty, relative to the worktree, no symlinks), root_causes=[{key, summary, status, evidence}]. '
                   'Without that file the work is not accepted. If a newer attempt supersedes this one the file is rejected. Stop immediately on OPERATOR STOP.')


def dispatch_codex(db, r):
    """Hand the task to the existing Codex session. Delivery is NOT execution. A write-ahead intent (DELEGATED/DISPATCHING with the token)
    is persisted BEFORE `codex queue` runs, so a crash or a timeout can never lose or duplicate the record of a possibly delivered message."""
    name, attempt, aid = r['name'], r['attempts'] + 1, agents.new_attempt_id()
    rdir = result_dir(name, attempt)
    rdir.mkdir(parents=True, exist_ok=True)
    cwd = r['cwd'] or str(ROOT)
    token, contract = result_contract(name, attempt, aid, rdir, cwd)
    base = dict(task=name, attempt=attempt, git_sha=git('rev-parse', 'HEAD'), code_dirty=bool(git('status', '--porcelain', '--untracked-files=no')),
                config_sha=hashlib.sha256(r['cmd'].encode()).hexdigest()[:16])
    kv_set(db, f'base:{name}:{attempt}', base)
    thread = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
    db.execute("UPDATE tasks SET state='DELEGATED', phase='DISPATCHING', attempts=?, started=?, owner='codex', attempt_id=?, dispatched_at=?, finished=NULL, exit_code=NULL, "
               "dispatch_ref=?, reason='dispatching to codex (intent recorded)' WHERE id=?",
               (attempt, time.time(), aid, time.time(), json.dumps(dict(thread=thread, item_id=None, token=token)), r['id']))
    ev(db, name, 'DISPATCH_INTENT', token)
    prompt = f"{r['repro']}\n{agent_context(r)}\n{agents.RULES}\n{contract}"
    ad = agents.ADAPTERS['codex']
    try:
        p = subprocess.run([ad.bin, 'queue', '--thread', thread, '--message', prompt], capture_output=True, text=True,
                           timeout=float(os.environ.get('NIGHTLY_DISPATCH_TIMEOUT_S', 45)))
        rc, out = p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        ev(db, name, 'DISPATCH_UNKNOWN', 'codex queue timed out: delivery unknown, lease kept, will be resolved by observation')
        return  # DISPATCHING stays: the message may still be delivered; never freed, never re-sent
    except OSError as e:
        rc, out = 127, str(e)
    item = agents.parse_queued_id(out)
    if rc == 0 and item:
        db.execute("UPDATE tasks SET phase='DISPATCHED', dispatch_ref=?, reason='delivered to codex; execution unconfirmed' WHERE id=?",
                   (json.dumps(dict(thread=thread, item_id=item, token=token)), r['id']))
        ev(db, name, 'DISPATCHED', json.dumps(dict(thread=thread, item_id=item, token=token)))
    elif rc == 0:  # delivery reported but unverifiable id: observe by token
        ev(db, name, 'DISPATCH_UNKNOWN', 'rc=0 without a queued id')
    else:  # the CLI itself failed: nothing was queued
        ev(db, name, 'DISPATCH_FAILED', out[-300:])
        full = db.execute('SELECT * FROM tasks WHERE id=?', (r['id'],)).fetchone()
        hd = handoff_decision(db, full, rc, 'fail', out) or ('NEEDS_ENGINEERING', 'dispatch to codex failed')
        db.execute("UPDATE tasks SET state=?, reason=?, phase=NULL WHERE id=? AND state='DELEGATED'", (hd[0], hd[1], r['id']))
    checkpoint(db, f'dispatched {name}')


def find_result_file(row, cwd=None):
    name, attempt = row['name'], row['attempts']
    cwd = cwd or acwd(row)
    cands = [Path(cwd) / '.qazbot' / 'results' / f'{name}.{attempt}.json']
    if not row['attempt_wt']:
        cands.insert(0, result_dir(name, attempt) / 'engineering_result.json')
    for p in cands:
        if p.is_file() and not p.is_symlink():
            return p
    return None


def _inside(root, rel):
    """rel must be a relative path that stays inside root and is not (behind) a symlink."""
    if not isinstance(rel, str) or not rel or os.path.isabs(rel):
        return None
    full = Path(root) / rel
    try:
        if os.path.realpath(full) != os.path.normpath(os.path.join(os.path.realpath(root), rel)):
            return None  # a symlink or .. changed where it really points
        if not str(os.path.realpath(full)).startswith(os.path.realpath(root) + os.sep):
            return None
    except OSError:
        return None
    return full if full.is_file() and not full.is_symlink() else None


def verify_engineering_result(db, row, path):
    """STRUCTURAL verification only (cheap, synchronous): identity, fencing, Git SHA, artefact binding, allow-listed test command.
    The test re-run itself is a separate non-blocking job. Never raises."""
    try:
        return _verify(db, row, path)
    except Exception as e:  # malformed input must reject the result, never crash the orchestrator
        return False, None, f'malformed result ({type(e).__name__}): {str(e)[:120]}'


def _verify(db, row, path):
    import shlex
    cwd = acwd(row)
    isolated = bool(row['attempt_wt'])
    d = json.loads(path.read_text())
    if not isinstance(d, dict):
        return False, None, 'malformed result: not an object'
    if d.get('task') != row['name'] or d.get('attempt') != row['attempts']:
        return False, None, 'result is for another task/attempt'
    if d.get('attempt_id') != row['attempt_id']:
        return False, None, 'STALE_ATTEMPT: attempt_id does not match the current attempt (superseded or forged)'
    base = (kv_get(db, f"base:{row['name']}:{row['attempts']}", {}) or {}).get('git_sha')
    sha = d.get('git_sha')
    if isolated:  # the orchestrator committed the attempt; the agent's own sha, if any, must lie on this attempt's branch
        tip = attempts.head(cwd)
        if sha is not None and (not isinstance(sha, str) or subprocess.run(['git', '-C', cwd, 'merge-base', '--is-ancestor', sha, tip], capture_output=True).returncode != 0):
            return False, None, 'git_sha is not on this attempt\'s branch'
        sha = tip
        changed = subprocess.run(['git', '-C', cwd, 'diff', '--name-only', f'{base}..{tip}'], capture_output=True, text=True).stdout.split()
        if not changed:
            return False, None, 'attempt produced no change relative to its base SHA'
    else:
        if not isinstance(sha, str) or not re.fullmatch(r'[0-9a-f]{40}', sha) or subprocess.run(['git', '-C', cwd, 'cat-file', '-e', sha + '^{commit}'], capture_output=True).returncode != 0:
            return False, None, 'git_sha is not a commit in the task repository'
        changed = None
    if base and subprocess.run(['git', '-C', cwd, 'merge-base', '--is-ancestor', base, sha], capture_output=True).returncode != 0:
        return False, None, 'git_sha does not descend from the SHA the attempt started from'
    t = d.get('tests')
    if not isinstance(t, dict) or not isinstance(t.get('cmd'), str):
        return False, None, 'tests{cmd} missing: result not verifiable'
    if not isolated and (type(t.get('passed')) is not int or type(t.get('failed')) is not int):
        return False, None, 'tests{passed,failed} missing: result not verifiable'
    files = d.get('files')
    if not isinstance(files, list) or not files:
        return False, None, 'files[] must be a non-empty list: result is not bound to artefacts'
    for f in files:
        if not isinstance(f, dict):
            return False, None, 'files[] entry is not an object'
        fp = _inside(cwd, f.get('path'))
        if fp is None:
            return False, None, f"file path rejected (absolute, outside the worktree, symlink or missing): {str(f.get('path'))[:80]}"
        actual = store.sha256_file(fp)
        if f.get('sha256') is not None and f.get('sha256') != actual:
            return False, None, f"file hash mismatch: {f.get('path')}"
        if isolated and f['path'] not in changed:
            return False, None, f"file not changed by this attempt: {f.get('path')}"
        f['sha256'] = actual  # hashes are computed here, not trusted
    rcs = d.get('root_causes', [])
    if not isinstance(rcs, list) or not all(isinstance(x, dict) for x in rcs):
        return False, None, 'root_causes must be a list of objects'
    argv = shlex.split(t['cmd'])
    allow = [shlex.split(x) for x in os.environ.get('NIGHTLY_VERIFY_ALLOW', 'python3 -m pytest;pytest').split(';') if x.strip()]
    if not argv or not any(argv[:len(a)] == a for a in allow):
        return False, None, f"tests.cmd not allow-listed for orchestrator re-run: {t['cmd'][:60]}"
    item = {'scenario_id': f"engineering:{row['name']}", 'seed': row['attempt_id'], 'verdict': 'UNVERIFIED',
            'metrics': {'tests_cmd': t['cmd'], 'git_sha': sha, 'branch': row['branch']}, 'safety_violations': 0, 'notes': str(d.get('summary') or '')[:300]}
    return True, (item, d, argv), 'structurally accepted'


VERIFS = {}


def start_verification(db, row, path, res):
    """Structural checks passed: persist VERIFYING (lease kept) and launch the test re-run as a separate niced job."""
    item, d, argv = res
    rdir = result_dir(row['name'], row['attempts'])
    rdir.mkdir(parents=True, exist_ok=True)
    tmp = rdir / '.engineering_result.json.tmp'
    tmp.write_text(json.dumps(d))
    os.replace(tmp, rdir / 'engineering_result.json')
    db.execute("UPDATE tasks SET state='VERIFYING', phase='RESULT_RECORDED', reason='result accepted structurally; tests re-running' WHERE id=? AND state IN ('DELEGATED','RUNNING','VERIFYING')", (row['id'],))
    ev(db, row['name'], 'RESULT', 'structurally accepted; verification job starting')
    job = attempts.VerifyJob(argv, acwd(row), str(rdir / 'verify.log'), float(os.environ.get('NIGHTLY_VERIFY_TIMEOUT_S', 900)), int(os.environ.get('NIGHTLY_VERIFY_NICE', 10)))
    VERIFS[(row['id'], row['attempts'])] = job
    kv_set(db, f"verifpid:{row['id']}", job.p.pid)  # so a stop/restart can kill a job whose owner is gone


def record_engineering_result(db, row, item_d, ok):
    item, d = item_d[0], item_d[1]
    attempt = row['attempts']
    rdir = result_dir(row['name'], attempt)
    rdir.mkdir(parents=True, exist_ok=True)
    item = dict(item, verdict='PASS' if ok else 'FAIL')
    item['metrics'] = dict(item['metrics'], rerun_ok=ok)
    base = kv_get(db, f"base:{row['name']}:{attempt}", None) or dict(task=row['name'], attempt=attempt, git_sha=git('rev-parse', 'HEAD'), code_dirty=False)
    state = 'COMPLETED' if ok else 'NEEDS_ENGINEERING'
    reason = None if ok else 'result recorded but tests failing on orchestrator re-run'
    meta = dict(base=base, items=[item], final_state=state, exit_code=0, signature=None)
    ev_path, man_sha = store.publish(RUN, row['name'], attempt, rdir, meta)  # evidence durable BEFORE the queue may say COMPLETED
    extra = [("UPDATE tasks SET phase=? WHERE id=?", ('VERIFIED' if ok else 'RESULT_RECORDED', row['id']))]
    now = time.time()
    for rc_ in d.get('root_causes') or []:
        if isinstance(rc_, dict) and rc_.get('key') and rc_.get('summary'):
            st_ = 'VERIFIED' if ok and rc_.get('status') == 'VERIFIED' and rc_.get('evidence') else 'HYPOTHESIS'
            extra.append(("INSERT INTO root_causes(key,summary,status,evidence,task,ts) VALUES(?,?,?,?,?,?) ON CONFLICT(key) DO UPDATE SET summary=excluded.summary, "
                          "status=excluded.status, evidence=excluded.evidence, task=excluded.task, ts=excluded.ts WHERE root_causes.status!='VERIFIED'",
                          (str(rc_['key'])[:80], str(rc_['summary'])[:400], st_, str(rc_.get('evidence') or '')[:300], row['name'], now)))
    store.commit(db, row['id'], [item], ev_path, man_sha, base, state, 0, reason, None, extra=extra)
    kv_set(db, f"tried:{row['name']}", [])
    route_log(db, row, 'PASS' if ok else 'GATE_FAIL', time.time() - (row['started'] or time.time()))
    if not ok:
        kv_set(db, f"esc:{row['name']}", kv_get(db, f"esc:{row['name']}", 0) + 1)  # a failed gate escalates the next attempt instead of repeating it
    if ok and row['owner'] == 'claude' and row['model'] and row['effort']:
        cv = kv_get(db, 'claude_verified', {})
        cv.setdefault(row['model'], [])
        if row['effort'] not in cv[row['model']]:
            cv[row['model']].append(row['effort'])
        kv_set(db, 'claude_verified', cv)  # first real success proves this model/effort pair is accepted by the CLI
    if ok:
        kv_set(db, f"verified_sha:{row['name']}", item['metrics']['git_sha'])
        full_ = db.execute('SELECT * FROM tasks WHERE id=?', (row['id'],)).fetchone()
        if mergeq.enqueue(db, RUN, kv_get, full_):
            ev(db, row['name'], 'MERGEQ_QUEUED', mergeq.branch_name())
        ev(db, row['name'], 'INTEGRATION_READY', f"branch={row['branch']} sha={item['metrics']['git_sha']} (not merged anywhere; merge is a separate gated step)")


def poll_delegated(db, force=False):
    """Observation + verification progress. Never starts agents, never kills them, never raises."""
    if not force and time.time() - kv_get(db, 'last_delegate_poll', 0) < float(os.environ.get('NIGHTLY_POLL_S', 20)):
        return
    kv_set(db, 'last_delegate_poll', time.time())
    for row in db.execute("SELECT * FROM tasks WHERE state IN ('DELEGATED','VERIFYING')").fetchall():
        try:
            _poll_verifying(db, row) if row['state'] == 'VERIFYING' else _poll_one(db, row)
        except Exception as e:
            ev(db, row['name'], 'POLL_ERROR', f'{type(e).__name__}: {str(e)[:200]}')


def _poll_verifying(db, row):
    key = (row['id'], row['attempts'])
    f = find_result_file(row)
    if f is None:
        db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING', reason='result file vanished during verification' WHERE id=? AND state='VERIFYING'", (row['id'],))
        return
    ok_, res, why = verify_engineering_result(db, row, f)
    if not ok_:
        db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING', reason=? WHERE id=? AND state='VERIFYING'", (f'verification precondition failed: {why}'[:300], row['id']))
        return
    job = VERIFS.get(key)
    if job is None:  # orchestrator restarted: the job is simply started again (idempotent)
        start_verification(db, row, f, res)
        return
    out = job.poll()
    if out is None:
        return
    VERIFS.pop(key, None)
    ok, why2 = out
    ev(db, row['name'], 'VERIFIED' if ok else 'VERIFY_FAILED', why2)
    record_engineering_result(db, row, res, ok)
    checkpoint(db, f"verified {row['name']}")


def _poll_one(db, row):
    grace = float(os.environ.get('NIGHTLY_RESULT_GRACE_S', 120))
    ref = json.loads(row['dispatch_ref'] or '{}')
    f = find_result_file(row)
    if f:
        ok_, res, why = verify_engineering_result(db, row, f)
        if ok_:
            start_verification(db, row, f, res)
            return
        if kv_get(db, f"badres:{row['name']}:{row['attempts']}") != why:
            kv_set(db, f"badres:{row['name']}:{row['attempts']}", why)
            ev(db, row['name'], 'RESULT_REJECTED', why)
    obs = agents.codex_observe(ref.get('thread'), ref.get('item_id'), ref.get('token'))
    phase = row['phase']
    if not obs['source_ok']:
        return  # unknown stays unknown
    if obs['turn_status'] == 'inProgress':
        new = 'RUNNING'
    elif obs['turn_status'] or (obs['queued'] is False and ref.get('item_id')):
        new = 'ACKNOWLEDGED' if phase in ('DISPATCHED', 'DISPATCHING', None) else phase
    elif obs['queued'] and phase == 'DISPATCHING':
        new = 'DISPATCHED'
    else:
        new = phase
    if new != phase:
        db.execute("UPDATE tasks SET phase=? WHERE id=? AND state='DELEGATED' AND attempt_id=?", (new, row['id'], row['attempt_id']))
        ev(db, row['name'], 'PHASE', f'{phase}->{new}')
    age = time.time() - (row['dispatched_at'] or time.time())
    if obs['turn_status'] in ('completed', 'failed', 'interrupted'):
        key = f"turn_done_at:{row['name']}:{row['attempts']}"
        seen = kv_get(db, key)
        if seen is None:
            kv_set(db, key, time.time())
        elif time.time() - seen > grace:  # attempt proven ended and still no verified result: release the lease, hand to engineering
            n = db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING', reason=? WHERE id=? AND state='DELEGATED' AND attempt_id=?",
                           (f"codex turn {obs['turn_status']} without a verified result", row['id'], row['attempt_id'])).rowcount
            if n:
                ev(db, row['name'], 'ATTEMPT_ENDED_NO_RESULT', obs['turn_status'])
                eng_autocheckpoint(db, db.execute('SELECT * FROM tasks WHERE id=?', (row['id'],)).fetchone(), 'orchestrator-delegate-ended', f"turn {obs['turn_status']}")
                checkpoint(db, f"ended {row['name']}")
    elif obs['turn_status'] is None and age > float(os.environ.get('NIGHTLY_NO_ACK_ALERT_S', 3600)) and not kv_get(db, f"noack:{row['name']}:{row['attempts']}"):
        kv_set(db, f"noack:{row['name']}:{row['attempts']}", True)  # regardless of queue visibility; lease kept, nothing assumed
        ev(db, row['name'], 'NO_ACK_ALERT', f"phase={new} delivered {int(age)}s ago, queued={obs['queued']}: not success, not safe to assume stopped; lease kept; operator: `nq.py supersede {row['name']} --force-lease-release`")


def tick(db, cur):
    """One scheduling step. cur = (row, Popen, log, last_size, last_progress) or None. Returns the new cur."""
    poll_delegated(db)
    try:
        mergeq.step(db, RUN, lambda t, e, d='': ev(db, t, e, d))
    except Exception as e_:
        ev(db, '*', 'MERGEQ_ERROR', f'{type(e_).__name__}: {str(e_)[:200]}')
    db.execute("UPDATE tasks SET state='PENDING', reason='reset reached: retrying' WHERE state='PAUSED_LIMIT' AND not_before<=?", (time.time(),))
    if cur:
        r, p, log, size, prog = cur
        rc = p.poll()
        if rc is not None:
            finish(db, r, p, log, 'ok' if rc == 0 else 'fail')
            return None
        sz = log.stat().st_size if log.exists() else 0
        hb = Path(str(log) + '.hb')
        beat = max(hb.stat().st_mtime if hb.exists() else 0, prog)
        if sz != size:
            beat = time.time()
        now = time.time()
        if now - r['started'] > r['timeout_s']:
            os.killpg(p.pid, signal.SIGKILL); p.wait(); finish(db, r, p, log, 'timeout'); return None
        if now - beat > r['heartbeat_s']:  # no output and no heartbeat for heartbeat_s: genuinely stuck, not just slow
            os.killpg(p.pid, signal.SIGKILL); p.wait(); finish(db, r, p, log, 'hang'); return None
        return (r, p, log, sz, beat)
    for r in ready(db):
        if r['kind'] == 'engineering':
            sel = select_agent(db, r)
            if sel is None:
                db.execute("UPDATE tasks SET state='PAUSED_AGENT', reason='no agent available (not installed)' WHERE id=?", (r['id'],))
                ev(db, r['name'], 'PAUSED_AGENT', 'waiting for agent availability')
                checkpoint(db, f"paused {r['name']}")
                continue
            if 'wait_until' in sel:
                db.execute("UPDATE tasks SET state='PAUSED_LIMIT', not_before=?, reason=? WHERE id=?", (sel['wait_until'], sel['reason'][:300], r['id']))
                ev(db, r['name'], 'PAUSED_LIMIT', sel['reason'])
                checkpoint(db, f"paused-limit {r['name']}")
                continue
            a = sel['agent']
            db.execute('UPDATE tasks SET model=?, effort=? WHERE id=?', (sel['model'], sel['effort'], r['id']))
            ev(db, r['name'], 'ROUTE', json.dumps(sel['route']))
            r = dict(r) | {'model': sel['model'], 'effort': sel['effort']}
            kv_set(db, f"quota_before:{r['name']}:{r['attempts'] + 1}", (agents.codex_usage() if a == 'codex' else agents.claude_usage()).get('used_pct'))
            ad = agents.ADAPTERS[a]
            isolate = os.environ.get('NIGHTLY_ISOLATE', '1') != '0' and ad.mode == 'process' and attempts.is_git_repo(r['cwd'] or str(ROOT))
            if not isolate:
                fw = agents.foreign_writers(r['cwd'] or str(ROOT), own_sessions(db))
                if fw:  # an agent we did not start (operator TUI) is already in this worktree: never become a second writer
                    if kv_get(db, f"leasewarn:{r['name']}") != [p_[0] for p_ in fw]:
                        kv_set(db, f"leasewarn:{r['name']}", [p_[0] for p_ in fw])
                        ev(db, r['name'], 'LEASE_BLOCKED_FOREIGN_WRITER', json.dumps(fw)[:400])
                    continue
            kv_set(db, f"tried:{r['name']}", kv_get(db, f"tried:{r['name']}", []) + [a])
            db.execute('UPDATE tasks SET owner=? WHERE id=?', (a, r['id']))
            ev(db, r['name'], 'OWNER', a)
            if ad.mode == 'queue':
                dispatch_codex(db, r)
                continue
            attempt_n, aid = r['attempts'] + 1, agents.new_attempt_id()
            run_cwd, wt = r['cwd'] or str(ROOT), None
            if isolate:
                bf = r['base_from']
                base_sha = (kv_get(db, f'verified_sha:{bf}') if bf else None) or attempts.head(run_cwd)
                try:
                    wt, branch = attempts.prepare(run_cwd, RUN, r['name'], attempt_n, aid, base_sha)
                except Exception as e:
                    ev(db, r['name'], 'ATTEMPT_PREPARE_FAILED', str(e)[:300])
                    db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING', reason=? WHERE id=?", (f'cannot prepare attempt worktree: {e}'[:300], r['id']))
                    continue
                db.execute('UPDATE tasks SET attempt_wt=?, branch=? WHERE id=?', (wt, branch, r['id']))
                kv_set(db, f"base:{r['name']}:{attempt_n}", dict(task=r['name'], attempt=attempt_n, git_sha=base_sha, code_dirty=False,
                                                                   config_sha=hashlib.sha256(r['cmd'].encode()).hexdigest()[:16]))
                run_cwd = wt
            else:
                db.execute('UPDATE tasks SET attempt_wt=NULL, branch=NULL WHERE id=?', (r['id'],))
            _tok, contract = result_contract(r['name'], attempt_n, aid, result_dir(r['name'], attempt_n), run_cwd, isolated=bool(wt))
            db.execute('UPDATE tasks SET attempt_id=? WHERE id=?', (aid, r['id']))
            cmd = ad.command(r, agent_context(r) + '\n' + contract, cwd=run_cwd, model=r['model'], effort=r['effort'], result_dir=result_dir(r['name'], attempt_n))
            r = dict(r) | {'cmd': cmd, 'attempt_id': aid, 'run_cwd': run_cwd}
        p, log = run_mechanical(db, r)
        row = db.execute('SELECT * FROM tasks WHERE id=?', (r['id'],)).fetchone()
        return (row, p, log, 0, time.time())
    return None


def agent_context(db_path_row):
    """Established facts for a fresh agent, read from SQLite (not from a previous agent's last message)."""
    db = connect()
    parts = [store.known_causes_text(db)]
    c = db.execute('SELECT * FROM eng_checkpoints WHERE task=? ORDER BY id DESC LIMIT 1', (db_path_row['name'],)).fetchone()
    if c:
        parts.append(f"Last checkpoint ({c['source']}, verified={bool(c['verified'])}, git {c['git_sha']}): next={c['next_step']}; unfinished={c['unfinished']}. "
                     "Re-check git state first; uncommitted changes are UNVERIFIED.")
    return '\n'.join(x for x in parts if x)


def agent_command(agent, r, extra=''):
    return agents.ADAPTERS[agent].command(r, agent_context(r) + ('\n' + extra if extra else ''))


def handoff_decision(db, full, rc, verdict, tail):
    """Engineering task ended badly: decide continue-elsewhere vs wait vs engineering, from facts only.
    Returns (state, reason) or None. Never needs the dying agent's last message."""
    cause = agents.classify(rc, verdict, tail)
    if cause in ('ok', 'error'):
        return None
    owner = full['owner']
    st = kv_get(db, 'agents', {})
    if cause == 'limit' and owner:  # confirmed by provider text in the log; reset time is not parsed/assumed
        # reset time is NOT known from a log line: do not invent one. recheck_after is only a bounded probe cadence.
        st.setdefault(owner, {}).update(status='usage_limited', retry_after=None, reset_known=False,
                                        recheck_after=time.time() + int(os.environ.get('NIGHTLY_LIMIT_PROBE_S', 1800)))
        kv_set(db, 'agents', st)
    tried = kv_get(db, f"tried:{full['name']}", [])
    now = time.time()
    nh = kv_get(db, f"handoffs:{full['name']}", 0)
    if nh >= int(os.environ.get('NIGHTLY_MAX_HANDOFFS', 4)):
        return 'NEEDS_ENGINEERING', f'handoff cap reached ({nh}): stop bouncing between agents'
    for a in db_get_agents(full):
        s = st.get(a, {})
        if a not in tried and s.get('installed', agents.ADAPTERS[a].installed()) and not is_limited(s, now):
            db.execute('UPDATE tasks SET max_attempts=max_attempts+1 WHERE id=?', (full['id'],))  # a handoff is not a failed attempt
            kv_set(db, f"handoffs:{full['name']}", nh + 1)
            return 'PENDING', f'HANDOFF {owner}->{a} ({cause})'
    kv_set(db, f"tried:{full['name']}", [])  # waiting for a reset: the limited owner may take the task again later
    if cause == 'limit':
        return 'PAUSED_AGENT', f'{owner} limited and no other agent available (waiting)'
    return 'NEEDS_ENGINEERING', f'agent {owner} lost and no alternative agent'


def revive_agents(db):
    """PAUSED_AGENT tasks go back to PENDING when an agent is believed available again (retry_after elapsed)."""
    st = kv_get(db, 'agents', {})
    now = time.time()
    for a, s in st.items():
        if s.get('status') == 'usage_limited' and not is_limited(s, now):
            s['status'] = 'unknown'
    kv_set(db, 'agents', st)
    if any(s.get('installed') and s.get('status') != 'usage_limited' for s in st.values()):
        n = db.execute("UPDATE tasks SET state='PENDING' WHERE state='PAUSED_AGENT'").rowcount
        if n:
            ev(db, '*', 'REVIVE', f'{n} PAUSED_AGENT -> PENDING')


def daemon(args):
    db = connect()
    lock = open(RUN / 'orchestrator.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)  # single-owner lease
    except OSError:
        sys.exit('another orchestrator owns the lease')
    lock.write(str(os.getpid())); lock.flush()
    recover(db)
    agent_probe(db)
    ev(db, '*', 'DAEMON_START', f'pid={os.getpid()} sha={git("rev-parse", "HEAD")}')
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    cur, last_revive = None, 0
    t_end = time.time() + args.max_seconds if args.max_seconds else None
    while not stop and not (t_end and time.time() > t_end):
        cur = tick(db, cur)
        if time.time() - last_revive > 300:
            revive_agents(db); last_revive = time.time()
        if cur is None and not args.keep_alive and not db.execute("SELECT 1 FROM tasks WHERE state IN ('PENDING','RUNNING','DELEGATED','VERIFYING','PAUSED_LIMIT') LIMIT 1").fetchone() \
                and not db.execute("SELECT 1 FROM merge_queue WHERE state IN ('QUEUED','GATING') LIMIT 1").fetchone():
            break
        time.sleep(args.poll)
    if cur:  # graceful stop: leave the child to be recovered on next start rather than orphaning silently
        os.killpg(cur[1].pid, signal.SIGTERM)
    checkpoint(db, 'daemon exit')


def cmd_status(args):
    db = connect()
    for r in db.execute('SELECT id,name,kind,state,attempts,priority,reason FROM tasks ORDER BY id'):
        print(f"{r['id']:3} {r['state']:17} {r['kind'][:4]} a={r['attempts']} {r['name']}  {r['reason'] or ''}")
    print(json.dumps(kv_get(db, 'agents', {})))


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='c', required=True)
    d = sp.add_parser('daemon'); d.add_argument('--poll', type=float, default=2.0); d.add_argument('--keep-alive', action='store_true'); d.add_argument('--max-seconds', type=float, default=0)
    e = sp.add_parser('enqueue'); e.add_argument('name'); e.add_argument('cmd'); e.add_argument('--kind', default='mechanical')
    e.add_argument('--priority', type=int); e.add_argument('--deps', default='[]'); e.add_argument('--resource'); e.add_argument('--timeout', type=int)
    e.add_argument('--heartbeat', type=int); e.add_argument('--cwd'); e.add_argument('--state'); e.add_argument('--reason'); e.add_argument('--klass'); e.add_argument('--requires-result', action='store_true'); e.add_argument('--model'); e.add_argument('--effort'); e.add_argument('--base-from'); e.add_argument('--task-type'); e.add_argument('--complexity'); e.add_argument('--risk')
    sp.add_parser('status')
    m = sp.add_parser('mark'); m.add_argument('name'); m.add_argument('state'); m.add_argument('--reason', default='')
    ag = sp.add_parser('agent'); ag.add_argument('name'); ag.add_argument('status'); ag.add_argument('--retry-after-s', type=int, default=3600)
    sp.add_parser('checkpoint')
    rl = sp.add_parser('results'); rl.add_argument('--task'); rl.add_argument('--json', action='store_true')
    sp.add_parser('verify-evidence')
    sp.add_parser('usage')
    mq = sp.add_parser('mergeq'); mq.add_argument('action', choices=['status', 'step', 'publish'])
    sp.add_parser('poll')
    su = sp.add_parser('supersede'); su.add_argument('task'); su.add_argument('--force-lease-release', action='store_true')
    ec = sp.add_parser('eng-checkpoint'); ec.add_argument('task'); ec.add_argument('--agent'); ec.add_argument('--next'); ec.add_argument('--hypotheses', default='[]')
    ec.add_argument('--decisions', default='[]'); ec.add_argument('--files', default='[]'); ec.add_argument('--unfinished', default='[]'); ec.add_argument('--verified', action='store_true')
    rc_ = sp.add_parser('root-cause'); rc_.add_argument('key'); rc_.add_argument('summary'); rc_.add_argument('--status', default='HYPOTHESIS', choices=['VERIFIED', 'HYPOTHESIS', 'REFUTED'])
    rc_.add_argument('--evidence'); rc_.add_argument('--task')
    a = ap.parse_args()
    if a.c == 'daemon':
        daemon(a)
    elif a.c == 'status':
        cmd_status(a)
    elif a.c == 'enqueue':
        db = connect()
        ok = enqueue(db, a.name, a.cmd, kind=a.kind, priority=a.priority, deps=json.loads(a.deps), resource=a.resource,
                     timeout_s=a.timeout, heartbeat_s=a.heartbeat, cwd=a.cwd, state=a.state, reason=a.reason)
        if a.klass:
            db.execute('UPDATE tasks SET klass=? WHERE name=?', (a.klass, a.name))
        if a.requires_result:
            db.execute('UPDATE tasks SET requires_result=1 WHERE name=?', (a.name,))
        db.execute('UPDATE tasks SET pin_model=?, pin_effort=?, base_from=?, task_type=?, complexity=?, risk=? WHERE name=?', (a.model, a.effort, a.base_from, a.task_type, a.complexity, a.risk, a.name))
        checkpoint(db, f'enqueue {a.name}')
        print('enqueued' if ok else 'exists (history kept)')
    elif a.c == 'mark':
        db = connect()
        db.execute('UPDATE tasks SET state=?, reason=? WHERE name=?', (a.state, a.reason, a.name)); ev(db, a.name, 'MARK', a.state)
        checkpoint(db, f'mark {a.name}')
    elif a.c == 'agent':
        db = connect(); st = agent_probe(db)
        st.setdefault(a.name, {}).update(status=a.status, retry_after=time.time() + a.retry_after_s); kv_set(db, 'agents', st)
        checkpoint(db, f'agent {a.name} {a.status}')
    elif a.c == 'checkpoint':
        checkpoint(connect(), 'manual')
    elif a.c == 'results':
        db = connect()
        q = 'SELECT * FROM results' + (' WHERE task=?' if a.task else '') + ' ORDER BY id'
        rows = [dict(r) for r in db.execute(q, (a.task,) if a.task else ())]
        if a.json:
            print(json.dumps(rows, indent=1))
        for r in rows:
            if not a.json:
                print(f"{r['id']:4} {r['verdict']:10} {r['status']:16} sv={r['safety_violations']} {r['task']}#{r['attempt']} {r['scenario_id']} seed={r['seed']} sha={str(r['git_sha'])[:9]}{' DIRTY' if r['code_dirty'] else ''}")
    elif a.c == 'mergeq':
        db = connect()
        if a.action == 'status':
            print(json.dumps([dict(r) for r in db.execute('SELECT id,task,branch,substr(sha,1,12) sha,state,detail FROM merge_queue ORDER BY id')], indent=1))
        elif a.action == 'step':
            lk = open(RUN / 'orchestrator.lock', 'a')
            try:
                fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                sys.exit('the daemon owns the lease and advances the merge queue by itself')
            while mergeq.step(db, RUN, lambda t, e, d='': ev(db, t, e, d)):
                time.sleep(0.5)
            print('idle')
        else:
            r_ = db.execute('SELECT repo FROM merge_queue ORDER BY id DESC LIMIT 1').fetchone()
            print(mergeq.publish(r_['repo'] if r_ else str(ROOT)))
    elif a.c == 'usage':
        db = connect(); print(json.dumps({k: v.get('usage') for k, v in refresh_usage(db, force=True).items()}, indent=1))
    elif a.c == 'poll':
        db = connect()
        lk = open(RUN / 'orchestrator.lock', 'a')
        try:
            fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)  # one writer of delegated state at a time
        except OSError:
            sys.exit('the daemon owns the lease and polls by itself; not polling from a second process')
        poll_delegated(db, force=True); checkpoint(db, 'poll')
    elif a.c == 'supersede':
        db = connect()
        if not a.force_lease_release:
            sys.exit('refused: supersede releases the worktree lease while the old Codex delivery may still run later. Re-run with --force-lease-release if you accept that; a late result of the old attempt is rejected (attempt_id) but its file edits are not prevented.')
        r = db.execute("SELECT * FROM tasks WHERE name=? AND state='DELEGATED'", (a.task,)).fetchone()
        if not r:
            sys.exit('no such DELEGATED task')
        db.execute("UPDATE tasks SET state='PENDING', phase='SUPERSEDED', attempt_id=NULL, max_attempts=max_attempts+1, reason='superseded by operator; old attempt result will be rejected' WHERE id=?", (r['id'],))
        ev(db, a.task, 'SUPERSEDED', f"operator forced; old attempt_id {r['attempt_id']} will be rejected")
        checkpoint(db, f'supersede {a.task}')
    elif a.c == 'verify-evidence':
        db = connect(); rep = store.adopt_orphans(db, RUN); checkpoint(db, 'verify-evidence')
        print(json.dumps(rep)); sys.exit(1 if rep['damaged'] else 0)
    elif a.c == 'eng-checkpoint':
        db = connect()
        store.save_engineering_checkpoint(db, a.task, 'agent', agent=a.agent, git_sha=git('rev-parse', 'HEAD'), worktree=str(ROOT), hypotheses=json.loads(a.hypotheses),
                                          decisions=json.loads(a.decisions), files_changed=json.loads(a.files), unfinished=json.loads(a.unfinished), next_step=a.next, verified=a.verified)
        checkpoint(db, f'eng-checkpoint {a.task}')
    elif a.c == 'root-cause':
        db = connect()
        print('stored' if store.add_root_cause(db, a.key, a.summary, a.status, a.evidence, a.task) else 'kept existing VERIFIED cause')
        checkpoint(db, f'root-cause {a.key}')


if __name__ == '__main__':
    main()
