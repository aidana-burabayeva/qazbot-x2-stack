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
    db.execute('PRAGMA synchronous=NORMAL')
    db.executescript(SCHEMA)
    store.init(db)
    cols = [r[1] for r in db.execute('PRAGMA table_info(tasks)')]
    if 'requires_result' not in cols:
        db.execute('ALTER TABLE tasks ADD COLUMN requires_result INTEGER NOT NULL DEFAULT 0')  # 1: no result.json -> never COMPLETED
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
    busy = {r['resource'] for r in db.execute("SELECT resource FROM tasks WHERE state='RUNNING' AND resource IS NOT NULL")}
    now = time.time()
    for r in db.execute("SELECT * FROM tasks WHERE state='PENDING' AND not_before<=? ORDER BY priority, id", (now,)):
        if not all(d in done for d in json.loads(r['deps'])):
            continue
        if r['resource'] and r['resource'] in busy:
            continue
        yield r


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
    kv_set(db, f'base:{name}:{attempt}', base)
    ev(db, name, 'START', r['cmd'])
    checkpoint(db, f'start {name}')
    with open(log, 'wb') as fh:
        p = subprocess.Popen(['bash', '-lc', r['cmd']], cwd=r['cwd'], stdout=fh, stderr=subprocess.STDOUT, start_new_session=True,
                             env=dict(os.environ, NIGHTLY_RESULT_DIR=str(rdir), NIGHTLY_TASK=name, NIGHTLY_ATTEMPT=str(attempt)))
    db.execute('UPDATE tasks SET pid=? WHERE id=?', (p.pid, r['id']))
    return p, log


def eng_autocheckpoint(db, r, source, why):
    """Engineering checkpoint written by ordinary code when an agent ends without writing its own. Never 'verified'."""
    cwd = r['cwd'] or str(ROOT)

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
        v = subprocess.run(['bash', '-lc', f'{a} --version'], capture_output=True, text=True).stdout.strip()
        cur = st.get(a, {})
        cur.update(version=v, installed=bool(v))
        cur.setdefault('status', 'unknown')
        st[a] = cur
    kv_set(db, 'agents', st)
    return st


def run_engineering(db, r):
    """Engineering task = headless agent call. Returns an agent that is installed and not in a recorded usage limit, else None."""
    st = agent_probe(db)
    now = time.time()
    for a in db_get_agents(r):
        s = st.get(a, {})
        limited = s.get('status') == 'usage_limited' and s.get('retry_after', 0) > now
        if s.get('installed') and not limited:
            return a
    return None


def db_get_agents(r):
    m = re.search(r'--agents=(\S+)', r['repro'] or '')
    return m.group(1).split(',') if m else ['codex', 'claude']


def tick(db, cur):
    """One scheduling step. cur = (row, Popen, log, last_size, last_progress) or None. Returns the new cur."""
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
            a = run_engineering(db, r)
            if a is None:
                db.execute("UPDATE tasks SET state='PAUSED_AGENT', reason='no agent available (limit or not installed)' WHERE id=?", (r['id'],))
                ev(db, r['name'], 'PAUSED_AGENT', 'waiting for agent availability')
                checkpoint(db, f"paused {r['name']}")
                continue
            r = dict(r) | {'cmd': agent_command(a, r)}
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


def agent_command(agent, r):
    prompt = f"{r['repro']}\n{agent_context(r)}\nRules: simulation only; never contact the physical X2 (no ssh/deploy/ros2 pub); do not weaken safety limits or goal tolerances; "\
             "work only in your own git worktree; on usage limit stop, write docs/nightly/HANDOFF.md and exit."
    q = json.dumps(prompt)
    if agent == 'codex':
        # `codex queue` hands the task to the EXISTING session (an open TUI holds the writer lock, so `exec resume` is refused:
        # "already has an active writer"). No duplicate session, no limit bypass: Codex reads it when its quota is back.
        return f"codex queue --thread {os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')} --message {q}"
    return f"claude -p {q} --permission-mode acceptEdits --output-format text"


def revive_agents(db):
    """PAUSED_AGENT tasks go back to PENDING when an agent is believed available again (retry_after elapsed)."""
    st = kv_get(db, 'agents', {})
    now = time.time()
    for a, s in st.items():
        if s.get('status') == 'usage_limited' and s.get('retry_after', 0) < now:
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
    while not stop:
        cur = tick(db, cur)
        if time.time() - last_revive > 300:
            revive_agents(db); last_revive = time.time()
        if cur is None and not args.keep_alive and not db.execute("SELECT 1 FROM tasks WHERE state IN ('PENDING','RUNNING') LIMIT 1").fetchone():
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
    d = sp.add_parser('daemon'); d.add_argument('--poll', type=float, default=2.0); d.add_argument('--keep-alive', action='store_true')
    e = sp.add_parser('enqueue'); e.add_argument('name'); e.add_argument('cmd'); e.add_argument('--kind', default='mechanical')
    e.add_argument('--priority', type=int); e.add_argument('--deps', default='[]'); e.add_argument('--resource'); e.add_argument('--timeout', type=int)
    e.add_argument('--heartbeat', type=int); e.add_argument('--cwd'); e.add_argument('--state'); e.add_argument('--reason'); e.add_argument('--klass'); e.add_argument('--requires-result', action='store_true')
    sp.add_parser('status')
    m = sp.add_parser('mark'); m.add_argument('name'); m.add_argument('state'); m.add_argument('--reason', default='')
    ag = sp.add_parser('agent'); ag.add_argument('name'); ag.add_argument('status'); ag.add_argument('--retry-after-s', type=int, default=3600)
    sp.add_parser('checkpoint')
    rl = sp.add_parser('results'); rl.add_argument('--task'); rl.add_argument('--json', action='store_true')
    sp.add_parser('verify-evidence')
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
