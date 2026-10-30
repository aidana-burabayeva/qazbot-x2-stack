#!/usr/bin/env python3
"""Zero-loss result store for the nightly lab (SIM ONLY, stdlib only). Independent of any AI agent.

Contract: evidence is published atomically on disk FIRST (tmp dir -> fsync -> rename), and only then the SQLite
index row and the task state change in ONE transaction. A crash anywhere leaves either nothing, an adoptable
evidence dir (self-describing MANIFEST.json), or a complete record. Nothing is ever deleted: failed attempts are
kept, incomplete publishes are moved to evidence/_quarantine.

Fault injection for tests: env NQ_FAULT=<point> kills the process (exit 137) at that point.
Points: before_result_write, during_evidence, after_evidence_before_commit, after_commit.
"""
import hashlib
import json
import os
import shutil
import time
from pathlib import Path

VERDICTS = ('PASS', 'FAIL', 'UNVERIFIED')

SCHEMA = """
CREATE TABLE IF NOT EXISTS results(
  id INTEGER PRIMARY KEY, result_key TEXT UNIQUE NOT NULL, run_id TEXT NOT NULL, task TEXT NOT NULL, attempt INTEGER NOT NULL,
  scenario_id TEXT NOT NULL, seed TEXT, git_sha TEXT, code_dirty INTEGER NOT NULL DEFAULT 0, map_sha TEXT, config_sha TEXT,
  verdict TEXT NOT NULL, metrics TEXT NOT NULL DEFAULT '{}', safety_violations INTEGER NOT NULL DEFAULT 0, notes TEXT,
  evidence_path TEXT, manifest_sha256 TEXT, status TEXT NOT NULL DEFAULT 'STORED', ts REAL);
CREATE TABLE IF NOT EXISTS eng_checkpoints(
  id INTEGER PRIMARY KEY, task TEXT NOT NULL, ts REAL, agent TEXT, source TEXT NOT NULL, git_sha TEXT, worktree TEXT,
  hypotheses TEXT, decisions TEXT, files_changed TEXT, unfinished TEXT, next_step TEXT, verified INTEGER NOT NULL DEFAULT 0, detail TEXT);
CREATE TABLE IF NOT EXISTS root_causes(
  id INTEGER PRIMARY KEY, key TEXT UNIQUE NOT NULL, summary TEXT NOT NULL, status TEXT NOT NULL, evidence TEXT, task TEXT, ts REAL);
"""


def fault(point):
    if os.environ.get('NQ_FAULT') == point:
        os._exit(137)


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def fsync_dir(p):
    fd = os.open(p, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def init(db):
    db.executescript(SCHEMA)


def make_key(task, attempt, scenario_id):
    return f'{task}|{attempt}|{scenario_id}'


def run_id_for(task, attempt):
    return hashlib.sha1(f'{task}|{attempt}'.encode()).hexdigest()[:16]


def validate(items):
    """Reject results that could produce a false PASS. Returns (clean_items, errors)."""
    errs, clean = [], []
    if isinstance(items, dict):
        items = [items]
    if not isinstance(items, list) or not items:
        return [], ['result.json must be an object or a non-empty list of objects']
    seen = set()
    for i, it in enumerate(items):
        if not isinstance(it, dict):
            errs.append(f'#{i}: not an object'); continue
        sid = it.get('scenario_id')
        if not sid or not isinstance(sid, str):
            errs.append(f'#{i}: scenario_id missing'); continue
        if sid in seen:
            errs.append(f'#{i}: duplicate scenario_id {sid}'); continue
        seen.add(sid)
        if it.get('verdict') not in VERDICTS:
            errs.append(f'{sid}: verdict must be one of {VERDICTS}'); continue
        if not isinstance(it.get('metrics'), dict):
            errs.append(f'{sid}: metrics must be an object'); continue
        sv = it.get('safety_violations', 0)
        if not isinstance(sv, int) or sv < 0:
            errs.append(f'{sid}: safety_violations must be a non-negative int'); continue
        it = dict(it)
        it['notes'] = it.get('notes') or ''
        if it['verdict'] == 'PASS' and sv > 0:
            it['verdict'] = 'FAIL'
            it['notes'] += f' [forced FAIL: PASS with safety_violations={sv}]'
        if it['verdict'] == 'PASS' and it.get('seed') is None:
            it['verdict'] = 'UNVERIFIED'
            it['notes'] += ' [PASS without seed is not reproducible: UNVERIFIED]'
        clean.append(it)
    return clean, errs


def publish(run, task, attempt, src_dir, meta):
    """Atomically publish src_dir as evidence/<run_id>/ with MANIFEST.json. Idempotent for the same (task, attempt).
    Returns (evidence_path, manifest_sha256)."""
    ev_root = Path(run) / 'evidence'
    ev_root.mkdir(parents=True, exist_ok=True)
    rid = run_id_for(task, attempt)
    final = ev_root / rid
    if final.exists():
        return str(final), sha256_file(final / 'MANIFEST.json')  # same (task, attempt) already published: idempotent
    tmp = ev_root / f'.tmp-{rid}-{os.getpid()}'
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir()
    files = {}
    fault('before_result_write')
    for p in sorted(Path(src_dir).rglob('*')):
        if p.is_file():
            rel = str(p.relative_to(src_dir))
            dst = tmp / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, dst)
            with open(dst, 'rb') as f:
                os.fsync(f.fileno())
            files[rel] = sha256_file(dst)
            if len(files) == 1:
                fault('during_evidence')
    man = dict(run_id=rid, task=task, attempt=attempt, files=files, created=time.time(), meta=meta)
    mp = tmp / 'MANIFEST.json'
    mp.write_text(json.dumps(man, indent=1, sort_keys=True))
    with open(mp, 'rb') as f:
        os.fsync(f.fileno())
    fsync_dir(tmp)
    os.rename(tmp, final)  # atomic: either the full evidence dir exists or nothing does
    fsync_dir(ev_root)
    fault('after_evidence_before_commit')
    return str(final), sha256_file(final / 'MANIFEST.json')


def commit(db, task_id, items, ev_path, man_sha, base, final_state, exit_code, reason, signature, now=None):
    """One transaction: result rows + task state. Evidence already durable on disk."""
    now = now or time.time()
    db.execute('BEGIN IMMEDIATE')
    try:
        for it in items:
            key = make_key(base['task'], base['attempt'], it['scenario_id'])
            db.execute('INSERT OR IGNORE INTO results(result_key,run_id,task,attempt,scenario_id,seed,git_sha,code_dirty,map_sha,config_sha,verdict,metrics,safety_violations,notes,evidence_path,manifest_sha256,ts) '
                       'VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                       (key, run_id_for(base['task'], base['attempt']), base['task'], base['attempt'], it['scenario_id'],
                        None if it.get('seed') is None else str(it['seed']), base.get('git_sha'), int(bool(base.get('code_dirty'))),
                        it.get('map_sha'), it.get('config_sha') or base.get('config_sha'), it['verdict'], json.dumps(it['metrics'], sort_keys=True),
                        it.get('safety_violations', 0), it.get('notes'), ev_path, man_sha, now))
        db.execute('UPDATE tasks SET state=?, exit_code=?, pid=NULL, finished=?, signature=?, reason=? WHERE id=?',
                   (final_state, exit_code, now, signature, reason, task_id))
        db.execute('COMMIT')
    except BaseException:
        db.execute('ROLLBACK')
        raise
    fault('after_commit')


def adopt_orphans(db, run):
    """Crash recovery. Evidence dir with valid manifest but no index -> adopt (rebuild rows from the manifest).
    Partial tmp dirs -> quarantined (never deleted). Index rows whose evidence vanished/changed -> EVIDENCE_MISSING."""
    ev_root = Path(run) / 'evidence'
    report = dict(adopted=[], quarantined=[], damaged=[])
    if not ev_root.exists():
        return report
    q = ev_root / '_quarantine'
    for d in sorted(ev_root.iterdir()):
        if d.name == '_quarantine':
            continue
        if d.name.startswith('.tmp-'):
            q.mkdir(exist_ok=True)
            dest = q / f'{d.name}.{int(time.time())}'
            os.rename(d, dest)
            report['quarantined'].append(dest.name)
            continue
        mp = d / 'MANIFEST.json'
        if not mp.exists():
            q.mkdir(exist_ok=True)
            os.rename(d, q / f'{d.name}.nomanifest.{int(time.time())}')
            report['quarantined'].append(d.name)
            continue
        if db.execute('SELECT 1 FROM results WHERE run_id=? LIMIT 1', (d.name,)).fetchone():
            continue
        man = json.loads(mp.read_text())
        bad = [f for f, h in man['files'].items() if not (d / f).exists() or sha256_file(d / f) != h]
        if bad:
            report['damaged'].append((d.name, bad))
            continue
        meta = man.get('meta') or {}
        items = meta.get('items') or []
        if not items:
            continue
        base = meta['base']
        t = db.execute('SELECT id FROM tasks WHERE name=?', (man['task'],)).fetchone()
        if not t:
            continue
        commit(db, t['id'], items, str(d), sha256_file(mp), base, meta.get('final_state', 'COMPLETED'), meta.get('exit_code'),
               'adopted evidence after crash (index was missing)', meta.get('signature'))
        report['adopted'].append(d.name)
    for r in db.execute("SELECT DISTINCT run_id, evidence_path, manifest_sha256, task FROM results WHERE status='STORED'").fetchall():
        p = Path(r['evidence_path'])
        mp = p / 'MANIFEST.json'
        ok = mp.exists() and sha256_file(mp) == r['manifest_sha256']
        if ok:
            man = json.loads(mp.read_text())
            ok = all((p / f).exists() and sha256_file(p / f) == h for f, h in man['files'].items())
        if not ok:
            db.execute("UPDATE results SET status='EVIDENCE_MISSING' WHERE run_id=?", (r['run_id'],))
            db.execute("UPDATE tasks SET state='NEEDS_ENGINEERING', reason='evidence damaged or missing: result not trusted' WHERE name=? AND state='COMPLETED'", (r['task'],))
            report['damaged'].append((r['run_id'], 'verify failed'))
    return report


def save_engineering_checkpoint(db, task, source, agent=None, git_sha=None, worktree=None, hypotheses=None, decisions=None,
                                files_changed=None, unfinished=None, next_step=None, verified=False, detail=None):
    j = lambda v: json.dumps(v) if v is not None else None  # noqa: E731
    db.execute('INSERT INTO eng_checkpoints(task,ts,agent,source,git_sha,worktree,hypotheses,decisions,files_changed,unfinished,next_step,verified,detail) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)',
               (task, time.time(), agent, source, git_sha, worktree, j(hypotheses), j(decisions), j(files_changed), j(unfinished), next_step, int(verified), (detail or '')[:4000]))


def add_root_cause(db, key, summary, status, evidence=None, task=None):
    """status: VERIFIED | HYPOTHESIS | REFUTED. A VERIFIED cause is never downgraded by a later hypothesis."""
    cur = db.execute('SELECT status FROM root_causes WHERE key=?', (key,)).fetchone()
    if cur and cur['status'] == 'VERIFIED' and status != 'VERIFIED':
        return False
    db.execute('INSERT INTO root_causes(key,summary,status,evidence,task,ts) VALUES(?,?,?,?,?,?) ON CONFLICT(key) DO UPDATE SET summary=excluded.summary,status=excluded.status,evidence=excluded.evidence,task=excluded.task,ts=excluded.ts',
               (key, summary, status, evidence, task, time.time()))
    return True


def known_causes_text(db):
    rows = db.execute("SELECT key,summary,status,evidence FROM root_causes WHERE status IN ('VERIFIED','REFUTED') ORDER BY ts").fetchall()
    if not rows:
        return ''
    return 'ALREADY ESTABLISHED (do not re-investigate; re-check only if new evidence contradicts):\n' + \
        '\n'.join(f"- [{r['status']}] {r['key']}: {r['summary']} (evidence: {r['evidence'] or 'n/a'})" for r in rows)
