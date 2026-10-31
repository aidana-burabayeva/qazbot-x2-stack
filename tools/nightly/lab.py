#!/usr/bin/env python3
"""QazBot nightly lab: operator commands stop / resume / status (SIM ONLY, stdlib only, no AI agent needed).

  lab.py stop    FREEZE STOP CHECKPOINT DOCUMENT VALIDATE COMMIT MERGE PUSH VERIFY
  lab.py resume  RESTORE FETCH SAFE-PULL DIFF TRIAGE PRIORITIZE START
  lab.py status  read-only

Never: force push, reset, clean, stash, rebase, sudo, ssh, contact with the physical X2.
Stop is idempotent. Publication state is explicit: STOPPED_OK | STOPPED_PUBLISH_PENDING. Codex acknowledgement
is STOP_UNVERIFIED until Codex writes the ack file it was asked to write.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nq  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'operator_skills'))
import opskills  # noqa: E402

ROOT, RUN = nq.ROOT, nq.RUN
HW = ROOT / 'docs' / 'hardware'
UNITS = ['qazbot-nightly.service', 'qazbot-nightly-enqueue.service', 'qazbot-nightly-enqueue.timer']
UNIT_DIR = Path(os.environ.get('NIGHTLY_UNIT_DIR', Path.home() / '.config' / 'systemd' / 'user'))
ACK = RUN / 'ack' / 'codex_stop.txt'
CODEX_THREAD = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
# stop commits only touch these; anything else dirty is reported and left alone (it may belong to someone else)
COMMIT_ALLOW = ('docs/', 'tools/nightly/', 'tests/nightly/', '.claude/skills/', 'CLAUDE.md', '.gitignore')
BLOCK_PATH = re.compile(r'(^|/)(run/|__pycache__/)|\.(db|db-wal|db-shm|log|db3|mcap|bag|pem|key)$|(^|/)\.env$')
SECRET = re.compile(r'-----BEGIN [A-Z ]*PRIVATE KEY|ghp_[A-Za-z0-9]{20,}|sk-[A-Za-z0-9]{20,}|\b(password|passwd|secret|token|api_?key)\b\s*[=:]\s*[\'"]?[A-Za-z0-9/+_.-]{8,}[\'"]?\s*$', re.I | re.M)
REGRESSION = os.environ.get('NIGHTLY_REGRESSION',
                            'source /opt/ros/humble/setup.bash 2>/dev/null; python3 -m pytest -q -p no:cacheprovider tests/nightly tests/motion_adapter')
USAGE_ROBOT_PATHS = ('docs/hardware/ROBOT_TO_LAB_FEEDBACK.md', 'docs/hardware/ROBOT_FINDINGS.md', 'docs/evidence/robot_', 'docs/HANDOFF.md', 'docs/TEST_STATUS.md')


def sh(cmd, timeout=120, cwd=None):
    r = subprocess.run(cmd if isinstance(cmd, list) else ['bash', '-lc', cmd], capture_output=True, text=True, timeout=timeout, cwd=cwd or ROOT)
    return r.returncode, (r.stdout + r.stderr).strip()


def git(*a, timeout=60):
    return sh(['git', '-C', str(ROOT), *a], timeout)


def porcelain():
    r = subprocess.run(['git', '-C', str(ROOT), 'status', '--porcelain', '-uall'], capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]  # no strip(): the leading status column is significant


def gout(*a):
    rc, o = git(*a)
    return o if rc == 0 else ''


def now_iso():
    return time.strftime('%Y-%m-%dT%H:%M:%S%z')


def replace_block(path, tag, body):
    """Idempotent generated section: <!-- BEGIN tag --> ... <!-- END tag -->. Hand-written text outside is never touched."""
    b, e = f'<!-- BEGIN {tag} -->', f'<!-- END {tag} -->'
    text = path.read_text() if path.exists() else ''
    block = f'{b}\n{body.rstrip()}\n{e}'
    if b in text and e in text:
        text = re.sub(re.escape(b) + r'.*?' + re.escape(e), lambda _m: block, text, flags=re.S)
    else:
        text = text.rstrip() + ('\n\n' if text else '') + block + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


# ------------------------------------------------------------------ state

def state_get(db):
    return {k: nq.kv_get(db, k) for k in ('lab_state', 'last_lab_stop_sha', 'last_robot_integrated_sha', 'current_integration_sha',
                                           'codex_stop', 'frozen', 'last_stop_ts', 'last_resume_ts')}


def counts(db):
    return {r['state']: r['n'] for r in db.execute('SELECT state, COUNT(*) n FROM tasks GROUP BY state')}


def unit_state(u):
    rc, o = sh(f'systemctl --user is-enabled {u}; systemctl --user is-active {u}')
    return o.replace('\n', '/')


def docker_projects():
    rc, o = sh("docker ps --format '{{.Names}}' 2>/dev/null")
    return [n for n in o.split() if n.startswith('x2-twin')]


def git_view():
    br = gout('rev-parse', '--abbrev-ref', 'HEAD')
    up = gout('rev-parse', '--abbrev-ref', '@{u}')
    ahead = behind = None
    if up:
        a = gout('rev-list', '--left-right', '--count', f'HEAD...{up}').split()
        if len(a) == 2:
            ahead, behind = int(a[0]), int(a[1])
    dirty = porcelain()
    return dict(branch=br, head=gout('rev-parse', 'HEAD'), upstream=up, ahead=ahead, behind=behind, dirty=dirty,
                worktrees=[ln for ln in gout('worktree', 'list').splitlines()])


# ------------------------------------------------------------------ status

def cmd_status(args):
    db = nq.connect()
    codex_ack_status(db)
    st = state_get(db)
    g = git_view()
    out = dict(lab_state=st['lab_state'] or 'UNKNOWN', frozen=bool(st['frozen']), shas={k: st[k] for k in ('last_lab_stop_sha', 'last_robot_integrated_sha', 'current_integration_sha')},
               codex_stop=st['codex_stop'], queue=counts(db), units={u: unit_state(u) for u in UNITS}, docker=docker_projects(), git=g,
               codex_ack=ACK.read_text().strip() if ACK.exists() else None)
    if args.json:
        print(json.dumps(out, indent=1, default=str))
        return
    print(f"lab: {out['lab_state']}  frozen={out['frozen']}  codex_stop={out['codex_stop']}  ack={out['codex_ack']}")
    print('shas :', out['shas'])
    print('queue:', out['queue'])
    print('units:', out['units'])
    print('docker x2-twin containers:', out['docker'] or 'none')
    print(f"git  : {g['branch']} {g['head'][:9]} up={g['upstream']} ahead={g['ahead']} behind={g['behind']} dirty={len(g['dirty'])}")
    if g['dirty']:
        print('       ' + '\n       '.join(g['dirty'][:12]))
    print('Not controllable from here: other open Claude/Codex TUI sessions, session-only /loop cron jobs and /goal hooks of open Claude sessions.')


# ------------------------------------------------------------------ stop

def freeze(db):
    nq.kv_set(db, 'frozen', True)
    nq.kv_set(db, 'lab_state', 'STOPPING')
    nq.ev(db, '*', 'FREEZE', 'operator stop')


def stop_units(args):
    """Timer first: removing a unit link while its timer still points at it makes the timer fail ('Unit to trigger vanished')."""
    res = {}
    if args.no_systemd:
        return {'skipped': 'no-systemd'}
    order = [UNITS[2], UNITS[1], UNITS[0]]
    for u in order:
        sh(f'systemctl --user disable --now {u}')
    for u in order:
        link = UNIT_DIR / u
        if link.is_symlink():
            link.unlink()
    sh('systemctl --user daemon-reload')
    sh('systemctl --user reset-failed ' + ' '.join(UNITS))
    for u in UNITS:
        res[u] = unit_state(u)
    return res


def park_tasks(db):
    """Running tasks: kill their process groups, requeue-on-resume. Pending/paused: parked as-is. Previous state kept for resume."""
    parked = nq.kv_get(db, 'parked', {})
    for r in db.execute("SELECT id FROM tasks WHERE state='VERIFYING'").fetchall():  # verification jobs are children we started: stop them, resume restarts them
        vp = nq.kv_get(db, f"verifpid:{r['id']}")
        if vp:
            try:
                os.killpg(vp, __import__('signal').SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    for r in db.execute("SELECT * FROM tasks WHERE state IN ('RUNNING','PENDING','PAUSED_AGENT','INTERRUPTED')").fetchall():
        if r['state'] == 'RUNNING' and r['pid']:
            for sig in ('SIGTERM', 'SIGKILL'):
                try:
                    os.killpg(r['pid'], getattr(__import__('signal'), sig))
                except (ProcessLookupError, PermissionError):
                    break
                time.sleep(1.5)
        prev = 'PENDING' if r['state'] in ('RUNNING', 'INTERRUPTED') else r['state']
        parked[r['name']] = prev
        db.execute("UPDATE tasks SET state='NEEDS_APPROVAL', pid=NULL, reason=?, attempts=MAX(attempts-?,0) WHERE id=?",
                   (f"LAB_STOP (was {r['state']})", 1 if r['state'] == 'RUNNING' else 0, r['id']))
        nq.ev(db, r['name'], 'PARKED', r['state'])
    nq.kv_set(db, 'parked', parked)
    return parked


def stop_sim(args):
    if args.no_docker:
        return 'skipped'
    names = docker_projects()
    if names:
        sh('docker compose -p x2-twin down', timeout=180)
    return {'before': names, 'after': docker_projects()}


def notify_codex(db, args, sha):
    if args.no_notify:
        nq.kv_set(db, 'codex_stop', 'STOP_UNVERIFIED')
        return 'not sent (--no-notify): STOP_UNVERIFIED'
    ACK.parent.mkdir(parents=True, exist_ok=True)
    ACK.unlink(missing_ok=True)
    msg = (f'OPERATOR STOP {now_iso()}. Stop all work now: no new sim runs, no commits to shared branches, no contact with the physical X2. '
           f'Finish the atomic step, commit only in your own worktree, then acknowledge by writing "STOPPED <your HEAD sha>" to {ACK}. '
           f'Lab HEAD at stop: {sha}. Do not resume until the operator runs /nightly-resume.')
    try:
        rc, o = sh([nq.agents.ADAPTERS['codex'].bin, 'queue', '--thread', CODEX_THREAD, '--message', msg], timeout=45)  # adapter bin: tests point it at a stub
    except (OSError, subprocess.TimeoutExpired) as ex:  # agent missing/hung must never block the stop
        rc, o = 127, f'codex unavailable: {ex}'
    nq.kv_set(db, 'codex_stop', 'STOP_UNVERIFIED' if rc == 0 else 'STOP_NOT_DELIVERED')
    return f'rc={rc} {o[:200]}'


def codex_ack_status(db):
    """An ack file proves the Codex SESSION stopped. A delegated attempt still recorded as open overrides it: not proven ended."""
    if db.execute("SELECT 1 FROM tasks WHERE state='DELEGATED' LIMIT 1").fetchone():
        nq.kv_set(db, 'codex_stop', 'STOP_UNVERIFIED')
        return 'STOP_UNVERIFIED'
    if ACK.exists() and ACK.read_text().startswith('STOPPED'):
        nq.kv_set(db, 'codex_stop', 'STOP_ACKED')
        return 'STOP_ACKED'
    return nq.kv_get(db, 'codex_stop', 'UNKNOWN')


def robot_doc_data():
    p = HW / 'lab_robot_state.json'
    return json.loads(p.read_text()) if p.exists() else {}


def render_handoff(db, g, tests):
    d = robot_doc_data()
    head, br = g['head'], g['branch']
    contract = dict(contract_version=opskills.load_manifest()['contract_version'], doc='LAB_TO_ROBOT_HANDOFF', lab_branch=br,
                    lab_sha=head, generated=now_iso(), package_version=opskills.load_manifest()['version'])
    L = [f'<!-- QAZBOT-CONTRACT {json.dumps(contract)} -->', '', f'# Lab → Robot handoff', '',
         f'Сгенерировано `tools/nightly/lab.py stop` {now_iso()}. Ручные данные: `docs/hardware/lab_robot_state.json`. Сначала verify SHA ниже через `git ls-remote`.', '',
         '## Что доступно через Git', '',
         f'- Ветка: `{br}`, SHA на момент остановки: `{head}`', f'- Интеграция с production snapshot (`origin/main`): `{gout("rev-parse", "origin/main")}`',
         '- Тег остановки ставится после push: `lab-stop/<UTC>` (точный SHA в `docs/hardware/LAB_STATE.json` нельзя записать в тот же коммит; он в теге).', '',
         '## Тесты на момент остановки', '', f"- Regression: **{tests['verdict']}** — {tests['summary']}"]
    L += ['', '## Классы верификации (контракт: SIM_VERIFIED / HARDWARE_VERIFIED / UNVERIFIED)', '']
    cls = {'SIM_VERIFIED': [], 'SIM_FAILED': [], 'UNVERIFIED': [], 'NEEDS_HARDWARE': [], 'HARDWARE_VERIFIED': []}
    for t in d.get('tests', []):
        st = t['status'].upper()
        k = 'SIM_VERIFIED' if st.startswith('PASS') else 'SIM_FAILED' if st.startswith('FAIL') else 'NEEDS_HARDWARE' if 'HARDWARE' in st else 'UNVERIFIED'
        cls[k].append(f"{t['name']} — {t['note']}")
    for k, v in cls.items():
        L.append(f'**{k}**' + ('' if v else ': нет (HARDWARE_VERIFIED появляется только из ROBOT_TO_LAB_FEEDBACK.md после замеров на X2)' if k == 'HARDWARE_VERIFIED' else ': нет'))
        L += [f'- {x}' for x in v]
    for title, key in (('Затронутые Nav2 / ROS 2 параметры', 'params'), ('Известные ошибки (открыты)', 'open_bugs'), ('НЕЛЬЗЯ переносить на X2', 'do_not_transfer'),
                       ('Проверки на физическом роботе (только по разрешению оператора)', 'robot_checks'), ('Диагностика (команды; read-only на роботе)', 'diag_commands'),
                       ('Риски (TF, локализация, Nav2, MC, сенсоры)', 'risks')):
        L += ['', f'## {title}', '']
        L += [f'- {x}' if key != 'diag_commands' else f'    {x}' for x in d.get(key, ['OPEN: не заполнено'])]
    L += ['', '## Ограничения', '', '- Лаборатория не деплоит и не шлёт команд на X2. Любой результат симуляции — кандидат, не квалификация. Движение, SSH, правка конфигурации и deploy — только по отдельному разрешению оператора.', '']
    return '\n'.join(L)


def render_fidelity():
    d = robot_doc_data().get('fidelity_requests', [])
    L = ['# Twin fidelity requests (данные, нужные с физического X2)', '',
         'Статус каждого пункта `NEEDS_HARDWARE`, пока нет замера. Источник: расхождения и допущения симуляции (`lab_robot_state.json`). Не подгонять twin под один сценарий.', '']
    for i, r in enumerate(d, 1):
        L += [f"## FID-{i:02d}. {r['title']}", '', f"- Что уточняет: {r['hypothesis']}", f"- Замеры: {r['measurements']}", f"- Topics / TF / параметры / логи: {r['data']}",
              f"- Условия воспроизведения: {r['conditions']}", f"- Движение робота: {r['motion']}", f"- Ожидаемая точность: {r['accuracy']}",
              f"- Сопоставление robot ↔ twin: {r['mapping']}", '- Статус: NEEDS_HARDWARE', '']
    return '\n'.join(L)


def run_regression(args):
    if args.no_tests:
        return dict(verdict='SKIPPED', summary='--no-tests', rc=None)
    rc, o = sh(REGRESSION, timeout=900)
    last = o.splitlines()[-1] if o else ''
    return dict(verdict='PASS' if rc == 0 else 'FAIL', summary=last[:200], rc=rc)


def document(db, g, tests, parked, cstat, units):
    HW.mkdir(parents=True, exist_ok=True)
    (HW / 'LAB_TO_ROBOT_HANDOFF.md').write_text(render_handoff(db, g, tests))
    (HW / 'TWIN_FIDELITY_REQUESTS.md').write_text(render_fidelity())
    q = counts(db)
    body = (f"## LAB STOP {now_iso()}\n- HEAD `{g['head'][:12]}` ({g['branch']}); очередь {q}; запаркованных задач {len(parked)}.\n"
            f"- Codex: {cstat}. Units: {units}.\n- Regression: {tests['verdict']} ({tests['summary']}).\n"
            "- Не контролируется отсюда: открытые TUI Claude/Codex, session-only /loop cron и /goal hook открытых сессий.\n- Возобновление: `python3 tools/nightly/lab.py resume` (`/nightly-resume`).")
    for f in ('STATUS.md', 'HANDOFF.md'):
        replace_block(ROOT / 'docs' / 'nightly' / f, 'LAB_STOP', body)
    replace_block(ROOT / 'docs' / 'nightly' / 'DECISIONS.md', 'LAB_STOP', f"- {now_iso()}: оператор остановил лабораторию (`lab.py stop`); задачи запаркованы, расписания отключены, возобновление только `lab.py resume`.")
    replace_block(ROOT / 'docs' / 'TEST_STATUS.md', 'LAB_STOP_TESTS', f"### Nightly lab regression at stop ({now_iso()})\n- `{REGRESSION.split(';')[-1].strip()}`: **{tests['verdict']}** — {tests['summary']}")
    (HW / 'LAB_STATE.json').write_text(json.dumps(dict(stopped=now_iso(), branch=g['branch'], head_before_stop_commit=g['head'], codex_stop=cstat,
                                                         last_robot_integrated_sha=nq.kv_get(db, 'last_robot_integrated_sha'), queue=q), indent=1) + '\n')


def commit(args, g):
    """Stage only allow-listed paths; refuse runtime artefacts and secrets. Others' dirty files are reported, never touched."""
    skipped, staged = [], []
    for ln in porcelain():
        path = ln[3:].split(' -> ')[-1].strip('"')
        if BLOCK_PATH.search(path):
            skipped.append((path, 'runtime/blocked')); continue
        if not path.startswith(COMMIT_ALLOW):
            skipped.append((path, 'outside stop allow-list (left untouched)')); continue
        staged.append(path)
    if not staged:
        return dict(commit=None, staged=[], skipped=skipped)
    git('add', '--', *staged)
    diff = gout('diff', '--cached', '-U0')
    added = '\n'.join(ln for ln in diff.splitlines() if ln.startswith('+') and not ln.startswith('+++'))
    m = SECRET.search(added)
    if m:
        git('reset', '-q', '--', *staged)  # unstage only what we staged
        raise SystemExit(f'refused: possible secret in staged diff ({m.group(0)[:20]}…). Nothing committed.')
    rc, o = git('commit', '-q', '-m', f'lab-stop: operator stop, state, robot handoff ({now_iso()})\n\nGenerated by tools/nightly/lab.py stop. SIM ONLY.')
    return dict(commit=gout('rev-parse', 'HEAD') if rc == 0 else None, staged=staged, skipped=skipped, rc=rc, out=o[:300])


def push_and_verify(args, g):
    if args.no_push:
        return 'STOPPED_PUBLISH_PENDING', 'push skipped (--no-push)'
    br = gout('rev-parse', '--abbrev-ref', 'HEAD')
    head = gout('rev-parse', 'HEAD')
    rc, o = git('push', 'origin', f'HEAD:refs/heads/{br}', timeout=120)  # plain push: a non-ff is rejected, never forced
    if rc != 0:
        return 'STOPPED_PUBLISH_PENDING', f'push failed: {o[-300:]}'
    tag = 'lab-stop/' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    git('tag', '-a', tag, '-m', f'lab stop {head}', head)
    git('push', 'origin', tag, timeout=60)
    rc, ls = git('ls-remote', 'origin', f'refs/heads/{br}', f'refs/tags/{tag}^{{}}')
    remote = {ln.split()[1]: ln.split()[0] for ln in ls.splitlines() if ln.strip()}
    ok_branch = remote.get(f'refs/heads/{br}') == head
    git('fetch', '--prune', 'origin', br, timeout=60)
    docs_ok = git('cat-file', '-e', f'origin/{br}:docs/hardware/LAB_TO_ROBOT_HANDOFF.md')[0] == 0
    if ok_branch and docs_ok:
        return 'STOPPED_OK', f'remote {br}={head[:12]} tag={tag}'
    return 'STOPPED_PUBLISH_PENDING', f'verify failed: branch_ok={ok_branch} docs_ok={docs_ok}'


def cmd_stop(args):
    log = []
    db = nq.connect()
    freeze(db); log.append('FREEZE ok')
    units = stop_units(args); log.append(f'STOP units {units}')
    parked = park_tasks(db); log.append(f'STOP parked {len(parked)} tasks')
    open_deleg = [r['name'] for r in db.execute("SELECT name FROM tasks WHERE state='DELEGATED'")]
    if open_deleg:  # a delegated Codex attempt cannot be proven stopped from here: lease kept, never reported as stopped
        nq.kv_set(db, 'delegated_open', open_deleg)
        log.append(f'STOP DELEGATED_OPEN {open_deleg}: Codex attempt(s) not proven ended -> STOP_UNVERIFIED, lease kept')
    log.append(f'STOP sim {stop_sim(args)}')
    g = git_view()
    log.append(f'STOP codex {notify_codex(db, args, g["head"])}')
    nq.checkpoint(db, 'lab stop'); log.append('CHECKPOINT ok')
    tests = run_regression(args); log.append(f"VALIDATE {tests['verdict']} {tests['summary']}")
    e_, w_, _i = opskills.validate('lab'); log.append(f"VALIDATE operator-skills {'FAIL ' + str(e_) if e_ else 'OK'}")
    document(db, g, tests, parked, codex_ack_status(db), units); log.append('DOCUMENT ok')
    c = commit(args, g); log.append(f"COMMIT {c.get('commit') and c['commit'][:12]} staged={len(c['staged'])} skipped={c['skipped']}")
    log.append('MERGE n/a: merge queue is P3 (not implemented); the work branch is the integration branch and nothing is auto-merged to main')
    verdict, detail = push_and_verify(args, g); log.append(f'PUSH/VERIFY {verdict} {detail}')
    head = gout('rev-parse', 'HEAD')
    nq.kv_set(db, 'last_lab_stop_sha', head)
    nq.kv_set(db, 'last_stop_ts', time.time())
    nq.kv_set(db, 'lab_state', verdict)
    nq.checkpoint(db, 'lab stop done')
    print('\n'.join(log))
    print(f'last_lab_stop_sha={head}  state={verdict}  codex_stop={nq.kv_get(db, "codex_stop")}')
    if tests['verdict'] == 'FAIL':
        print('WARNING: regression FAIL at stop; documented as such.')
    sys.exit(0 if verdict == 'STOPPED_OK' else 3)


# ------------------------------------------------------------------ resume

def triage(paths, subjects):
    """Classify new upstream changes. Robot-side findings outrank lab optimisation."""
    tasks = []
    robot = [p for p in paths if p.startswith(USAGE_ROBOT_PATHS)]
    if robot:
        tasks.append(('triage_robot_findings', 'robot-side files changed: ' + ', '.join(robot[:6]), 'SIM_THEN_HARDWARE', 10))
    for s in subjects:
        if re.search(r'\b(blocker|fail|stall|bug|regress|collision|reject)\b', s, re.I):
            tasks.append(('triage_blocker_' + re.sub(r'\W+', '_', s)[:40].lower(), s, 'SIM_THEN_HARDWARE', 15))
    return tasks



ROBOT_AREAS = (('navigation', 'ws/src/navigation/'), ('lio', 'ws/src/lio/'), ('maps', 'ws/src/maps/'), ('stack', 'ws/src/'))
BLOCKER_RE = re.compile(r'blocker|блокер|\bFAIL\b|NEEDS_(HARDWARE|ENGINEERING|APPROVAL)|TWIN_MISMATCH|нестабил|unstable|регресс|regression', re.I)
FEEDBACK_FILES = ('docs/hardware/ROBOT_TO_LAB_FEEDBACK.md', 'docs/HANDOFF.md', 'docs/TEST_STATUS.md')


def latest_robot_ref():
    """Newest published robot baseline branch (origin/baseline/x2-robot-*), or the operator override NIGHTLY_ROBOT_REF."""
    ref = os.environ.get('NIGHTLY_ROBOT_REF')
    if ref:
        return ref if rev_ok(ref) else None
    out = gout('for-each-ref', '--sort=-committerdate', '--format=%(refname:short)', 'refs/remotes/origin/baseline/x2-robot-*')
    return out.splitlines()[0] if out else None


def rev_ok(ref):
    return git('rev-parse', '--verify', '-q', ref + '^{commit}')[0] == 0


def robot_sync(db):
    """Robot -> Lab: analyse only what the physical-robot branch published SINCE the last processed robot baseline.
    Reads with git show/diff from the remote ref (never merges, so diverged histories are harmless and the feedback file need not exist
    in the lab branch). The checkpoint moves only after every task was created."""
    log = []
    ref = latest_robot_ref()
    if not ref:
        return ['ROBOT-SYNC no robot baseline ref (origin/baseline/x2-robot-*) found: nothing to analyse (not an error)']
    tip = gout('rev-parse', ref + '^{commit}')
    last = nq.kv_get(db, 'last_robot_integrated_sha')
    if last and tip == last:
        return [f'ROBOT-SYNC {ref} {tip[:9]} already processed']
    valid_last = bool(last) and rev_ok(last)
    if valid_last:
        mb = gout('merge-base', last, tip)
        base = mb or last  # diverged robot history: compare from the common ancestor, still read-only
    else:
        base = gout('merge-base', 'HEAD', tip)  # first run: only what the robot added beyond the lab's own history
    if not base:
        return [f'ROBOT-SYNC {ref} shares no history with the lab branch; analysis skipped, checkpoint not moved']
    try:
        status = [ln.split('\t') for ln in gout('diff', '--name-status', '--no-renames', base, tip).splitlines() if ln.strip()]
        paths = [x[-1] for x in status]
        subjects = gout('log', '--format=%h %s', f'{base}..{tip}').splitlines()
        log.append(f'ROBOT-SYNC {ref} tip={tip[:9]} base={base[:9]} commits={len(subjects)} files={len(paths)} diverged_from_last={bool(valid_last and base != last)}')
        made = []
        areas = {}
        for name, prefix in ROBOT_AREAS[:3]:
            hit = [p for p in paths if p.startswith(prefix)]
            if hit:
                areas[name] = hit
        for name, hit in areas.items():
            tname = f'robot_parity_{name}_{tip[:7]}'
            text = (f'ROBOT SYNC: the physical-robot baseline {ref} ({tip[:9]}) changed installed {name} files since {base[:9]}: {", ".join(hit[:12])}. '
                    'Establish whether the digital twin / lab code reflects each change (intentional sim difference vs unintended drift), reproduce in simulation where possible, '
                    'and fix or document. Simulation only; never contact the robot. Do not change localization, P20 and costmap in one step.')
            if nq.enqueue(db, tname, 'true', kind='engineering', repro=text, priority=12, state='PENDING'):
                made.append(tname)
                db.execute('UPDATE tasks SET klass=?, task_type=? WHERE name=?', ('SIM_THEN_HARDWARE', 'nav2', tname))
        # blockers: added lines of robot-side status documents + the current feedback file (read from the robot ref, may be absent in the lab branch)
        lines = set()
        for f in FEEDBACK_FILES:
            diff = gout('diff', '-U0', base, tip, '--', f)
            lines |= {ln[1:].strip() for ln in diff.splitlines() if ln.startswith('+') and not ln.startswith('+++') and BLOCKER_RE.search(ln)}
        fb = gout('show', f'{tip}:docs/hardware/ROBOT_TO_LAB_FEEDBACK.md') if git('cat-file', '-e', f'{tip}:docs/hardware/ROBOT_TO_LAB_FEEDBACK.md')[0] == 0 else ''
        lines |= {ln.strip() for ln in fb.splitlines() if BLOCKER_RE.search(ln) and not ln.lstrip().startswith('<!--')}
        for ln in sorted(lines)[:20]:
            h = __import__('hashlib').sha1(ln.encode()).hexdigest()[:6]
            tname = f'robot_blocker_{h}'
            text = f'ROBOT-REPORTED BLOCKER (from {ref} {tip[:9]}): {ln[:400]}. Reproduce in simulation first, then establish the root cause; no fix before a reproduction. Simulation only.'
            if nq.enqueue(db, tname, 'true', kind='engineering', repro=text, priority=10, state='PENDING'):
                made.append(tname)
                db.execute('UPDATE tasks SET klass=?, task_type=? WHERE name=?', ('SIM_THEN_HARDWARE', 'root_cause_analysis', tname))
        if os.environ.get('NIGHTLY_TEST_FAIL_ROBOT_SYNC'):
            raise RuntimeError('injected failure after task creation')
        nq.kv_set(db, 'last_robot_integrated_sha', tip)  # only after everything above succeeded
        nq.kv_set(db, 'last_robot_ref', ref)
        log.append(f'ROBOT-SYNC created {len(made)} task(s): {made}; checkpoint last_robot_integrated_sha={tip[:9]}')
    except Exception as e:
        log.append(f'ROBOT-SYNC FAILED ({type(e).__name__}: {e}); checkpoint NOT advanced, will be retried on the next resume')
    return log


def cmd_resume(args):
    db = nq.connect()
    g0 = git_view()
    log = [f"RESTORE state={nq.kv_get(db, 'lab_state')} last_stop={nq.kv_get(db, 'last_lab_stop_sha')} queue={counts(db)}"]
    if not args.no_fetch:
        rc, o = git('fetch', '--prune', timeout=120)
        log.append(f'FETCH rc={rc} {o[-150:]}')
    g = git_view()
    divergence = None
    tracked_dirty = [ln for ln in g['dirty'] if not ln.startswith('??')]
    if g['upstream'] and g['behind']:
        if g['ahead']:
            divergence = f"diverged ahead={g['ahead']} behind={g['behind']}: no auto force/reset/rebase"
        elif tracked_dirty:
            divergence = f"behind={g['behind']} but worktree has {len(tracked_dirty)} tracked modification(s): pull skipped"
        else:
            rc, o = git('pull', '--ff-only', timeout=120)
            log.append(f'SAFE-PULL rc={rc} {o[-120:]}')
            g = git_view()
    log.append(f"GIT {g['branch']} {g['head'][:12]} ahead={g['ahead']} behind={g['behind']} dirty={len(g['dirty'])} divergence={divergence}")
    # DIFF since the last integrated point (stop sha, else the robot-integrated sha)
    base = nq.kv_get(db, 'last_lab_stop_sha')
    new_tasks = []
    if base and git('cat-file', '-e', base)[0] == 0:
        paths = gout('diff', '--name-only', f'{base}..HEAD').splitlines()
        subjects = gout('log', '--format=%s', f'{base}..HEAD').splitlines()
        subjects = [s for s in subjects if not s.startswith('lab-stop:')]
        paths = [p for p in paths if not p.startswith('docs/hardware/LAB_') and not p.startswith('docs/nightly/')]
        log.append(f'DIFF {base[:9]}..HEAD files={len(paths)} commits={len(subjects)}')
        new_tasks = triage(paths, subjects)
    else:
        log.append('DIFF no valid last_lab_stop_sha: nothing compared (UNVERIFIED baseline)')
    ups = g['upstream']
    if divergence:
        new_tasks.append((f"integrate_{g['head'][:9]}", divergence, 'NEEDS_APPROVAL', 5))
    for name, why, klass, prio in new_tasks:
        ok = nq.enqueue(db, f"{name}_{time.strftime('%Y%m%d')}", 'true', kind='mechanical', priority=prio,
                        state='NEEDS_APPROVAL' if klass == 'NEEDS_APPROVAL' else 'NEEDS_ENGINEERING', reason=why[:200])
        db.execute('UPDATE tasks SET klass=? WHERE name=?', (klass, f"{name}_{time.strftime('%Y%m%d')}"))
        log.append(f'TRIAGE {klass} p{prio} {name} ({"new" if ok else "exists"})')
    fb = opskills.contract_block(HW / 'ROBOT_TO_LAB_FEEDBACK.md')
    if fb:  # robot-side checkpoints written by /robot-sync-validate; the lab only reads them
        for k in ('last_lab_integrated_sha', 'last_lab_reviewed_sha'):
            if fb.get(k):
                nq.kv_set(db, 'robot_reported_' + k, fb[k])
        log.append(f"FEEDBACK robot reports integrated={str(fb.get('last_lab_integrated_sha'))[:9]} reviewed={str(fb.get('last_lab_reviewed_sha'))[:9]}")
    log.extend(robot_sync(db))
    nq.kv_set(db, 'current_integration_sha', g['head'])
    # PRIORITIZE/restore parked work (a git problem never blocks independent sim tasks, but results on stale code are flagged)
    parked = nq.kv_get(db, 'parked', {})
    restored = 0
    for name, prev in parked.items():
        n = db.execute("UPDATE tasks SET state=?, reason=? WHERE name=? AND state='NEEDS_APPROVAL' AND reason LIKE 'LAB_STOP%'",
                       (prev, 'restored by resume' + (f' [STALE-CODE: {divergence}]' if divergence else ''), name)).rowcount
        restored += n
    nq.kv_set(db, 'parked', {})
    nq.kv_set(db, 'frozen', False)
    nq.kv_set(db, 'delegated_open', [])
    nq.kv_set(db, 'lab_state', 'RUNNING' if not args.no_start else 'RESUMED_NOT_STARTED')
    nq.kv_set(db, 'last_resume_ts', time.time())
    log.append(f'PRIORITIZE restored {restored} parked tasks; frozen=False')
    nq.ev(db, '*', 'RESUME', f'restored={restored} divergence={divergence}')
    nq.checkpoint(db, 'lab resume')
    if args.no_start or args.no_systemd:
        log.append('START skipped (' + ('--no-start' if args.no_start else '--no-systemd') + ')')
    else:
        UNIT_DIR.mkdir(parents=True, exist_ok=True)
        for u in UNITS:
            link = UNIT_DIR / u
            src = ROOT / 'tools' / 'nightly' / 'systemd' / u
            if link.is_symlink() or link.exists():
                link.unlink()
            link.symlink_to(src)
        sh('systemctl --user daemon-reload')
        rc, o = sh('systemctl --user enable --now qazbot-nightly.service qazbot-nightly-enqueue.timer')
        log.append(f'START rc={rc} {o[-120:]} units={ {u: unit_state(u) for u in UNITS} }')
    print('\n'.join(log))
    print('Physical X2: no motion/SSH/deploy is performed or allowed by the lab; hardware qualification needs a separate operator confirmation.')


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='c', required=True)
    s = sp.add_parser('stop')
    for f in ('no-systemd', 'no-docker', 'no-notify', 'no-tests', 'no-push'):
        s.add_argument('--' + f, action='store_true')
    r = sp.add_parser('resume')
    for f in ('no-fetch', 'no-start', 'no-systemd'):
        r.add_argument('--' + f, action='store_true')
    st = sp.add_parser('status'); st.add_argument('--json', action='store_true')
    a = ap.parse_args()
    if a.c in ('stop', 'resume') and opskills.detect_role() != 'LAB':
        sys.exit(f'refused: role is {opskills.detect_role()}; lab stop/resume need role LAB (QAZBOT_ROLE=LAB or ~/.config/qazbot/role)')
    {'stop': cmd_stop, 'resume': cmd_resume, 'status': cmd_status}[a.c](a)


if __name__ == '__main__':
    main()
