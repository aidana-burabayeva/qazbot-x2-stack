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
USAGE_ROBOT_PATHS = ('docs/hardware/ROBOT_FINDINGS.md', 'docs/evidence/robot_', 'docs/HANDOFF.md', 'docs/TEST_STATUS.md')


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
    res = {}
    if args.no_systemd:
        return {'skipped': 'no-systemd'}
    for u in UNITS:
        sh(f'systemctl --user disable --now {u}')
        link = UNIT_DIR / u
        if link.is_symlink():
            link.unlink()
        res[u] = unit_state(u)
    sh('systemctl --user daemon-reload')
    return res


def park_tasks(db):
    """Running tasks: kill their process groups, requeue-on-resume. Pending/paused: parked as-is. Previous state kept for resume."""
    parked = nq.kv_get(db, 'parked', {})
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
    rc, o = sh(['codex', 'queue', '--thread', CODEX_THREAD, '--message', msg], timeout=45)
    nq.kv_set(db, 'codex_stop', 'STOP_UNVERIFIED' if rc == 0 else 'STOP_NOT_DELIVERED')
    return f'rc={rc} {o[:200]}'


def codex_ack_status(db):
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
    L = [f'# Lab → Robot handoff', '',
         f'Сгенерировано `tools/nightly/lab.py stop` {now_iso()}. Ручные данные: `docs/hardware/lab_robot_state.json`. Сначала verify SHA ниже через `git ls-remote`.', '',
         '## Что доступно через Git', '',
         f'- Ветка: `{br}`, SHA на момент остановки: `{head}`', f'- Интеграция с production snapshot (`origin/main`): `{gout("rev-parse", "origin/main")}`',
         '- Тег остановки ставится после push: `lab-stop/<UTC>` (точный SHA в `docs/hardware/LAB_STATE.json` нельзя записать в тот же коммит; он в теге).', '',
         '## Тесты на момент остановки', '', f"- Regression: **{tests['verdict']}** — {tests['summary']}"]
    for t in d.get('tests', []):
        L.append(f"- {t['status']}: {t['name']} — {t['note']}")
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
    log.append(f'STOP sim {stop_sim(args)}')
    g = git_view()
    log.append(f'STOP codex {notify_codex(db, args, g["head"])}')
    nq.checkpoint(db, 'lab stop'); log.append('CHECKPOINT ok')
    tests = run_regression(args); log.append(f"VALIDATE {tests['verdict']} {tests['summary']}")
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
    # robot-integrated pointer: origin/main is the production snapshot
    om = gout('rev-parse', 'origin/main')
    if om and git('merge-base', '--is-ancestor', om, 'HEAD')[0] == 0:
        nq.kv_set(db, 'last_robot_integrated_sha', om)
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
    {'stop': cmd_stop, 'resume': cmd_resume, 'status': cmd_status}[a.c](a)


if __name__ == '__main__':
    main()
