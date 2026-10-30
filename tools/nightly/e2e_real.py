#!/usr/bin/env python3
"""REAL-agent E2E: Claude -> Codex -> Claude on a throw-away repo. SPENDS PROVIDER QUOTA (small). SIM ONLY, no robot, no production.

  python3 tools/nightly/e2e_real.py --i-understand-this-spends-quota [--out DIR]

Uses the real `claude -p` and real `codex exec` through the normal QazBot Nightly orchestrator (nq.py daemon). Each step runs in its own
attempt worktree cut from the previous VERIFIED commit; the orchestrator commits, hashes and re-runs the declared pytest command itself.
Unit tests never run this (they use stubs); it is only started explicitly.
"""
import argparse
import hashlib
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
NQ = HERE / 'nq.py'


def sh(*a, cwd=None, env=None, timeout=300):
    return subprocess.run(a, cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)


def make_repo(root):
    (root / 'calc').mkdir(parents=True)
    (root / 'tests').mkdir()
    (root / 'calc' / '__init__.py').write_text('')
    (root / 'calc' / 'a.py').write_text('def add(a, b):\n    return a - b\n')
    (root / 'calc' / 'b.py').write_text('def mul(a, b):\n    return a + b\n')
    (root / 'tests' / 'test_a.py').write_text('from calc.a import add\n\n\ndef test_add():\n    assert add(2, 3) == 5\n    assert add(-1, 1) == 0\n')
    (root / 'tests' / 'test_b.py').write_text('from calc.b import mul\n\n\ndef test_mul():\n    assert mul(2, 3) == 6\n    assert mul(0, 5) == 0\n')
    (root / 'CHANGELOG.md').write_text('# calc changelog\n')
    sh('git', 'init', '-q', '-b', 'main', str(root))
    sh('git', '-C', str(root), '-c', 'user.name=e2e', '-c', 'user.email=e2e@localhost', 'add', '-A')
    sh('git', '-C', str(root), '-c', 'user.name=e2e', '-c', 'user.email=e2e@localhost', 'commit', '-q', '-m', 'initial: two bugs')
    (root / '.git' / 'info' / 'exclude').write_text('run/\n')


TASKS = [
    ('s1_fix_add', 'claude', 'claude-haiku-5-5', 'low', None, [],
     'Fix the bug in calc/a.py so that tests/test_a.py passes. Change only calc/a.py. Use tests.cmd "python3 -m pytest -q tests/test_a.py" and files=[{"path":"calc/a.py"}].'),
    ('s2_fix_mul', 'codex', 'gpt-6.1-sol', 'low', 's1_fix_add', ['s1_fix_add'],
     'The previous step (done by another agent, already committed in your worktree) fixed calc/a.py. Now fix the bug in calc/b.py so that tests/test_b.py passes, and keep tests/test_a.py passing. Change only calc/b.py. '
     'Use tests.cmd "python3 -m pytest -q tests/test_a.py tests/test_b.py" and files=[{"path":"calc/b.py"}].'),
    ('s3_integrate', 'claude', 'claude-haiku-5-5', 'low', 's2_fix_mul', ['s2_fix_mul'],
     'Two previous steps by other agents are committed in your worktree (calc/a.py and calc/b.py fixed). Create tests/test_integration.py asserting add(2, 3) == 5 and mul(2, 3) == 6 (import from calc.a and calc.b), '
     'and append the single line "- fixed add and mul" to CHANGELOG.md. Use tests.cmd "python3 -m pytest -q" and files=[{"path":"tests/test_integration.py"},{"path":"CHANGELOG.md"}].'),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--i-understand-this-spends-quota', action='store_true')
    ap.add_argument('--out', default=None)
    ap.add_argument('--max-seconds', type=int, default=1500)
    a = ap.parse_args()
    if not a.i_understand_this_spends_quota:
        sys.exit('refused: this starts REAL claude and codex runs (small quota use). Pass --i-understand-this-spends-quota.')
    base = Path(tempfile.mkdtemp(prefix='qazbot_e2e_'))
    root = base / 'repo'
    make_repo(root)
    env = dict(os.environ, NIGHTLY_ROOT=str(root), NIGHTLY_RUN=str(root / 'run' / 'nightly'), NIGHTLY_CODEX_MODE='exec', NIGHTLY_CLAUDE_MAX_BUDGET_USD='1',
               QAZBOT_ROLE='LAB', NIGHTLY_POLL_S='3')
    for k in ('NIGHTLY_CLAUDE_BIN', 'NIGHTLY_CODEX_BIN', 'NIGHTLY_CODEX_OBSERVE_CMD', 'NIGHTLY_CODEX_USAGE_CMD', 'NIGHTLY_CODEX_SOCK', 'NIGHTLY_CODEX_QUEUE_DB'):
        env.pop(k, None)   # real binaries, real sources
    for name, agent, model, effort, base_from, deps, text in TASKS:
        cmd = ['python3', str(NQ), 'enqueue', name, f'{text} --agents={agent}', '--kind', 'engineering', '--cwd', str(root), '--model', model, '--effort', effort,
               '--deps', json.dumps(deps), '--timeout', '900', '--heartbeat', '600']
        if base_from:
            cmd += ['--base-from', base_from]
        r = sh(*cmd, env=env)
        print(name, r.stdout.strip(), r.stderr.strip()[:200])
    t0 = time.time()
    d = subprocess.run(['python3', str(NQ), 'daemon', '--poll', '2', '--max-seconds', str(a.max_seconds)], env=env, capture_output=True, text=True)
    print('daemon rc', d.returncode, 'after', int(time.time() - t0), 's', d.stderr[-300:])
    db = sqlite3.connect(root / 'run' / 'nightly' / 'queue.db'); db.row_factory = sqlite3.Row
    tasks = [dict(r) for r in db.execute('SELECT name,state,phase,owner,model,effort,attempts,attempt_wt,branch,reason FROM tasks ORDER BY id')]
    results = [dict(r) for r in db.execute('SELECT task,attempt,scenario_id,verdict,metrics,evidence_path,manifest_sha256,status FROM results ORDER BY id')]
    events = [dict(r) for r in db.execute("SELECT datetime(ts,'unixepoch','localtime') ts,task,event,substr(detail,1,200) detail FROM events WHERE task!='*' ORDER BY ts")]
    out = Path(a.out or base / 'evidence'); out.mkdir(parents=True, exist_ok=True)
    # independent check: clean clone of the LAST verified branch, run the full tests myself (not the orchestrator)
    last = tasks[-1]
    indep = {'branch': last.get('branch')}
    if last.get('branch') and last['state'] == 'COMPLETED':
        chk = base / 'indep'
        sh('git', 'clone', '-q', str(root), str(chk))
        sh('git', '-C', str(chk), 'checkout', '-q', last['branch'])
        pr = sh('python3', '-m', 'pytest', '-q', cwd=chk, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
        indep.update(rc=pr.returncode, out=pr.stdout[-300:], tip=sh('git', '-C', str(chk), 'rev-parse', 'HEAD').stdout.strip(),
                     log=sh('git', '-C', str(chk), 'log', '--format=%h %an %s', '-5').stdout.strip().splitlines())
    summary = dict(tasks=tasks, results=results, events=events, independent_check=indep, repo=str(root), seconds=int(time.time() - t0))
    (out / 'e2e_summary.json').write_text(json.dumps(summary, indent=1))
    logs = out / 'agent_logs'; logs.mkdir(exist_ok=True)
    for p in sorted((root / 'run' / 'nightly' / 'logs').glob('*.log')):
        shutil.copy2(p, logs / p.name)
    for p in sorted((root / 'run' / 'nightly' / 'work').glob('*/last_message.txt')):
        shutil.copy2(p, logs / (p.parent.name + '.last_message.txt'))
    man = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'MANIFEST.sha256.json'}
    (out / 'MANIFEST.sha256.json').write_text(json.dumps(man, indent=1))
    print(json.dumps({'tasks': [(t['name'], t['state'], t['owner']) for t in tasks], 'verdicts': [(r['task'], r['verdict']) for r in results], 'independent': indep.get('rc'), 'out': str(out)}, indent=1))
    ok = all(t['state'] == 'COMPLETED' for t in tasks) and indep.get('rc') == 0
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
