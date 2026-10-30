"""P1 minimal end-to-end: automatic Claude <-> Codex handoff on simulated agent loss. Fake CLIs on PATH; real nq.py daemon.
Nothing relies on the dying agent's final message; context comes from SQLite + Git."""
import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

NQ = Path(__file__).resolve().parents[2] / 'tools' / 'nightly' / 'nq.py'


def stub(bindir, name, body):
    p = bindir / name
    p.write_text('#!/usr/bin/env bash\n[ "$1" = "--version" ] && { echo "stub-' + name + '"; exit 0; }\n' + body)
    p.chmod(0o755)
    assert p.exists()


@pytest.fixture
def env(tmp_path):
    root = tmp_path / 'root'; root.mkdir()
    subprocess.run(['git', 'init', '-q', str(root)], check=True)
    subprocess.run(['git', '-C', str(root), '-c', 'user.email=a@a', '-c', 'user.name=a', 'commit', '-q', '--allow-empty', '-m', 'i'], check=True)
    b = tmp_path / 'bin'; b.mkdir()
    log = tmp_path / 'calls.log'
    e = dict(os.environ, NIGHTLY_ROOT=str(root), NIGHTLY_RUN=str(root / 'run' / 'nightly'), NIGHTLY_CLAUDE_BIN=str(b / 'claude'), NIGHTLY_CODEX_BIN=str(b / 'codex'), STUBLOG=str(log))
    return e, root, b, log


def nq(e, *a):
    return subprocess.run([sys.executable, str(NQ), *a], capture_output=True, text=True, env=e, timeout=60)


def daemon(e):
    return subprocess.run([sys.executable, str(NQ), 'daemon', '--poll', '0.2'], capture_output=True, text=True, env=e, timeout=90)


def q(root, sql, *a):
    c = sqlite3.connect(root / 'run' / 'nightly' / 'queue.db'); c.row_factory = sqlite3.Row
    return c.execute(sql, a).fetchall()


CODEX_OK = 'echo "codex $*" >> "$STUBLOG"; exit 0\n'
CLAUDE_LIMIT = 'echo "claude $*" >> "$STUBLOG"; echo wip > wip.txt; echo "Claude usage limit reached, try again at 5pm"; exit 1\n'


def test_limit_hits_claude_task_continues_on_codex_with_context_no_final_message(env):
    e, root, b, log = env
    stub(b, 'claude', CLAUDE_LIMIT); stub(b, 'codex', CODEX_OK)
    nq(e, 'root-cause', 'rc1', 'goal tolerances differ', '--status', 'VERIFIED', '--evidence', 'ccw_gate/')
    nq(e, 'enqueue', 'eng', 'investigate --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    assert daemon(e).returncode == 0
    t = q(root, "SELECT * FROM tasks WHERE name='eng'")[0]
    assert t['state'] == 'COMPLETED' and t['owner'] == 'codex'
    calls = log.read_text().splitlines()
    assert calls[0].startswith('claude ') and calls[1].startswith('codex ')
    assert 'goal tolerances differ' in calls[1] and 'do not re-investigate' in calls[1]            # verified cause carried over
    assert 'Last checkpoint (orchestrator-finish' in calls[1] and 'wip.txt' in calls[1]            # recovered from git, not from the agent
    st = json.loads(q(root, "SELECT v FROM kv WHERE k='agents'")[0]['v'])
    assert st['claude']['status'] == 'usage_limited'
    assert any('HANDOFF claude->codex (limit)' in r['detail'] for r in q(root, "SELECT detail FROM events WHERE event='HANDOFF'"))
    cp = q(root, "SELECT * FROM eng_checkpoints WHERE task='eng' ORDER BY id")
    assert cp and all(c['verified'] == 0 for c in cp)                                                 # unfinished work never marked verified


def test_agent_killed_silently_is_lost_and_handed_off(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "claude $*" >> "$STUBLOG"; kill -9 $$\n'); stub(b, 'codex', CODEX_OK)
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e)
    t = q(root, "SELECT * FROM tasks WHERE name='eng'")[0]
    assert t['state'] == 'COMPLETED' and t['owner'] == 'codex'
    assert any('(lost)' in r['detail'] for r in q(root, "SELECT detail FROM events WHERE event='HANDOFF'"))


def test_codex_to_claude_direction(env):
    e, root, b, log = env
    stub(b, 'codex', 'echo "codex $*" >> "$STUBLOG"; echo "rate limit exceeded"; exit 1\n'); stub(b, 'claude', 'echo "claude $*" >> "$STUBLOG"; exit 0\n')
    nq(e, 'enqueue', 'eng', 'x --agents=codex,claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e)
    t = q(root, "SELECT * FROM tasks WHERE name='eng'")[0]
    assert t['state'] == 'COMPLETED' and t['owner'] == 'claude'


def test_genuine_agent_error_is_not_handed_off_blindly(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "claude $*" >> "$STUBLOG"; echo "Traceback: real bug in the task"; exit 2\n'); stub(b, 'codex', CODEX_OK)
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e)
    assert q(root, "SELECT state FROM tasks WHERE name='eng'")[0]['state'] == 'NEEDS_ENGINEERING'
    assert not any(ln.startswith('codex ') for ln in log.read_text().splitlines())


def test_both_agents_lost_ends_in_needs_engineering_not_loop(env):
    e, root, b, log = env
    stub(b, 'claude', CLAUDE_LIMIT); stub(b, 'codex', 'echo "codex $*" >> "$STUBLOG"; kill -9 $$\n')
    nq(e, 'enqueue', 'eng', 'x --agents=claude,codex', '--kind', 'engineering', '--cwd', str(root))
    daemon(e)
    s = q(root, "SELECT state FROM tasks WHERE name='eng'")[0]['state']
    assert s in ('NEEDS_ENGINEERING', 'PAUSED_AGENT') and len(log.read_text().splitlines()) == 2   # each agent tried exactly once


def test_single_writer_per_worktree_engineering_tasks_do_not_overlap(env):
    e, root, b, log = env
    stub(b, 'claude', 'echo "S $(date +%s.%N)" >> "$STUBLOG"; sleep 1.5; echo "E $(date +%s.%N)" >> "$STUBLOG"\n'); stub(b, 'codex', CODEX_OK)
    nq(e, 'enqueue', 'e1', 'a --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    nq(e, 'enqueue', 'e2', 'b --agents=claude', '--kind', 'engineering', '--cwd', str(root))
    daemon(e)
    seq = [ln[0] for ln in log.read_text().splitlines()]
    assert seq == ['S', 'E', 'S', 'E']
    assert {r['resource'] for r in q(root, "SELECT resource FROM tasks")} == {f'wt:{root}'}
