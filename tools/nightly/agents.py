#!/usr/bin/env python3
"""Provider adapters for the QazBot Nightly orchestrator (SIM ONLY, stdlib only).

The orchestrator owns task lifecycle; an adapter only knows how to talk to ONE provider. No adapter may rely on the
agent's final message: state comes from SQLite + Git (engineering checkpoint), never from the agent's last words.

Interface: installed() command() resume_task() request_stop() get_usage() get_status() checkpoint() handoff()
Proven facts (this host): claude 2.1.x supports `claude -p`; codex 0.165 `codex queue --thread` delivers into the existing
session (`exec resume` is refused while a TUI holds the writer lock). Quota/reset APIs are NOT proven -> get_usage()
reports UNKNOWN and nothing is assumed about remaining quota.
"""
import json
import os
import re
import shutil
import subprocess
import time

LIMIT_RE = re.compile(r'usage limit|rate limit|quota|limit reached|try again at|too many requests', re.I)
RULES = ('Rules: simulation only; never contact the physical X2 (no ssh/deploy/ros2 pub); do not weaken safety limits or goal tolerances; '
         'work only in your own git worktree; commit nothing to shared branches; on usage limit stop and exit (state is recovered from SQLite and Git).')


class Adapter:
    name = '?'

    @property
    def bin(self):
        return os.environ.get(f'NIGHTLY_{self.name.upper()}_BIN', self.name)  # tests point this at a stub; a login shell resets PATH

    def installed(self):
        return shutil.which(self.bin) is not None

    def version(self):
        try:
            return subprocess.run([self.bin, '--version'], capture_output=True, text=True, timeout=20).stdout.strip()
        except Exception:
            return ''

    def prompt(self, task, context):
        return f"{task['repro']}\n{context}\n{RULES}"

    def command(self, task, context):
        raise NotImplementedError

    def resume_task(self, task, context):
        """Recovery is stateless on purpose: a fresh invocation with the context rebuilt from SQLite/Git."""
        return self.command(task, context)

    def request_stop(self, message):
        return None  # no cooperative channel proven; orchestrator kills the process group

    def get_usage(self):
        return {'agent': self.name, 'source': None, 'used_pct': None, 'reset_at': None, 'status': 'UNKNOWN'}

    def get_status(self, agents_state):
        s = agents_state.get(self.name, {})
        limited = s.get('status') == 'usage_limited' and s.get('retry_after', 0) > time.time()
        return {'installed': self.installed(), 'limited': limited, 'retry_after': s.get('retry_after')}

    def checkpoint(self, task):
        return None  # written by ordinary code (nq.eng_autocheckpoint), not requested from the agent

    def handoff(self, task, to_agent):
        return {'from': self.name, 'to': to_agent, 'task': task['name'], 'via': 'sqlite+git'}


class ClaudeAdapter(Adapter):
    name = 'claude'

    def command(self, task, context):
        return f"{self.bin} -p {json.dumps(self.prompt(task, context))} --permission-mode acceptEdits --output-format text"


class CodexAdapter(Adapter):
    name = 'codex'

    def command(self, task, context):
        thread = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
        return f"{self.bin} queue --thread {thread} --message {json.dumps(self.prompt(task, context))}"

    def request_stop(self, message):
        thread = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
        return subprocess.run([self.bin, 'queue', '--thread', thread, '--message', message], capture_output=True, text=True, timeout=45)


ADAPTERS = {'claude': ClaudeAdapter(), 'codex': CodexAdapter()}


def classify(rc, verdict, log_tail):
    """Why an agent invocation ended badly: 'limit' (provider limit text), 'lost' (killed/hung/silent), else 'error'."""
    if rc == 0 and verdict == 'ok':
        return 'ok'
    if LIMIT_RE.search(log_tail or ''):
        return 'limit'
    if verdict in ('hang', 'timeout') or (rc is not None and rc < 0) or rc in (137, 143) or not (log_tail or '').strip():
        return 'lost'
    return 'error'
