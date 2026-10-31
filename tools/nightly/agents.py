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
import shlex
import shutil
import subprocess
import time
from pathlib import Path

LIMIT_RE = re.compile(r"usage limit|rate limit|quota|limit reached|try again at|too many requests|hit your[\w\s-]{0,24}limit|limit\s*[·.\-:]*\s*resets", re.I)
# observed verbatim from the real claude CLI: "You've hit your 5-hour limit · resets 7:10pm (Asia/Almaty)"
RESET_RE = re.compile(r"resets\s+(\d{1,2})(?::(\d{2}))?\s*([ap]m)\s*\(([A-Za-z_]+/[A-Za-z_]+)\)", re.I)
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
        scope = ('ENGINEERING ONLY: offline/source/replay/tests/review. No Docker/MuJoCo/MC or live ROS graph. '
                 'Never label offline tests SIM_VERIFIED. Request L3_QUALIFICATION for runtime evidence; '
                 'missing hardware facts -> NEEDS_HARDWARE; preserve evidence, then work on an independent issue. '
                 if dict(task).get('execution_tier') == 'ENGINEERING' else '')
        return f"{task['repro']}\n{context}\n{scope}{RULES}"

    def command(self, task, context, cwd=None, model=None, effort=None, result_dir=None):
        raise NotImplementedError

    mode = 'process'

    def resume_task(self, task, context, **kw):
        """Recovery is stateless on purpose: a fresh invocation with the context rebuilt from SQLite/Git."""
        return self.command(task, context, **kw)

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

    def command(self, task, context, cwd=None, model=None, effort=None, result_dir=None):
        opts = ' --permission-mode acceptEdits --output-format text --no-session-persistence'
        if model:
            opts += f' --model {shlex.quote(model)}'
        if effort:
            opts += f' --effort {shlex.quote(effort)}'
        budget = os.environ.get('NIGHTLY_CLAUDE_MAX_BUDGET_USD')
        if budget:
            opts += f' --max-budget-usd {shlex.quote(budget)}'
        return f"{self.bin} -p {shlex.quote(self.prompt(task, context))}{opts}"


class CodexAdapter(Adapter):
    """Default `exec` mode: official non-interactive `codex exec` in its own session and process group -> no open TUI needed,
    explicit process lifecycle, exit code + last-message file + JSON events. `queue` mode (legacy) delivers into an existing TUI session."""
    name = 'codex'

    @property
    def mode(self):
        return 'queue' if os.environ.get('NIGHTLY_CODEX_MODE', 'exec') == 'queue' else 'process'  # 'exec' = process mode

    def command(self, task, context, cwd=None, model=None, effort=None, result_dir=None):
        if self.mode == 'queue':
            thread = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
            return f"{self.bin} queue --thread {thread} --message {shlex.quote(self.prompt(task, context))}"
        parts = [self.bin, 'exec', '--json', '--skip-git-repo-check', '-s', 'workspace-write']
        if cwd:
            parts += ['-C', cwd]
        if model:
            parts += ['-m', model]
        if effort:
            parts += ['-c', f'model_reasoning_effort="{effort}"']
        if result_dir:
            parts += ['-o', str(Path(result_dir) / 'last_message.txt')]
        parts += ['--', self.prompt(task, context)]
        return ' '.join(shlex.quote(x) for x in parts)

    def request_stop(self, message):
        thread = os.environ.get('NIGHTLY_CODEX_SESSION', '01a18872-7bd1-7701-b367-b63e344d449d')
        return subprocess.run([self.bin, 'queue', '--thread', thread, '--message', message], capture_output=True, text=True, timeout=45)


ADAPTERS = {'claude': ClaudeAdapter(), 'codex': CodexAdapter()}


def parse_reset(text, now=None):
    """Official reset time stated BY THE PROVIDER in its limit message, as epoch seconds; None if the text has no explicit time+zone."""
    from datetime import datetime, timedelta
    from zoneinfo import ZoneInfo
    m = RESET_RE.search(text or '')
    if not m:
        return None
    try:
        tz = ZoneInfo(m.group(4))
    except Exception:
        return None
    h, mi, ap = int(m.group(1)) % 12, int(m.group(2) or 0), m.group(3).lower()
    h += 12 if ap == 'pm' else 0
    now_dt = datetime.fromtimestamp(now if now is not None else time.time(), tz)
    t = now_dt.replace(hour=h, minute=mi, second=0, microsecond=0)
    if t <= now_dt:
        t += timedelta(days=1)
    return t.timestamp()


def classify(rc, verdict, log_tail):
    """Why an agent invocation ended badly: 'limit' (provider limit text), 'lost' (killed/hung/silent), else 'error'."""
    if rc == 0 and verdict == 'ok':
        return 'ok'
    if LIMIT_RE.search(log_tail or ''):
        return 'limit'
    if verdict in ('hang', 'timeout') or (rc is not None and rc < 0) or rc in (137, 143) or not (log_tail or '').strip():
        return 'lost'
    return 'error'


# ----------------------------------------------------------------------------------------------------------------------
# Facts about the INSTALLED Codex (0.165 CLI / 0.166 app-server daemon), established by inspection, not assumed:
#  * `codex queue` only appends a row to ~/.codex/queue_1.sqlite (queued_items) and returns "Queued message <id>". It confirms
#    DELIVERY, never execution. A row disappears only when the session consumes it; an idle/interrupted session leaves it queued.
#  * Execution is observable through the app-server (unix socket + websocket, JSON-RPC): thread/turns/list gives turns with
#    status inProgress|completed|failed|interrupted and the user message text.
#  * Quota/reset is observable through account/rateLimits/read (primary 5h and secondary weekly window: usedPercent, resetsAt).
#  * account/rateLimitResetCredit/consume spends credits: this module must never call it.
# Claude: no documented usage/reset interface in the installed CLI (2.1.x) -> UNKNOWN, never guessed.
# ----------------------------------------------------------------------------------------------------------------------
import base64
import socket
import sqlite3
import struct
import uuid

ALLOWED_METHODS = {'initialize', 'account/rateLimits/read', 'thread/turns/list'}  # explicit allowlist: everything else (credit consume, delete, logout...) is impossible
APP_SOCK = os.path.expanduser(os.environ.get('NIGHTLY_CODEX_SOCK', '~/.codex/app-server-control/app-server-control.sock'))
QUEUE_DB = os.path.expanduser(os.environ.get('NIGHTLY_CODEX_QUEUE_DB', '~/.codex/queue_1.sqlite'))
QUEUED_RE = re.compile(r'Queued message (\S+) for thread')


class AppServer:
    """Minimal read-only JSON-RPC-over-websocket client for the local Codex app-server (stdlib only)."""

    def __init__(self, timeout=8):
        self.t = timeout
        self.s = None
        self.rest = b''
        self.n = 0

    def __enter__(self):
        s = socket.socket(socket.AF_UNIX)
        s.settimeout(self.t)
        s.connect(APP_SOCK)
        key = base64.b64encode(os.urandom(16)).decode()
        s.sendall(f'GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n'.encode())
        buf = b''
        while b'\r\n\r\n' not in buf:
            chunk = s.recv(4096)
            if not chunk:
                raise ConnectionError('closed during handshake')
            buf += chunk
        head, _, self.rest = buf.partition(b'\r\n\r\n')
        if b' 101 ' not in head.split(b'\r\n')[0]:
            raise ConnectionError(head.split(b'\r\n')[0])
        self.s = s
        self.call('initialize', {'clientInfo': {'name': 'qazbot-nightly', 'title': None, 'version': '1'}, 'capabilities': None})
        self._send({'jsonrpc': '2.0', 'method': 'initialized'})
        return self

    def __exit__(self, *a):
        try:
            self.s.close()
        except Exception:
            pass

    def _send(self, o):
        d = json.dumps(o).encode()
        m = os.urandom(4)
        n = len(d)
        hdr = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('>H', n) if n < 65536 else bytes([0x80 | 127]) + struct.pack('>Q', n))
        self.s.sendall(hdr + m + bytes(b ^ m[i % 4] for i, b in enumerate(d)))

    def _need(self, k):
        while len(self.rest) < k:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError('closed')
            self.rest += chunk
        out, self.rest = self.rest[:k], self.rest[k:]
        return out

    def _recv(self):
        h = self._need(2)
        n = h[1] & 0x7f
        if n == 126:
            n = struct.unpack('>H', self._need(2))[0]
        elif n == 127:
            n = struct.unpack('>Q', self._need(8))[0]
        return self._need(n).decode()

    def call(self, method, params):
        if method not in ALLOWED_METHODS:
            raise PermissionError(f'app-server method not allow-listed: {method}')
        self.n += 1
        i = self.n
        self._send({'jsonrpc': '2.0', 'id': i, 'method': method, 'params': params})
        deadline = time.time() + self.t
        while time.time() < deadline:
            m = json.loads(self._recv())
            if m.get('id') == i:
                if 'error' in m:
                    raise RuntimeError(m['error'])
                return m['result']
        raise TimeoutError(method)


def _external(env, *args):
    """Test seam: NIGHTLY_CODEX_<env> points at an executable that prints JSON; production uses the real app-server."""
    cmd = os.environ.get('NIGHTLY_CODEX_' + env)
    if not cmd:
        return None
    r = subprocess.run([cmd, *args], capture_output=True, text=True, timeout=30)
    return json.loads(r.stdout)


def codex_usage():
    """AVAILABLE | LIMITED | UNKNOWN, with the provider's own reset time. Never an estimate."""
    try:
        res = _external('USAGE_CMD')
        if res is None:
            with AppServer() as a:
                res = a.call('account/rateLimits/read', None)
        rl = res['rateLimits']
        wins = {k: rl.get(k) for k in ('primary', 'secondary') if rl.get(k)}
        full = [w for w in wins.values() if w['usedPercent'] >= 100]
        limited = bool(rl.get('rateLimitReachedType')) or bool(full) or res.get('ordinaryUsageAllowed') is False
        resets = [w['resetsAt'] for w in (full or wins.values()) if w.get('resetsAt')]
        return {'agent': 'codex', 'source': 'app-server account/rateLimits/read', 'status': 'LIMITED' if limited else 'AVAILABLE',
                'used_pct': {k: w['usedPercent'] for k, w in wins.items()}, 'reset_at': (max(resets) if limited and resets else None),
                'next_reset_at': {k: w.get('resetsAt') for k, w in wins.items()}, 'measured_at': time.time()}
    except Exception as e:  # unreachable source is UNKNOWN, never AVAILABLE
        return {'agent': 'codex', 'source': 'app-server', 'status': 'UNKNOWN', 'used_pct': None, 'reset_at': None, 'error': str(e)[:200], 'measured_at': time.time()}


def claude_usage():
    return {'agent': 'claude', 'source': None, 'status': 'UNKNOWN', 'used_pct': None, 'reset_at': None,
            'note': 'no documented usage/reset interface in the installed CLI; only a provider limit message in a task log can confirm LIMITED'}


def queue_present(item_id, token=None):
    """True: still queued (NOT consumed). False: row gone. None: cannot tell. Without an id the token is searched in the payloads."""
    if not item_id and not token:
        return None
    try:
        db = sqlite3.connect(f'file:{QUEUE_DB}?mode=ro', uri=True, timeout=5)
        try:
            if item_id:
                return db.execute('SELECT 1 FROM queued_items WHERE id=?', (item_id,)).fetchone() is not None
            return True if db.execute('SELECT 1 FROM queued_items WHERE payload_json LIKE ?', (f'%{token}%',)).fetchone() else None
        finally:
            db.close()
    except Exception:
        return None


def codex_observe(thread, item_id, token):
    """What can be PROVEN about a dispatched message. turn_status: None (no turn carries the token) | inProgress | completed | failed | interrupted."""
    out = {'queued': queue_present(item_id, token), 'turn_status': None, 'turn_id': None, 'source_ok': True}
    try:
        ext = _external('OBSERVE_CMD', thread, item_id or '', token)
        if ext is not None:
            out.update(ext)
            return out
        with AppServer() as a:
            res = a.call('thread/turns/list', {'threadId': thread, 'limit': 40, 'itemsView': 'summary', 'sortDirection': 'desc', 'cursor': None})
        t = match_turn(res.get('data', []), token)
        if t is not None:
            out.update(turn_status=t.get('status'), turn_id=t.get('id'), completed_at=t.get('completedAt'))
    except Exception as e:
        out.update(source_ok=False, error=str(e)[:200])
    return out


def match_turn(turns, token):
    """The turn the dispatched message STARTED: token must be in that turn's first user message. A later turn that merely quotes it never matches."""
    for t in turns:
        first = next((i for i in t.get('items', []) if isinstance(i, dict) and i.get('type') == 'userMessage'), None)
        if first is not None and token in json.dumps(first.get('content', '')):
            return t
    return None


def parse_queued_id(stdout):
    m = QUEUED_RE.search(stdout or '')
    return m.group(1) if m else None


def foreign_writers(worktree, own_sessions=()):
    """Agent processes (claude/codex) whose cwd is inside `worktree` and that the orchestrator did not start (e.g. an operator's TUI)."""
    wt = os.path.realpath(worktree)
    found = []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) in own_sessions or int(d) == os.getpid():
            continue
        try:
            cwd = os.path.realpath(os.readlink(f'/proc/{d}/cwd'))
            argv = open(f'/proc/{d}/cmdline', 'rb').read().split(b'\0')
            sid = os.getsid(int(d))
        except (OSError, ProcessLookupError):
            continue
        if sid in own_sessions:
            continue
        names = {os.path.basename(a.decode(errors='ignore')) for a in argv[:2] if a}
        if (cwd == wt or cwd.startswith(wt + os.sep)) and names & {'claude', 'codex'}:
            found.append((int(d), ' '.join(a.decode(errors='ignore') for a in argv[:3])))
    return found


def new_attempt_id():
    return uuid.uuid4().hex[:16]
