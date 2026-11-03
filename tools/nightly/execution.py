"""Capabilities for the existing queue; this is an authorization policy, not a scheduler."""
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import time

ENGINEERING = 'ENGINEERING'
L3 = 'L3_QUALIFICATION'
FULL = 'FULL_NIGHTLY'
MODES = (ENGINEERING, L3, FULL)
STATES = {ENGINEERING:'RUNNING_ENGINEERING', L3:'QUALIFYING_L3', FULL:'RUNNING_FULL'}


def managed(root):
    # Throw-away generic queue fixtures/repos do not carry the LAB deployment configuration.
    return (Path(root)/'tools/nightly/labctl.sh').is_file()


def get(db, key, default=None):
    row=db.execute('SELECT v FROM kv WHERE k=?',(key,)).fetchone()
    return json.loads(row[0]) if row else default


def put(db,key,value):
    db.execute('INSERT INTO kv VALUES(?,?) ON CONFLICT(k) DO UPDATE SET v=excluded.v',(key,json.dumps(value)))


def authorize(db, mode, report, root, integration):
    if mode not in MODES or report.get('verdict')!='PASS' or report.get('mode')!=mode:
        raise ValueError('positive matching preflight required')
    auth=dict(mode=mode,root=str(Path(root).resolve()),integration_sha=integration,
              issued=time.time(),expires=time.time()+24*3600,l3_remaining=1 if mode==L3 else None,
              preflight_sha256=hashlib.sha256(json.dumps(report,sort_keys=True).encode()).hexdigest(),
              preflight_stamp=report['stamp'],images=report.get('images',{}),vendor_identity=report.get('vendor_identity',{}))
    put(db,'execution_authorization',auth)
    put(db,'execution_mode',mode)
    return auth


def task_tier(task):
    return dict(task).get('execution_tier') or FULL


def allowed(db, root, tier, integration=None):
    if tier not in MODES:
        return False,'unknown execution tier'
    auth=get(db,'execution_authorization',{})
    if not auth or auth.get('root')!=str(Path(root).resolve()) or auth.get('expires',0)<=time.time():
        return False,'no current positive mode preflight authorization'
    mode=auth.get('mode')
    if tier==ENGINEERING:
        return mode in MODES,'offline engineering capability' if mode in MODES else 'invalid mode'
    if integration is None or auth.get('integration_sha')!=integration:
        return False,'simulation authorization is for another integration SHA; engineering may continue'
    if tier==FULL:
        return mode==FULL,'FULL_NIGHTLY requires E2E qualification' if mode!=FULL else 'full capability'
    if mode==FULL:
        return True,'qualified simulation capability'
    if mode!=L3 or auth.get('l3_remaining',0)<=0:
        return False,'one-run L3 qualification capability missing/exhausted'
    return True,'one bounded official MC qualification run'


def consume_l3(db, task=None, attempt=None):
    """Called under the sole orchestrator lease, immediately before dispatch. Never resets on retry."""
    auth=get(db,'execution_authorization',{})
    if auth.get('mode')==L3:
        if auth.get('l3_remaining',0)!=1:
            raise ValueError('L3 budget exhausted')
        auth['l3_remaining']=0
        auth['l3_claim']={'task':task,'attempt':attempt}
        put(db,'execution_authorization',auth)


def require(tier, *, root=None, run=None):
    """Guard used by every LAB simulation entrypoint; no environment mode can grant authority."""
    root=Path(root or os.environ.get('NIGHTLY_ROOT',Path(__file__).resolve().parents[2]))
    run=Path(run or os.environ.get('NIGHTLY_RUN',root/'run/nightly'))
    try:
        db=sqlite3.connect((run/'queue.db').resolve().as_uri()+'?mode=ro',uri=True)
        db.row_factory=sqlite3.Row
        # Dispatch consumes the single-use budget. Only its RUNNING task can use it afterwards.
        name=os.environ.get('NIGHTLY_TASK')
        row=db.execute('SELECT * FROM tasks WHERE name=? AND state=\'RUNNING\'',(name,)).fetchone() if name else None
        if row is None or (task_tier(row)!=tier and not (tier==L3 and task_tier(row)==FULL)) or (tier!=ENGINEERING and row['resource']!='sim'):
            raise ValueError('simulation must be a tier-labelled RUNNING task in the existing queue')
        if str(row['attempts'])!=os.environ.get('NIGHTLY_ATTEMPT'):
            raise ValueError('stale worker attempt')
        token=os.environ.get('NIGHTLY_DISPATCH_TOKEN')
        if not token or token!=get(db,f'dispatch_token:{row["name"]}:{row["attempts"]}'):
            raise ValueError('only an orchestrator dispatch may start a simulation')
        auth=get(db,'execution_authorization',{})
        if auth.get('mode')==L3 and (auth.get('l3_remaining')!=0 or auth.get('l3_claim')!={'task':row['name'],'attempt':row['attempts']}):
            raise ValueError('L3 run was not claimed by this task/attempt')
        if (not auth or auth.get('mode') not in (L3,FULL) or auth.get('expires',0)<=time.time() or
                auth.get('root')!=str(root.resolve())):
            raise ValueError('no simulation capability')
        import subprocess
        integration=subprocess.check_output(['git','-C',str(root),'rev-parse','lab/stable'],text=True).strip()
        if integration!=auth.get('integration_sha'):
            raise ValueError('stale simulation source authorization')
        if tier==FULL and auth['mode']!=FULL:
            raise ValueError('FULL_NIGHTLY E2E authorization missing')
        if os.environ.get('QAZBOT_ROLE')!='LAB':
            raise ValueError('LAB role required')
        return dict(row)
    except (sqlite3.Error, OSError, ValueError) as exc:
        raise SystemExit('execution refused: '+str(exc))


def engineering_sandbox(argv, worktree, result_dir, writable_run=None):
    """Read-only host, isolated PID/IPC namespace, no Docker/systemd sockets, writable attempt only.

    Provider state directories remain writable for the installed noninteractive CLIs. HTTP
    access for provider APIs/docs is retained; physical robot access remains forbidden by policy.
    """
    import shutil
    binary=shutil.which('bwrap')
    if not binary:
        raise RuntimeError('Engineering requires tested bubblewrap isolation')
    cmd=[binary,'--die-with-parent','--new-session','--unshare-pid','--unshare-ipc','--cap-drop','ALL',
         '--ro-bind','/','/','--proc','/proc','--dev','/dev','--tmpfs','/run','--tmpfs','/tmp',
         # /etc/resolv.conf -> /run/systemd/resolve/stub-resolv.conf: without this mount the tmpfs
         # dangles the symlink and every provider CLI fails name resolution (verified: getent rc=2).
         # Only the resolver state is exposed; docker.sock and /run/user stay hidden.
         '--ro-bind-try','/run/systemd/resolve','/run/systemd/resolve',
         '--bind',str(result_dir),str(result_dir)]
    if writable_run is None:
        cmd+=['--bind',str(worktree),str(worktree)]
    else:
        cmd+=['--bind',str(writable_run),str(writable_run)]
    for name in ('.codex','.claude'):
        path=Path.home()/name
        if path.is_dir() and writable_run is None:
            cmd+=['--bind',str(path),str(path)]
    return cmd+['--chdir',str(worktree),'--',*argv]

# Campaign scope is independent of execution capability. ENGINEERING preflight remains mandatory.
X2_CAMPAIGN = 'x2-blockers-20261103'
X2_TASKS = ('x2_p0_preservation_restore', 'x2_p0_d05_collision_monitor', 'x2_p1_d04_cpp_d07')
ACTIVE_TASK_STATES = ('RUNNING', 'DELEGATED', 'VERIFYING')


def restricted(db):
    return get(db, 'campaign_scope') is not None


def scope_allows(db, name):
    scope = get(db, 'campaign_scope')
    return scope is None or (scope == X2_CAMPAIGN and name in X2_TASKS)


def active_leases(db):
    return [dict(r) for r in db.execute("SELECT name,state FROM tasks WHERE state IN ('RUNNING','DELEGATED','VERIFYING')").fetchall()]


def scope_idle(db):
    return not get(db, 'delegated_open', []) and not active_leases(db) and not db.execute("SELECT 1 FROM merge_queue WHERE state IN ('QUEUED','GATING') LIMIT 1").fetchone()


def next_target(db):
    if get(db, 'campaign_scope') != X2_CAMPAIGN:
        return None
    terminal = {'COMPLETED', 'FAILED', 'NEEDS_ENGINEERING', 'NEEDS_HARDWARE', 'NEEDS_APPROVAL', 'BLOCKED_EXECUTION'}
    for name in X2_TASKS:
        row = db.execute('SELECT state FROM tasks WHERE name=?', (name,)).fetchone()
        if row is None:
            return None  # incomplete allowlist preparation fails closed
        if row['state'] not in terminal:
            return name
    return None


def scope_dispatch_allowed(db, task):
    if not scope_allows(db, task['name']):
        return False
    if not restricted(db):
        return True
    if (task['name'] != next_target(db) or task['kind'] != 'engineering'
            or task_tier(task) != ENGINEERING or task['resource'] != 'engineering'):
        return False
    # Global serial admission, including old/foreign leases; never release them here.
    return scope_idle(db)


def require_scope_dispatch(db, task):
    if not scope_dispatch_allowed(db, task):
        raise RuntimeError('campaign scope/serial lease refuses dispatch: ' + task['name'])


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('tier',choices=MODES);a=p.parse_args()
    require(a.tier)
