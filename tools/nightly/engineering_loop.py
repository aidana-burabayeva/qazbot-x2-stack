#!/usr/bin/env python3
"""Run a list of REAL-agent engineering tasks through the QazBot Nightly orchestrator in an ISOLATED state dir and dump evidence.
SPENDS PROVIDER QUOTA. SIM ONLY. Does not touch the main (stopped) lab queue, systemd units or the robot.

  engineering_loop.py --tasks docs/nightly/loops/X.json --state DIR --out DIR --i-understand-this-spends-quota
tasks json: {"repo": ".", "gate": "<integration gate cmd>", "tasks": [{"name","agents","text","task_type","complexity","risk","deps","base_from","model","effort"}]}
"""
import argparse, hashlib, json, os, sqlite3, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
NQ = HERE / 'nq.py'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tasks', required=True); ap.add_argument('--state', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--max-seconds', type=int, default=1500); ap.add_argument('--i-understand-this-spends-quota', action='store_true')
    a = ap.parse_args()
    if not a.i_understand_this_spends_quota:
        sys.exit('refused: starts real claude/codex runs. Pass --i-understand-this-spends-quota.')
    spec = json.loads(Path(a.tasks).read_text())
    repo = str(Path(spec.get('repo', '.')).resolve())
    state = Path(a.state); state.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, NIGHTLY_ROOT=repo, NIGHTLY_RUN=str(state), NIGHTLY_CODEX_MODE='exec', NIGHTLY_CLAUDE_MAX_BUDGET_USD=str(spec.get('claude_budget_usd', 2)),
               QAZBOT_ROLE='LAB', NIGHTLY_POLL_S='3', NIGHTLY_INTEGRATION_GATE=spec['gate'], NIGHTLY_VERIFY_TIMEOUT_S='900')
    for k in ('NIGHTLY_CLAUDE_BIN', 'NIGHTLY_CODEX_BIN', 'NIGHTLY_CODEX_OBSERVE_CMD', 'NIGHTLY_CODEX_USAGE_CMD', 'NIGHTLY_CODEX_SOCK', 'NIGHTLY_CODEX_QUEUE_DB'):
        env.pop(k, None)
    for t in spec['tasks']:
        cmd = ['python3', str(NQ), 'enqueue', t['name'], f"{t['text']} --agents={t['agents']}", '--kind', 'engineering', '--cwd', repo, '--deps', json.dumps(t.get('deps', [])),
               '--timeout', str(t.get('timeout', 900)), '--heartbeat', '600']
        for opt in ('task_type', 'complexity', 'risk', 'base_from', 'model', 'effort'):
            if t.get(opt):
                cmd += ['--' + opt.replace('_', '-'), t[opt]]
        r = subprocess.run(cmd, env=env, capture_output=True, text=True); print(t['name'], r.stdout.strip(), r.stderr.strip()[:200])
    t0 = time.time()
    d = subprocess.run(['python3', str(NQ), 'daemon', '--poll', '2', '--max-seconds', str(a.max_seconds)], env=env, capture_output=True, text=True)
    secs = int(time.time() - t0)
    db = sqlite3.connect(state / 'queue.db'); db.row_factory = sqlite3.Row
    q = lambda s: [dict(r) for r in db.execute(s)]
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    summary = dict(seconds=secs, daemon_rc=d.returncode, repo=repo,
                   tasks=q('SELECT name,state,phase,owner,model,effort,attempts,branch,reason,task_type FROM tasks ORDER BY id'),
                   results=q('SELECT task,attempt,verdict,metrics,status FROM results ORDER BY id'),
                   router_stats=q('SELECT task,attempt,cls,agent,model,effort,escalation,outcome,round(seconds,1) seconds FROM router_stats ORDER BY id'),
                   merge_queue=q('SELECT task,branch,substr(sha,1,12) sha,state,detail,substr(merge_sha,1,12) merge_sha FROM merge_queue ORDER BY id'),
                   events=q("SELECT datetime(ts,'unixepoch','localtime') ts,task,event,substr(detail,1,180) detail FROM events WHERE task!='*' ORDER BY ts"))
    (out / 'loop_summary.json').write_text(json.dumps(summary, indent=1))
    logs = out / 'agent_logs'; logs.mkdir(exist_ok=True)
    for p in sorted((state / 'logs').glob('*.log')):
        (logs / p.name).write_bytes(p.read_bytes())
    for p in sorted((state / 'work').glob('*/last_message.txt')):
        (logs / (p.parent.name + '.last_message.txt')).write_bytes(p.read_bytes())
    if (state / 'integration_state.json').exists():
        (out / 'integration_state.json').write_bytes((state / 'integration_state.json').read_bytes())
    man = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'MANIFEST.sha256.json'}
    (out / 'MANIFEST.sha256.json').write_text(json.dumps(man, indent=1))
    print(json.dumps({'tasks': [(t['name'], t['state'], t['owner'], t['model'], t['effort']) for t in summary['tasks']], 'merge': [(m['task'], m['state']) for m in summary['merge_queue']],
                      'seconds': secs}, indent=1))
    sys.exit(0 if all(t['state'] == 'COMPLETED' for t in summary['tasks']) else 1)


if __name__ == '__main__':
    main()
