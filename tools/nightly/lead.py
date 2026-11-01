"""Evidence-based execution health and navigation ordering for nq; no scheduling loop."""
import hashlib
import json
import re
import time

INFRA={'PROVIDER_TRANSPORT','AUTHENTICATION','RATE_LIMIT','SANDBOX_NETWORK','MODEL_UNAVAILABLE','SQLITE_CONCURRENCY','ADAPTER_INVOCATION'}


def classify(text,rc=1):
    """Specific causal signals precede secondary absent output; no domain diagnosis from transport."""
    s=text.lower()
    rules=(('SQLITE_CONCURRENCY',('database is locked','database table is locked')),
           ('ADAPTER_INVOCATION',('--agents file not found','unknown option','unrecognized arguments')),
           ('RATE_LIMIT',('usage limit','rate limit','quota exceeded','hit your','too many requests')),
           ('AUTHENTICATION',('unauthorized','authentication failed','invalid api key','not logged in','401 unauthorized')),
           ('MODEL_UNAVAILABLE',('model not found','model is not available','model is not supported','does not exist or you do not have access')),
           ('SANDBOX_NETWORK',('bwrap:','network is unreachable','could not resolve host','temporary failure in name resolution','permission denied while trying to connect','read-only file system','eai_again',"can't reach the api server")),
           ('PROVIDER_TRANSPORT',('workspace routing discovery failed','transport channel closed','connection refused','connection reset','error sending request','failed to connect')))
    for category,signals in rules:
        for signal in signals:
            if signal in s:return category,signal
    return ('AGENT_EXECUTION','nonzero exit') if rc else ('MISSING_RESULT','missing mandatory engineering result')


def init(db):
    db.execute('CREATE TABLE IF NOT EXISTS execution_health(provider TEXT PRIMARY KEY,status TEXT,category TEXT,signature TEXT,evidence TEXT,model TEXT,source_sha TEXT,ts REAL)')
    db.execute('CREATE TABLE IF NOT EXISTS execution_incidents(signature TEXT PRIMARY KEY,provider TEXT,category TEXT,detail TEXT,evidence TEXT,ts REAL)')
    db.execute('CREATE TABLE IF NOT EXISTS execution_affected(signature TEXT,task TEXT,attempt INTEGER,PRIMARY KEY(signature,task,attempt))')
    db.execute('CREATE TABLE IF NOT EXISTS planning_decisions(id INTEGER PRIMARY KEY,ts REAL,task TEXT,facts_sha TEXT,base_priority INTEGER,effective_priority INTEGER,explanation TEXT)')


def health(db,provider):
    init(db)
    row=db.execute('SELECT * FROM execution_health WHERE provider=?',(provider,)).fetchone()
    return dict(row) if row else None


def observe(db,provider,category,detail,evidence,*,task=None,attempt=None,model=None,source_sha=None,success=False):
    init(db)
    signature=hashlib.sha256((provider+'|'+category+'|'+detail).encode()).hexdigest()[:20]
    db.execute('INSERT INTO execution_health VALUES(?,?,?,?,?,?,?,?) ON CONFLICT(provider) DO UPDATE SET status=excluded.status,category=excluded.category,signature=excluded.signature,evidence=excluded.evidence,model=excluded.model,source_sha=excluded.source_sha,ts=excluded.ts',
               (provider,'EXEC_VERIFIED' if success else 'BLOCKED',category,signature,str(evidence),model,source_sha,time.time()))
    if not success:
        db.execute('INSERT OR IGNORE INTO execution_incidents VALUES(?,?,?,?,?,?)',(signature,provider,category,detail,str(evidence),time.time()))
        if task:db.execute('INSERT OR IGNORE INTO execution_affected VALUES(?,?,?)',(signature,task,attempt))
    return signature


def block(db,nq,row,category,signal,evidence):
    """One infrastructure issue, original attempts retained; no AMCL/P23 diagnosis invented."""
    provider=row['owner'] or 'unknown'
    sig=observe(db,provider,category,signal,evidence,task=row['name'],attempt=row['attempts'],model=row['model'])
    name='infra_execution_'+sig
    nq.enqueue(db,name,'true',kind='mechanical',state='NEEDS_ENGINEERING',priority=0,
               execution_tier='ENGINEERING',reason=category+': '+signal,
               repro='Infrastructure diagnosis only; inspect '+str(evidence)+'. No blind retry. After a causal fix or new environment evidence run workflow.py capability-probe for '+provider+'. Not an AMCL/Nav2 defect.')
    nq.ev(db,row['name'],'EXECUTION_BLOCKED',json.dumps(dict(category=category,signature=sig,evidence=str(evidence))))
    return sig


def navigation_band(task):
    t=dict(task);s=(t['name']+' '+str(t.get('task_type',''))).lower()
    if any(k in s for k in ('infra_','execution_probe_','amcl','initialpose','sensor_contract','localization','trust','tf_','odometry','freshness','collision','t1_','t5_')):return 0
    if any(k in s for k in ('p23','controller','costmap','planning','l3','d1_','frame_anchor')):return 1
    if any(k in s for k in ('dynamic','recovery','doorway','dropout')):return 2
    if any(k in s for k in ('gpu','rtf','optimization','bayesian','rl_')):return 3
    return 2


def order(db,rows):
    """Lexicographic rules, never an opaque weighted sum. Cache only fact changes."""
    init(db)
    all_tasks=[dict(r) for r in db.execute('SELECT * FROM tasks').fetchall()]
    health_rows=[dict(r) for r in db.execute('SELECT * FROM execution_health ORDER BY provider').fetchall()]
    causes=[dict(r) for r in db.execute('SELECT task,status,evidence FROM root_causes').fetchall()] if db.execute("SELECT 1 FROM sqlite_master WHERE name='root_causes'").fetchone() else []
    facts=json.dumps(dict(tasks=all_tasks,health=health_rows,causes=causes,candidates=[dict(r) for r in rows]),sort_keys=True,default=str)
    digest=hashlib.sha256(facts.encode()).hexdigest()
    saved=db.execute("SELECT v FROM kv WHERE k='navigation_plan_facts'").fetchone()
    changed=not saved or json.loads(saved[0])!=digest
    dependants={r['name']:sum(r['name'] in json.loads(t['deps']) and t['state']!='COMPLETED' for t in all_tasks) for r in all_tasks}
    bands={r['name']:navigation_band(r) for r in all_tasks}
    # A resource/evidence prerequisite inherits the most critical blocked dependent's band.
    for _ in range(len(all_tasks)):
        altered=False
        for t in all_tasks:
            child=bands.get(t['name'])
            if child is None:continue
            for dep in json.loads(t['deps']):
                if dep in bands and bands[dep]>child:bands[dep]=child;altered=True
        if not altered:break
    def key(r):
        # After hard safety/capability/dependency eligibility, localization gets first diagnostic turn.
        amcl=0 if 'amcl_initialpose' in r['name'] else 1
        confirmed=any(c['task']==r['name'] and c['status']=='VERIFIED' and c['evidence'] for c in causes)
        return (bands[r['name']],amcl,-dependants[r['name']],0 if confirmed else 1,r['priority'],r['id'])
    ordered=sorted(rows,key=key)
    if changed:
        for r in all_tasks:
            why=f"Navigation P{bands[r['name']]}; localization-first; blocks {dependants.get(r['name'],0)} task(s); base priority {r['priority']}; hard dependency/tier/provider gates remain mandatory"
            db.execute('INSERT INTO planning_decisions(ts,task,facts_sha,base_priority,effective_priority,explanation) VALUES(?,?,?,?,?,?)',
                       (time.time(),r['name'],digest,r['priority'],bands[r['name']],why))
        db.execute("INSERT INTO kv VALUES('navigation_plan_facts',?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",(json.dumps(digest),))
    return ordered


def claude_final_text(text):
    """Final assistant text from `claude -p --output-format stream-json`; None unless the last result event succeeded."""
    final=None
    for line in text.splitlines():
        try:event=json.loads(line)
        except ValueError:continue
        if isinstance(event,dict) and event.get('type')=='result':final=event
    if not final or final.get('is_error') or final.get('subtype')!='success':return None
    return str(final.get('result','')).strip()


def probe(nq,provider,model=None):
    """Explicit, finite capability qualification. Never inferred from quota/app-server/version."""
    import os
    from pathlib import Path
    import subprocess
    import agents,attempts,execution,shlex
    db=nq.connect()
    fp=fingerprint(provider,model)
    ok,why=execution.allowed(db,nq.ROOT,execution.ENGINEERING)
    if not ok:raise SystemExit('capability probe refused: '+why)
    aid=agents.new_attempt_id();name='capability_'+provider+'_'+aid[:12]
    out=nq.RUN/'capabilities'/name;out.mkdir(parents=True)
    sha=nq.mergeq.integration_head(nq.ROOT)
    wt,branch=attempts.prepare(nq.ROOT,nq.RUN,name,1,aid,sha)
    marker='QAZBOT_EXEC_OK_'+aid
    task=dict(repro='Reply with exactly '+marker+'. Do not run tools, modify files or contact the physical robot.',execution_tier='ENGINEERING')
    command=agents.ADAPTERS[provider].command(task,'',cwd=wt,model=model,result_dir=out)
    argv=execution.engineering_sandbox(['bash','-lc',command],wt,out)
    env=dict(os.environ,NIGHTLY_CODEX_MODE='exec',ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='87',DOCKER_HOST='',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    log=out/'task.log'
    with log.open('wb') as f:
        process=subprocess.Popen(argv,cwd=wt,env=env,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT)
        (out/'process.json').write_text(json.dumps({'pid':process.pid,'state':'RUNNING','started':time.time()}))
        rc=process.wait()  # same queue wall/heartbeat limits when dispatched by nq; no new retry/watchdog
    text=log.read_text(errors='replace')
    # A prompt echo is insufficient: Codex must publish a completed assistant message/turn.
    if provider=='codex':
        events=[]
        for line in text.splitlines():
            try:events.append(json.loads(line))
            except ValueError:pass
        passed=rc==0 and any(e.get('type')=='turn.completed' for e in events) and any(e.get('type')=='item.completed' and e.get('item',{}).get('type')=='agent_message' and e['item'].get('text','').strip()==marker for e in events)
    else:passed=rc==0 and claude_final_text(text)==marker
    category,detail=('EXECUTION_OK','verified assistant output') if passed else classify(text,rc)
    observe(db,provider,category,detail,log,model=model,source_sha=sha,success=passed)
    nq.kv_set(db,'execution_capability:'+provider+':'+str(model),dict(fingerprint=fingerprint(provider,model),status='EXEC_VERIFIED' if passed else 'BLOCKED',ts=time.time(),evidence=str(log)))
    report=dict(provider=provider,model=model,source_sha=sha,worktree=wt,branch=branch,rc=rc,passed=passed,category=category,evidence=str(log),sandbox='execution.engineering_sandbox',command=command.replace(marker,'<challenge>'))
    (out/'receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    if passed:
        for r in db.execute("SELECT * FROM tasks WHERE state IN ('BLOCKED_EXECUTION','PAUSED_AGENT')").fetchall():
            if provider in nq.db_get_agents(r):
                nq.kv_set(db,'tried:'+r['name'],[])
                db.execute("UPDATE tasks SET state='PENDING',reason=? WHERE id=?",('execution capability restored: '+str(out/'receipt.json'),r['id']))
                nq.ev(db,r['name'],'CAPABILITY_RESTORED',str(out/'receipt.json'))
    return report


def fingerprint(provider,model):
    import os,shutil
    from pathlib import Path
    import agents
    paths=[Path(__file__).with_name(n) for n in ('agents.py','execution.py')]
    binary=shutil.which(agents.ADAPTERS[provider].bin)
    if binary:paths.append(Path(binary).resolve())
    # Claude: settings.json, not ~/.claude.json (CLI/session state rewritten on every run)
    paths.extend([Path.home()/'.codex/config.toml',Path.home()/'.codex/auth.json'] if provider=='codex' else [Path.home()/'.claude/settings.json'])
    metadata=[]
    for path in paths:
        try:st=path.stat();metadata.append((str(path),st.st_size,st.st_mtime_ns))
        except OSError:metadata.append((str(path),'missing'))
    # not PATH: the resolved binary path is already in `metadata`, and an interactive shell's PATH differs from the service's
    env={k:os.environ.get(k) for k in ('HOME','HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','NO_PROXY')}
    return hashlib.sha256(json.dumps((provider,model,metadata,env),sort_keys=True).encode()).hexdigest()[:20]


def ensure_probe(db,nq,provider,model):
    """One diagnostic per changed execution facts, not a periodic retry/watchdog."""
    import shlex
    fp=fingerprint(provider,model)
    base='execution_probe_'+provider+'_'+fp
    name=base
    # enqueue() is idempotent by name: if the probe for exactly these facts already finished but the stored capability still does not
    # match (e.g. it was overwritten by an out-of-band probe), requalify again - at most twice, never a loop.
    cap=nq.kv_get(db,'execution_capability:'+provider+':'+str(model),{})
    row=db.execute('SELECT state FROM tasks WHERE name=?',(base,)).fetchone()
    if row and row[0] in ('COMPLETED','FAILED','NEEDS_ENGINEERING') and cap.get('fingerprint')!=fp:
        used=db.execute("SELECT COUNT(*) FROM tasks WHERE name LIKE ? ESCAPE '\\'",(base.replace('_','\\_')+'\\_r%',)).fetchone()[0]
        if used<2:name=f'{base}_r{used+1}'
    cmd='python3 tools/nightly/workflow.py capability-probe --provider '+provider
    if model:cmd+=' --model '+shlex.quote(model)
    nq.enqueue(db,name,cmd,kind='mechanical',execution_tier='ENGINEERING',priority=0,resource='engineering',max_attempts=1,
               repro='P0 execution qualification for changed adapter/configuration facts '+fp)
    return fp


def trusted_probe(command):
    import shlex
    try:argv=shlex.split(command)
    except ValueError:return False
    return _probe_argv(argv) and command==' '.join(argv) and (len(argv)==5 or re.fullmatch(r'[A-Za-z0-9_.:/-]+',argv[6]) is not None)


def _probe_argv(argv):
    return (len(argv) in (5,7) and argv[:4]==['python3','tools/nightly/workflow.py','capability-probe','--provider'] and
            argv[4] in ('claude','codex') and (len(argv)==5 or argv[5]=='--model'))
