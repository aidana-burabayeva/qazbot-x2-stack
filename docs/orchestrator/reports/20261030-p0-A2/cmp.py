import os,hashlib,subprocess,sys,collections
L='/home/agi/control_backend'; G=L+'/qazbot-x2-stack'
OUT='/home/agi/.orchestrator/reports/20261030-p0-A2'
CAP=20*1024*1024
def skip(rel):
    p=rel.split('/')
    return '__pycache__' in p or rel.endswith('.pyc') or '.git' in p or p[0] in('build','install','log') or rel.startswith('ws/build') or rel.startswith('ws/install') or rel.startswith('ws/log')
def h(p):
    m=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda:f.read(1<<20),b''): m.update(c)
    return m.hexdigest()
def walk(root,dirs,files):
    r={}
    for d in dirs:
        for dp,dn,fn in os.walk(root+'/'+d):
            dn[:]=[x for x in dn if x not in('__pycache__','.git')]
            for f in fn:
                a=os.path.join(dp,f); rel=os.path.relpath(a,root)
                if skip(rel): continue
                r[rel]=a
    for f in files:
        if os.path.exists(root+'/'+f): r[f]=root+'/'+f
    return r
dirs=['ws','tools','tests','run','docs','candidates_v13']
files=['passive_probe.py','README.md']
live=walk(L,dirs,files); git=walk(G,dirs+['.claude'],files+['AGENTS.md','CLAUDE.md','.gitignore'])
tracked=subprocess.run(['git','-C',G,'ls-files','-z'],capture_output=True).stdout.decode().split('\0')
tracked=set(t for t in tracked if t)
headh={}
out=subprocess.run(['git','-C',G,'ls-tree','-r','HEAD'],capture_output=True,text=True).stdout
blob={}
for l in out.splitlines():
    meta,path=l.split('\t',1); mode,typ,oid=meta.split()
    blob[path]=(mode,oid)
def info(a):
    if os.path.islink(a): return ('SYMLINK->'+os.readlink(a),os.lstat(a).st_size)
    s=os.path.getsize(a)
    if s>CAP: return ('SKIPPED_LARGE',s)
    return (h(a),s)
def gitblobsha(rel):
    r=subprocess.run(['git','-C',G,'show','HEAD:'+rel],capture_output=True)
    return hashlib.sha256(r.stdout).hexdigest() if r.returncode==0 else ''
rows=[]
allr=sorted(set(live)|set(git))
for rel in allr:
    li=info(live[rel]) if rel in live else None
    gi=info(git[rel]) if rel in git else None
    if li and gi:
        if 'SKIPPED_LARGE' in (li[0],gi[0]): c='SKIPPED_LARGE'
        else: c='IDENTICAL' if li[0]==gi[0] else 'DIFFERENT'
    elif li: c='ONLY_LIVE'
    else: c='ONLY_GIT'
    tr='tracked' if rel in tracked else 'untracked'
    hh=gitblobsha(rel) if rel in blob else ''
    rows.append((rel,c,li[0] if li else '',li[1] if li else '',gi[0] if gi else '',gi[1] if gi else '',tr,hh,
      ('LIVE==HEAD' if li and hh and li[0]==hh else ('LIVE!=HEAD' if li and hh else '')),
      ('WT==HEAD' if gi and hh and gi[0]==hh else ('WT!=HEAD' if gi and hh else ''))))
with open(OUT+'/all_files.tsv','w') as f:
    f.write('path\tclass\tlive_sha\tlive_size\tgit_wt_sha\tgit_size\tgit_tracked\thead_sha\tlive_vs_head\twt_vs_head\n')
    for r in rows: f.write('\t'.join(map(str,r))+'\n')
def top(rel):
    p=rel.split('/')
    if p[0]=='ws': return 'ws/src' 
    if p[0]=='docs' and len(p)>2 and p[1] in('evidence','roadmap_v13'): return 'docs/'+p[1]
    return p[0] if len(p)>1 else '(toplevel)'
cnt=collections.defaultdict(collections.Counter)
for r in rows: cnt[top(r[0])][r[1]]+=1
for d,c in sorted(cnt.items()): print(d,dict(c))
