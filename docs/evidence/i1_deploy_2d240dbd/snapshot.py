import json,time,collections,subprocess,gzip,hashlib
from pathlib import Path
p=Path('/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd')
events=[json.loads(x) for x in (p/'passive_observer.jsonl').read_text().splitlines() if x.strip()]
status=[x for x in events if x['kind']=='trust']; sums=[x for x in events if x['kind']=='summary']; seeds=[x for x in events if x['kind']=='initialpose']
out={'snapshot_wall':time.time(),'source_commit':'2d240dbd09ca69de87e9ac4b1acde735293ad6b2','initialposes':len(seeds),'trust_samples':len(status),'trust_last':status[-1] if status else None,'last_summary':sums[-1] if sums else None,'reason_counts':dict(collections.Counter(r for x in status for r in x['value']['reasons'])),'errors':[x for x in events if x['kind']=='error'],'stage':'PASSIVE_RUNNING_AWAITING_OPERATOR_SEED','motion_authorized':False}
(p/'passive_snapshot.json').write_text(json.dumps(out,indent=2))
(p/'process_snapshot.txt').write_text(subprocess.check_output(['ps','-eo','user,pid,ppid,args'],text=True))
for name in ['passive_observer.jsonl','observer.log','localization.log','trust.log','costmap.log']:
 data=(p/name).read_bytes(); archive=gzip.compress(data,mtime=0); (p/(name+'.gz')).write_bytes(archive)
print(json.dumps({'initialposes':len(seeds),'trust_samples':len(status),'snapshot':str(p/'passive_snapshot.json')}))
