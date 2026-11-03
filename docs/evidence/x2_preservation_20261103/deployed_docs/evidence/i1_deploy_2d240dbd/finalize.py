import json,gzip,hashlib,subprocess,time,collections
from pathlib import Path
p=Path('/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd')
d=json.loads((p/'passive_snapshot.json').read_text());d['stage']='FAIL_LIVE_FRESHNESS_ROLLED_BACK';d['motion_authorized']=False;d['owned_processes_stopped']=True;d['rollback_manifest']=json.loads((p/'deployment.json').read_text())
rows=[json.loads(x) for x in (p/'obstacle_probe.jsonl').read_text().splitlines()];last=rows[-1]
d['obstacle']={'operator_report':'Chair 1 m from frontmost robot/feet edge; no robot movement','samples':len(rows),'roi_base_coordinates':{'x':[.55,1.45],'abs_y_lt':.3},'first_roi_lethal':len(rows[0]['roi_lethal']),'last_roi_lethal':len(last['roi_lethal']),'max_roi_lethal':max(len(x['roi_lethal']) for x in rows),'removal_test':'NOT_RUN','object_identity':'Operator placement and new returns consistent; probe does not classify chair vs operator'}
events=[json.loads(x) for x in (p/'passive_observer.jsonl').read_text().splitlines()];seed=[x['wall'] for x in events if x['kind']=='initialpose'][-1];post=[x for x in events if x['kind']=='trust' and x['wall']>=seed];bad=[x for x in post if x['value']['reasons']];d['post_seed']['blocked_samples']=len(bad);d['post_seed']['max_input_ages']['odom_msg']=max(x['value']['input_ages'].get('odom_msg') or 0 for x in post);d['post_seed']['continuous_ready_claim']=False
(p/'passive_snapshot.json').write_text(json.dumps(d,indent=2))
(p/'freshness_events.json').write_text(json.dumps([{'wall':x['wall'],'reasons':x['value']['reasons'],'ages':x['value']['input_ages']} for x in bad],indent=2))
ps=subprocess.check_output(['ps','-eo','user,pid,ppid,args'],text=True)
(p/'process_snapshot_final.txt').write_text('\n'.join(x for x in ps.splitlines() if any(t in x for t in ['hal_sensor_orin_t2d5.yaml','hal_camera','fastlio_mapping --ros-args','python3 /home/agi/control_backend/ws/src/lio/','python3 /home/agi/control_backend/ws/src/navigation/localization_trust_node.py','/nav2_costmap_2d/nav2_costmap_2d --ros-args'] ))+'\n')
for name in ['passive_observer.jsonl','observer.log','observer_light.log','localization.log','trust.log','costmap.log','obstacle_probe.jsonl','obstacle_probe.log']:
 (p/(name+'.gz')).write_bytes(gzip.compress((p/name).read_bytes(),mtime=0))
manifest={f.name:{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in p.iterdir() if f.is_file() and f.name not in ['archive_manifest.json'] and f.suffix not in ['.log','.jsonl','.pid']}
(p/'archive_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({'post_seed':d['post_seed'],'obstacle':d['obstacle'],'stage':d['stage']},indent=2))
