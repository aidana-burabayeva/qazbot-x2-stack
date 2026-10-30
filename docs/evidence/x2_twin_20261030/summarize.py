#!/usr/bin/env python3
"""Recompute the small evidence summary; reads snapshots, never contacts a robot."""
import hashlib,json,math,pathlib,statistics
root=pathlib.Path(__file__).resolve().parent
s=root/'snapshot'
read=lambda name:json.loads((s/name).read_text())
inv=read('bag_inventory.json');scans=read('scan_samples.json');eps=read('initialpose_episodes.json')
result={'bag_count':len(inv),'mcap_count':sum(len(b['files']) for b in inv),'mcap_bytes':sum(f['bytes'] for b in inv for f in b['files']),'core':next(b for b in inv if b['path'].endswith('twin_core_190402')),'scan_samples':{},'initialpose_episodes':[],'deployed_snapshot_checks':[]}
for topic in sorted({v['topic'] for v in scans}):
 a=[v for v in scans if v['topic']==topic];lag=sorted(v['lag_s'] for v in a)
 result['scan_samples'][topic]={'sample_count':len(a),'method':'first 25 messages per topic per core split, not full-session percentiles','lag_p50_s':statistics.median(lag),'lag_p95_s':lag[min(len(lag)-1,int(.95*len(lag)))],'lag_max_s':max(lag),'median_finite_bins':statistics.median(v['finite'] for v in a)}
for ep in eps:
 a=ep['amcl_outputs']
 if not a:continue
 result['initialpose_episodes'].append({'seed_epoch':ep['seed']['t'],'outputs_in_first_8s_before_next_seed':len(a),'first_xy_m':a[0]['delta_xy'],'last_xy_m':a[-1]['delta_xy'],'last_yaw_deg':a[-1]['delta_yaw_deg']})
for v in read('deployed_manifest.json'):
 p=s/v['snapshot'];actual=hashlib.sha256(p.read_bytes()).hexdigest();ok=actual==v['sha256'];result['deployed_snapshot_checks'].append({'file':v['snapshot'],'sha256_ok':ok});assert ok,p
checks=read('mcap_integrity.json');assert all(v['summary_ok'] for v in checks)
result['mcap_summary_read_checks']=len(checks)
result['limitations']=['Original MCAPs remain on robot; local hashes do not imply an off-robot bag backup.','MCAP footer/summary readability and selected decoding checked; no full CRC validation performed.','Initialpose deltas are relative to operator seed, not independent ground-truth error.','Sequential, unmatched sessions cannot isolate a causal improvement.']
(root/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['bag_count','mcap_count','mcap_bytes','mcap_summary_read_checks','scan_samples','initialpose_episodes']},indent=2))
