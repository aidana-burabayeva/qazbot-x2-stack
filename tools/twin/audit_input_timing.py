#!/usr/bin/env python3
"""Offline source/arrival timing from finalized SIM bags; never publishes or restamps data."""
import argparse,bisect,json,sqlite3,struct
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();bag=a.run/'frames'
if not (bag/'metadata.yaml').is_file():raise RuntimeError('finalized bag required')
db=sqlite3.connect(next(bag.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
topics={i:(name,typ) for i,name,typ in db.execute('select id,name,type from topics')}
streams={};statuses=[]
for tid,arrival,raw in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name,typ=topics[tid]
    if raw[1]!=1:raise RuntimeError('requires recorded little-endian CDR')
    if name in ['/aima/hal/odom/state','/qazbot_sim/odom','/qazbot_sim/scan']:
        sec,nano=struct.unpack_from('<iI',raw,4)
        if not (sec>0 and 0<=nano<10**9):raise RuntimeError('invalid source header')
        streams.setdefault(name,[]).append((arrival/1e9,sec+nano/1e9))
    elif name=='/qazbot_sim/localization_trust/status':
        length=struct.unpack_from('<I',raw,4)[0];s=json.loads(raw[8:8+length-1])
        if s.get('initialized') and any(r.startswith('T1') for r in s.get('reasons',[])):statuses.append((arrival/1e9,s))
db.close();stats={};times={}
for name,rows in streams.items():
    v=np.asarray(rows);gaps=np.diff(v[:,1]);age=v[:,0]-v[:,1];times[name]=v[:,0].tolist()
    stats[name]=dict(samples=len(v),max_source_gap_s=float(gaps.max()) if len(gaps) else None,max_arrival_gap_s=float(np.diff(v[:,0]).max()) if len(v)>1 else None,p95_arrival_age_s=float(np.percentile(age,95)),max_arrival_age_s=float(age.max()))
failures=[]
for arrival,s in statuses:
    latest={}
    for name,rows in streams.items():
        i=bisect.bisect_right(times[name],arrival)-1
        if i>=0:latest[name]=dict(source_age_s=arrival-rows[i][1],since_arrival_s=arrival-rows[i][0])
    failures.append(dict(status_arrival_ros_s=arrival,reasons=s['reasons'],consumer_ages=s.get('input_ages'),independent_recorder_latest=latest))
result=dict(streams=stats,initialized_T1_statuses=failures,note='Status alignment uses recorder arrival, not the unrecorded evaluator snapshot instant. It localizes delay but does not prove its cause.')
(a.run/'input_timing_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(streams=stats,initialized_T1_statuses=len(failures))))
