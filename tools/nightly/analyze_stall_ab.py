#!/usr/bin/env python3
"""Offline pose-only analysis of the single s1_0046 yaw_lin_eps A/B (no ROS or sim control)."""
import argparse
import bisect
import csv
import gzip
import json
import math
from pathlib import Path
import statistics


def mean(xs):return statistics.mean(xs) if xs else None

def analyze(directory):
    directory=Path(directory)
    result=json.loads((directory/'result.json').read_text());goal=result['goals'][0]
    begin=goal['begin_ros_ns']/1e9;end=goal['result_ros_ns']/1e9
    with gzip.open(directory/'motion.csv.gz','rt') as f:raw=list(csv.DictReader(f))
    kinds={}
    for r in raw:
        t=int(r['received_ros_ns'])/1e9
        if begin<=t<=end:
            kinds.setdefault(r['kind'],[]).append((t,*[float(r[k]) for k in ['x','y','yaw','vx','vy','wz']]))
    for rows in kinds.values():rows.sort()
    truth=kinds.get('truth',[])
    def commands(kind,a,b):return [r for r in kinds.get(kind,[]) if a<=r[0]<=b]
    def describe(a,b):
        poses=[r for r in truth if a<=r[0]<=b]
        if len(poses)<2:return {}
        unwrapped=[poses[0][3]]
        for x,y in zip(poses,poses[1:]):unwrapped.append(unwrapped[-1]+math.remainder(y[3]-x[3],2*math.pi))
        stats={'duration_s':poses[-1][0]-poses[0][0],
            'pose_first':poses[0][1:4],'pose_last':poses[-1][1:4],
            'dxy_m':math.hypot(poses[-1][1]-poses[0][1],poses[-1][2]-poses[0][2]),
            'dyaw_rad':unwrapped[-1]-unwrapped[0],
            'yaw_excursion_rad':max(unwrapped)-min(unwrapped),
            'pose_yaw_rate_rad_s':(unwrapped[-1]-unwrapped[0])/(poses[-1][0]-poses[0][0])}
        for kind in ['nav_ns','authorized','mc']:
            rows=commands(kind,a,b)
            stats[kind]={'n':len(rows),'vx_mean':mean([r[4] for r in rows]),'wz_mean':mean([r[6] for r in rows]),
                'wz_min':min((r[6] for r in rows),default=None),'wz_max':max((r[6] for r in rows),default=None),
                'negative_turn_fraction':mean([r[6]<-.05 for r in rows]),
                'vx_under_002_fraction':mean([abs(r[4])<.02 for r in rows]),
                'vx_under_010_fraction':mean([abs(r[4])<.10 for r in rows])}
        return stats
    # Fixed historical diagnosis window is declared in the task; never choose a favourable interval per arm.
    fixed=describe(begin+60,min(begin+130,end))
    stalls=[]
    for i in range(0,len(truth),10):
        a=truth[i][0];b=a+10
        if b>end:break
        d=describe(a,b)
        if d.get('duration_s',0)<9:continue
        auth=d['authorized'];mc=d['mc']
        if (d['dxy_m']<.10 and d['yaw_excursion_rad']<.20 and
            (auth['negative_turn_fraction'] or 0)>.8 and abs(mc['wz_mean'] or 0)>.2):
            stalls.append({'from_goal_s':a-begin,'to_goal_s':b-begin,**d})
    states=[]
    for line in (directory/'status.jsonl').read_text().splitlines():
        r=json.loads(line)
        if r.get('kind')=='adapter':states.append(r)
    eps=sorted({r['data']['yaw_lin_eps'] for r in states if 'yaw_lin_eps' in r['data']})
    counters=[r['data'].get('counters',{}) for r in states]
    last=counters[-1] if counters else {}
    log=(directory/'stack.log').read_text(errors='replace')
    return {'nav_result':result['result'],'nav_status':goal['status'],'goal_duration_s':end-begin,
        'xy_error_m':goal['xy_error'],'yaw_error_rad':goal['yaw_error'],'controller_parameters':result['controller_parameters'],
        'observed_yaw_lin_eps':eps,'final_adapter_counters':last,
        'yaw_loop_per_published':last.get('yaw_loop',0)/max(1,last.get('published',0)),
        'fixed_60_130s':fixed,'commanded_stall_10s_window_count':len(stalls),'first_commanded_stall':stalls[0] if stalls else None,
        'failed_progress_log_count':log.count('Failed to make progress'),
        'limitation':'Pose-derived response. No MuJoCo contact forces are recorded; one A/B pair is not a repeatability estimate.'}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('directory');ap.add_argument('--out',required=True);args=ap.parse_args()
    root=Path(args.directory)
    result={arm:analyze(root/arm) for arm in ['A','B']}
    for k in ['controller_parameters']:
        assert result['A'][k]==result['B'][k],(k,'A/B changed more than epsilon')
    metadata=[json.loads((root/arm/'metadata.json').read_text()) for arm in ['A','B']]
    assert metadata[0]['scenario']==metadata[1]['scenario']
    assert metadata[0]['source_sha256']==metadata[1]['source_sha256']
    assert result['A']['observed_yaw_lin_eps']==[.02] and result['B']['observed_yaw_lin_eps']==[.10]
    result['invariants']={'scenario_equal':True,'source_hashes_equal':True,'controller_parameters_equal':True,'only_experimental_setting':'yaw_lin_eps .02 -> .10; vx preserved'}
    Path(args.out).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
