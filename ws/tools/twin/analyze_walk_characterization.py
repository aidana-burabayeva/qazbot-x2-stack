#!/usr/bin/env python3
"""Reassess complete direct-walk records; never edit the original sweep summary."""
import argparse,json,math
from pathlib import Path
import numpy as np
from parallel_turn_sweep import analyze_walk,write_json

def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();results=[]
    for case in sorted(a.directory.glob('case_*')):
        if not case.is_dir() or not (case/'meta.json').exists():continue
        meta=json.loads((case/'meta.json').read_text());d=np.load(case/'data.npz');o=d['odom'];f=d['frames'];r=dict(case=case.name,**analyze_walk(case))
        active=(o[:,0]>=meta['walk_on'])&(o[:,0]<=meta['walk_off']+1.)
        gaps=np.diff(o[active,0]);r['max_active_odom_gap_s']=float(gaps.max()) if len(gaps) else None
        r['usable_speed_measurement']=bool(r['aborted'] is None and r['valid_measurement'] and len(gaps) and gaps.max()<.15)
        frame_results={};i=np.searchsorted(f[:,0],meta['walk_on']);j=np.searchsorted(f[:,0],meta['walk_off'])
        for k,name in enumerate(['pelvis','torso','left_ankle','right_ankle']):
            b=2+k*4;angle=f[i,b+3];direction=np.array([math.cos(angle),math.sin(angle)])
            frame_results[name]=dict(coast_m=float((f[-1,b:b+2]-f[j,b:b+2])@direction),
              yaw_change_deg=math.degrees(math.atan2(math.sin(f[-1,b+3]-angle),math.cos(f[-1,b+3]-angle))))
        r['frames']=frame_results;results.append(r)
    write_json(a.directory/'walk_analysis.json',dict(results=results,usable=sum(r['usable_speed_measurement'] for r in results),total=len(results),note='Aborted/stale trials excluded; original summary retained unchanged. Speeds use host monotonic time, RTF not measured.'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for r in results:
        if not r['usable_speed_measurement']:continue
        d=np.load(a.directory/r['case']/'data.npz');m=json.loads((a.directory/r['case']/'meta.json').read_text());o=d['odom'];i=np.searchsorted(o[:,0],m['walk_off']);delta=np.linalg.norm(o[:,1:3]-o[i,1:3],axis=1);sel=o[:,0]>=m['walk_off']
        axes[0].scatter(r['commanded_speed'],r['measured_speed']);axes[1].plot(o[sel,0]-m['walk_off'],delta[sel],label=f"{r['case']}: {r['commanded_speed']:.2f}")
    axes[0].plot([0,.5],[0,.5],'k--',alpha=.4);axes[0].set(xlabel='Command (m/s)',ylabel='Measured forward speed (m/s)');axes[1].set(xlabel='Seconds since zero',ylabel='Pelvis displacement since zero (m)');axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(a.directory/'walk_response.png',dpi=150)
    print(json.dumps(dict(total=len(results),usable=sum(r['usable_speed_measurement'] for r in results))))
if __name__=='__main__':main()
