#!/usr/bin/env python3
"""Compare matching command windows against an archived .66 native-body proxy.
No fitting. No external truth claim; one replay can falsify a match, never certify it.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np

LIMITS=dict(yaw_rmse_deg=5.,final_yaw_error_deg=5.,xy_rmse_m=.05,final_xy_error_m=.05)


def relative(t,xy,yaw):
    yaw=np.unwrap(yaw);h=np.interp(0,t,yaw);origin=np.array([np.interp(0,t,xy[:,j]) for j in range(2)])
    c,s=np.cos(h),np.sin(h)
    return (xy-origin)@np.array([[c,-s],[s,c]]),yaw-h


def assess(reference,simulation):
    ref=json.loads(Path(reference).read_text());r=np.array(ref['trajectory'])
    with np.load(simulation) as data:o=data['odom'];meta=data['meta'];cmd=data['cmd']
    expected=ref['command']
    if abs(meta[0]-expected['forward'])>1e-6 or abs(meta[2]-expected['angular'])>1e-6 or abs(meta[3]-expected['duration_s'])>.05:
        raise ValueError('command conditions differ; comparison refused')
    q=o[:,4:8];yaw=np.arctan2(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),1-2*(q[:,1]**2+q[:,2]**2))
    st=o[:,0]-meta[6];rt=r[:,0]
    sxy,syaw=relative(st,o[:,1:3],yaw);rxy,ryaw=relative(rt,r[:,1:3],r[:,4])
    end=min(meta[3]+3,st[-1],rt[-1]);t=np.arange(0,end,.1)
    sx=np.column_stack([np.interp(t,st,sxy[:,j]) for j in range(2)]);rx=np.column_stack([np.interp(t,rt,rxy[:,j]) for j in range(2)])
    sa=np.interp(t,st,syaw);ra=np.interp(t,rt,ryaw)
    dyaw=np.degrees(sa-ra);dxy=np.linalg.norm(sx-rx,axis=1)
    metrics=dict(yaw_rmse_deg=float(np.sqrt(np.mean(dyaw**2))),final_yaw_error_deg=float(abs(dyaw[-1])),
                 xy_rmse_m=float(np.sqrt(np.mean(dxy**2))),final_xy_error_m=float(dxy[-1]))
    return dict(reference=ref['id'],split=ref['split'],reference_qualification=ref['qualification'],
                simulation=str(simulation),command=expected,comparison_end_s=float(t[-1]),
                real_final_yaw_deg=float(np.degrees(ra[-1])),sim_final_yaw_deg=float(np.degrees(sa[-1])),
                metrics=metrics,provisional_limits=LIMITS,matching={k:metrics[k]<=LIMITS[k] for k in LIMITS},
                physically_validated=False,reason='One sequential replay and native odometry proxy are insufficient for qualification; independent repeats, matched startup and external reference required.',
                trace=dict(t=t.tolist(),real_yaw_deg=np.degrees(ra).tolist(),sim_yaw_deg=np.degrees(sa).tolist(),xy_error_m=dxy.tolist()))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('reference');ap.add_argument('simulation');ap.add_argument('output',type=Path);a=ap.parse_args()
    result=assess(a.reference,a.simulation);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))
