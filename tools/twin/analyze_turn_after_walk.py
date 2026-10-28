#!/usr/bin/env python3
"""Summarize official-MC probe commands and joined pelvis/torso/ankle truth."""
import json,math
from pathlib import Path
import sys
import numpy as np

root=Path(sys.argv[1]);rows=[]
for path in sorted(root.glob('*/data.npz')):
    meta=json.loads(path.with_name('meta.json').read_text());d=np.load(path);o=d['odom'];f=d['frames'];c=d['cmd'];on,off=meta['turn_on'],meta['turn_off']
    row=dict(name=path.parent.name,angular=meta['angular'],kick=meta['kick'],kick_seconds=meta['kick_seconds'],aborted=meta['aborted'],frames={})
    t=f[:,0];i,j=np.searchsorted(t,[on,off]);i=min(i,len(t)-1);j=min(j,len(t)-1);steady=(t>=off-4)&(t<off)
    for k,name in enumerate(('pelvis','torso','left_ankle','right_ankle')):
        base=2+4*k;heading=np.unwrap(f[:,base+3]);xy=f[:,base:base+2]
        row['frames'][name]=dict(turn_at_zero_deg=float(np.degrees(heading[j]-heading[i])),final_turn_deg=float(np.degrees(heading[-1]-heading[i])),
            steady_yaw_rad_s=float(np.polyfit(t[steady],heading[steady],1)[0]),max_xy_from_turn_start_m=float(np.linalg.norm(xy[i:]-xy[i],axis=1).max()))
    row.update(min_pelvis_height_m=float(o[:,3].min()),max_fk_hal_join_s=float(f[:,1].max()),max_cmd_gap_s=float(np.diff(c[:,0]).max()),max_angular_command=float(np.abs(c[:,3]).max()))
    rows.append(row)
(root/'summary.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows,indent=2))
