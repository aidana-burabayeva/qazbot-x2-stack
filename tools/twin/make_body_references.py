#!/usr/bin/env python3
"""Package existing same-stamp native-body reconstructions; not external ground truth."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ap=argparse.ArgumentParser();ap.add_argument('evidence',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args()
for trial,split in [('trial05','development'),('trial06','holdout')]:
    p=a.evidence/(trial+'_data')/'kinematic_series.json';d=json.loads(p.read_text());on,off=d['on'],d['off']
    rows=np.array(d['native_base']);rows=rows[(rows[:,0]>=on-1)&(rows[:,0]<=off+3)]
    result=dict(id='robot66_'+trial,split=split,qualification='native leg odometry corrected to base_link with recorded vendor TF; proxy, not external truth',
                provenance=dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()),
                command=dict(forward=0.,angular=.5,duration_s=off-on),
                columns=['time_from_command_s','world_x','world_y','world_z','body_yaw_rad','tilt_deg'],
                trajectory=np.column_stack([rows[:,0]-on,rows[:,1:]]).tolist(),
                context='trial05 before chair removal; trial06 after removal. Environment/state differences not reproduced by flat-floor MuJoCo.')
    (a.output/(trial+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(trial,len(rows),off-on)
