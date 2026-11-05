#!/usr/bin/env python3
"""Standalone research plot of measured turn errors, without hiding rejected cases."""
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

folders=[Path(x) for x in sys.argv[1:]]
fig,axes=plt.subplots(1,len(folders),figsize=(6*len(folders),4.5),squeeze=False)
for ax,folder in zip(axes[0],folders):
    r=json.loads((folder/'summary.json').read_text());rows=r['results'];targets=sorted({x['target_deg'] for x in rows})
    leads=sorted({x['lead_time'] for x in rows})
    for i,lead in enumerate(leads):
        for x in rows:
            if x['lead_time']!=lead:continue
            jitter=(i-(len(leads)-1)/2)*.18+(x['repeat']-.5)*.06
            ax.scatter(targets.index(x['target_deg'])+jitter,x['frames']['pelvis']['error_deg'],
                       marker='o' if x['passed'] else 'x',color=f'C{i}',s=65,
                       label=f'lead={lead:.2f}s' if x is next(y for y in rows if y['lead_time']==lead) else None)
    limit=math.degrees(.1);ax.axhspan(-limit,limit,color='green',alpha=.10,label='0.1 rad tolerance')
    ax.axhline(limit,color='green',linestyle='--',linewidth=.8);ax.axhline(-limit,color='green',linestyle='--',linewidth=.8)
    ax.axhline(0,color='gray',linewidth=.5);ax.set_xticks(range(len(targets)),[str(int(x)) for x in targets])
    ax.set(title=f'{folder.name}\n{r["passed"]}/{r["total"]} full checks passed',xlabel='requested turn (degrees)',ylabel='final pelvis angle error (degrees)')
    ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
fig.tight_layout();destination=folders[-1]/'angle_errors.png';fig.savefig(destination,dpi=150);print(destination)
