#!/usr/bin/env python3
"""Analyze sim_step_run NPZ using projected pelvis heading and pose differences.
Rates use host monotonic seconds, not simulator time. No chest/foot claims.
Usage: analyze_step_truth.py DIRECTORY (writes summary.json)
"""
import json
from pathlib import Path
import sys
import numpy as np


def analyze(path):
    with np.load(path) as data:
        o, c, meta = data['odom'], data['cmd'], data['meta']
    if len(o) < 10 or len(c) < 10:
        raise ValueError(f'insufficient telemetry: {path}')
    t = o[:, 0]
    # quaternion columns qx=4,qy=5,qz=6,qw=7; projection valid upright.
    yaw = np.unwrap(np.arctan2(2*(o[:, 4]*o[:, 5]+o[:, 6]*o[:, 7]), 1-2*(o[:, 5]**2+o[:, 6]**2)))
    on, off = meta[6:8]
    i, j = np.searchsorted(t, [on, off])
    i, j = min(i, len(t)-1), min(j, len(t)-1)
    active = (t >= on) & (t < off)
    steady = (t >= max(on + 1, off - 4)) & (t < off)
    head = yaw[i]
    forward = (o[:, 1]-o[i, 1])*np.cos(head)+(o[:, 2]-o[i, 2])*np.sin(head)
    fit = lambda values: float(np.polyfit(t[steady]-on, values[steady], 1)[0]) if steady.sum() > 3 else None
    return dict(name=path.stem, command=dict(forward=float(meta[0]), angular=float(meta[2]), seconds=float(meta[3])),
                samples=len(o), min_height=float(o[:, 3].min()), max_command_gap_s=float(np.diff(c[:, 0]).max()),
                turn_at_zero_deg=float(np.degrees(yaw[j]-yaw[i])), final_turn_deg=float(np.degrees(yaw[-1]-yaw[i])),
                peak_turn_deg=float(np.degrees(np.abs(yaw[active]-yaw[i]).max())),
                steady_yaw_rad_s=fit(yaw), steady_forward_m_s=fit(forward),
                distance_at_zero_m=float(np.linalg.norm(o[j, 1:3]-o[i, 1:3])),
                final_distance_m=float(np.linalg.norm(o[-1, 1:3]-o[i, 1:3])),
                post_zero_displacement_m=float(np.linalg.norm(o[-1, 1:3]-o[j, 1:3])))


if __name__ == '__main__':
    directory = Path(sys.argv[1])
    rows = [analyze(p) for p in sorted(directory.glob('*.npz'))]
    text = json.dumps(dict(timebase='host monotonic; not simulation time', runs=rows), indent=2)
    (directory / 'summary.json').write_text(text + '\n')
    print(text)
