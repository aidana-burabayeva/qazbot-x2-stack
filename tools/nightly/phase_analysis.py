#!/usr/bin/env python3
"""Phase metrics of one goal from tests/p20_sim recorder output (result.json + recorder/motion.csv).

Timeline is relative to goal send (ROS clock). Truth pose is the simulator pose; MC rows are the commands handed to MC.
usage: phase_analysis.py <run_dir> [...]   -> JSON list on stdout
"""
import csv
import json
import math
import sys


def analyse(run):
    res = json.load(open(f'{run}/result.json'))
    goal = res['goals'][0]
    tx, ty, tyaw = goal['target']
    t0 = goal['begin_ros_ns']
    prm = res.get('controller_parameters', {})
    xy_tol = prm.get('goal_checker.xy_goal_tolerance')
    yaw_tol = prm.get('goal_checker.yaw_goal_tolerance')
    truth, mc = [], []
    for r in csv.DictReader(open(f'{run}/recorder/motion.csv')):
        t = (int(r['received_ros_ns']) - t0) / 1e9
        if t < 0:
            continue
        if r['kind'] == 'truth':
            truth.append((t, float(r['x']), float(r['y']), float(r['yaw'])))
        elif r['kind'] == 'mc':
            mc.append((t, float(r['vx']), float(r['wz'])))
    xy = [(t, math.hypot(x - tx, y - ty), abs(math.remainder(w - tyaw, 2 * math.pi))) for t, x, y, w in truth]
    inside = [(t, d <= xy_tol) for t, d, _ in xy]
    entries = [t for (t, i), (_, p) in zip(inside[1:], inside) if i and not p]
    exits = [t for (t, i), (_, p) in zip(inside[1:], inside) if p and not i]
    lin = [t for t, vx, _ in mc if abs(vx) > 0.05]
    ang = [t for t, _, wz in mc if abs(wz) > 0.05]
    end = (goal['result_ros_ns'] - t0) / 1e9
    cmd_end = max(lin + ang + [0])
    coast = [(t, d, a) for t, d, a in xy if t >= cmd_end]
    return {
        'run': run.rstrip('/').split('/')[-1], 'status': goal['status'], 'fault': goal['coordinator']['fault'],
        'goal_time_s': round(end, 2), 'xy_tol': xy_tol, 'yaw_tol': yaw_tol,
        'final_xy_err_m': round(goal['xy_error'], 3), 'final_yaw_err_rad': round(abs(goal['yaw_error']), 3),
        'xy_exit_count_after_first_entry': len(exits), 'first_xy_entry_s': round(entries[0], 2) if entries else None,
        'last_xy_entry_s': round(entries[-1], 2) if entries else None,
        'last_nonzero_linear_cmd_s': round(max(lin), 2) if lin else None,
        'last_nonzero_angular_cmd_s': round(max(ang), 2) if ang else None,
        'max_xy_dev_while_turning_m': round(max((d for t, d, _ in xy if t > 2), default=0), 3),
        'drift_after_last_cmd_m': round(abs(coast[-1][1] - coast[0][1]), 3) if len(coast) > 1 else None,
        'yaw_cross_count': sum(1 for (_, _, a), (_, _, b) in zip(xy[1:], xy) if (a < 0.1) != (b < 0.1)),
    }


if __name__ == '__main__':
    print(json.dumps([analyse(r) for r in sys.argv[1:]], indent=1))
