#!/usr/bin/env python3
"""Append-only experiment archive in the existing nq SQLite DB. Does not run optimizers or workers.

Records, including incomplete/failed experiments, are retained. Eligibility is deliberately
separate from ingestion. L1, L2 and L3/version cohorts never compete with each other.
"""
import argparse
import hashlib
import json
import math
import re
import time
from pathlib import Path

import nq

# Everything here must match before comparing controller configurations. Source version is
# intentionally part of the cohort: a code fix requires reevaluating affected configurations.
COHORT = ('fidelity', 'git_sha', 'map_sha256', 'stack_image_digest', 'simulation_image_digest',
          'world_sha256', 'ros_version', 'nav2_version', 'mc_version', 'scenario_set_sha256',
          'seeds', 'fixed_parameters_sha256', 'safety_contract_sha256', 'baseline_id')
FLAGS = ('collision', 'unknown_traversal', 't1_failure', 't5_failure', 'goal_tolerance_failure',
         'resource_degradation', 'mc_cycles_missed', 'incomplete_scenarios')
METRICS = ('duration_s', 'position_error_m', 'yaw_error_rad', 'min_clearance_m', 'success_rate')
QUALIFIED = {'L1': 'MODEL_VERIFIED', 'L2': 'NAV2_VERIFIED', 'L3': 'MC_MUJOCO_QUALIFIED'}


def init(db):
    db.execute('CREATE TABLE IF NOT EXISTS optimization_runs(run_id TEXT PRIMARY KEY, payload TEXT NOT NULL, recorded REAL NOT NULL)')


def record(db, payload):
    """Content-addressed ingestion never overwrites failed or rejected observations."""
    init(db)
    raw = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)
    rid = hashlib.sha256(raw.encode()).hexdigest()
    db.execute('INSERT OR IGNORE INTO optimization_runs VALUES(?,?,?)', (rid, raw, time.time()))
    return rid


def failures(db, d, root):
    errors = []
    if not isinstance(d, dict):
        return ['record is not an object']
    for key in (*COHORT, 'config_id', 'controller_parameters', 'scenarios', 'evidence', 'qualification_status', 'independent_validation', 'result_key'):
        if key not in d or d[key] is None or d[key] == '' or d[key] == []:
            errors.append('missing:' + key)
    if not isinstance(d.get('controller_parameters'), dict) or not d['controller_parameters']:
        errors.append('missing_controller_parameters')
    if not isinstance(d.get('seeds'), list) or not d['seeds'] or any(type(v) is not int for v in d['seeds']):
        errors.append('invalid_seeds')
    if not isinstance(d.get('scenarios'), list) or not d['scenarios'] or any(not isinstance(v, str) or not v for v in d['scenarios']):
        errors.append('invalid_scenarios')
    if not re.fullmatch('[0-9a-f]{40}', str(d.get('git_sha', ''))):
        errors.append('invalid_git_sha')
    for key in ('ros_version', 'nav2_version', 'mc_version'):
        if not isinstance(d.get(key), str) or d[key].upper() in ('', 'UNKNOWN', 'UNVERIFIED'):
            errors.append('unknown_version:' + key)
    if d.get('fidelity') not in QUALIFIED or d.get('qualification_status') != QUALIFIED.get(d.get('fidelity')):
        errors.append('unqualified_fidelity')
    if d.get('independent_validation') != 'PASS':
        errors.append('independent_validation')
    for key in ('map_sha256', 'world_sha256', 'scenario_set_sha256', 'fixed_parameters_sha256', 'safety_contract_sha256'):
        if not re.fullmatch('[0-9a-f]{64}', str(d.get(key, ''))):
            errors.append('invalid_hash:' + key)
    for key in ('stack_image_digest', 'simulation_image_digest'):
        if not re.fullmatch('sha256:[0-9a-f]{64}', str(d.get(key, ''))):
            errors.append('invalid_digest:' + key)
    flags = d.get('hard_failure_flags', {})
    if (not isinstance(flags, dict) or any(flags.get(key) is not False for key in FLAGS) or
            any(value is not False for value in flags.values())):
        errors.append('hard_failure_or_missing_gate')
    metrics = d.get('metrics', {})
    if (not isinstance(metrics, dict) or any(type(metrics.get(k)) not in (int, float) or
            not math.isfinite(metrics[k]) or metrics[k] < 0 for k in METRICS)):
        errors.append('missing_or_invalid_metrics')
    elif metrics['success_rate'] != 1 or metrics['duration_s'] <= 0 or metrics['min_clearance_m'] <= 0:
        errors.append('incomplete_or_failed_scenarios')
    if d.get('fidelity') == 'L3':
        profile = d.get('resource_profile', {})
        measured = ('cpu_peak_percent', 'ram_peak_bytes', 'disk_delta_bytes', 'evidence_bytes',
                    'rtf_min', 'missed_mc_cycles', 'wall_duration_s')
        if (not isinstance(profile, dict) or any(type(profile.get(k)) not in (int, float) or
                not math.isfinite(profile[k]) or profile[k] < 0 for k in measured)):
            errors.append('missing_l3_resource_measurements')
        elif (type(profile.get('workers')) is not int or profile['workers'] != 1 or profile['missed_mc_cycles'] != 0 or
              profile['rtf_min'] <= 0 or profile['wall_duration_s'] <= 0 or
              profile.get('timing_isolation_status') != 'PASS'):
            errors.append('l3_resources_unqualified')
    row = db.execute('SELECT verdict,status,git_sha,map_sha FROM results WHERE result_key=?', (d.get('result_key', ''),)).fetchone()
    if (not row or row['verdict'] != 'PASS' or row['status'] != 'STORED' or
            row['git_sha'] != d.get('git_sha') or row['map_sha'] != d.get('map_sha256')):
        errors.append('no_matching_stored_pass_result')
    evidence = d.get('evidence', [])
    if not isinstance(evidence, list) or not evidence:
        errors.append('missing_evidence')
    else:
        for item in evidence:
            try:
                rel = item['path']
                if not isinstance(rel, str) or Path(rel).is_absolute() or '..' in Path(rel).parts:
                    raise ValueError('unsafe path')
                path = root / rel
                if any(p.is_symlink() for p in (path, *path.parents)):
                    raise ValueError('symlink')
                path.resolve().relative_to(root.resolve())
                if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
                    raise ValueError('hash mismatch')
            except (KeyError, TypeError, OSError, ValueError):
                errors.append('invalid_evidence')
    return sorted(set(errors))


def objectives(d):
    m = d['metrics']
    return (m['duration_s'], m['position_error_m'], m['yaw_error_rad'], -m['min_clearance_m'])


def archive(db, root=nq.ROOT):
    init(db)
    groups, all_runs = {}, []
    for row in db.execute('SELECT run_id,payload FROM optimization_runs ORDER BY recorded,run_id'):
        d = json.loads(row['payload'])
        rejected = failures(db, d, Path(root))
        entry = dict(run_id=row['run_id'], record=d, eligibility='REJECTED' if rejected else 'ELIGIBLE', reasons=rejected)
        all_runs.append(entry)
        if isinstance(d, dict) and all(k in d for k in COHORT):
            cohort = {k: d[k] for k in COHORT}
            key = hashlib.sha256(json.dumps(cohort, sort_keys=True).encode()).hexdigest()
            group = groups.setdefault(key, {'cohort': cohort, 'runs': []})
            group['runs'].append(entry)
    for group in groups.values():
        safe = [r for r in group['runs'] if r['eligibility'] == 'ELIGIBLE']
        front = [a for a in safe if not any(all(x <= y for x, y in zip(objectives(b['record']), objectives(a['record']))) and
                                         objectives(a['record']) != objectives(b['record']) for b in safe)]
        pick = lambda key: min(safe, key=lambda r: (*key(r['record']), *objectives(r['record']), r['run_id']))['run_id'] if safe else None
        group['selected'] = {
            'baseline': [r['run_id'] for r in group['runs'] if r['record'].get('config_id') == group['cohort']['baseline_id']],
            'safest': pick(lambda d: (-d['metrics']['min_clearance_m'],)),
            'most_accurate': pick(lambda d: (d['metrics']['position_error_m'], d['metrics']['yaw_error_rad'])),
            'fastest_among_safe': pick(lambda d: (d['metrics']['duration_s'],)),
            'pareto': sorted(r['run_id'] for r in front),
        }
    return {'cohorts': groups, 'all_runs': all_runs, 'automatic_optimization_enabled': False,
            'note': 'Archive eligibility is not MC dynamics qualification or hardware acceptance. No worker is launched by this module.'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='command', required=True)
    add = sub.add_parser('record'); add.add_argument('file', type=Path)
    view = sub.add_parser('view'); view.add_argument('--out', type=Path)
    a = ap.parse_args()
    db = nq.connect()
    if a.command == 'record':
        print(record(db, json.loads(a.file.read_text())))
    else:
        data = json.dumps(archive(db), indent=2, allow_nan=False) + '\n'
        if a.out:
            # Archive exports are snapshots; original observations remain immutable in SQLite.
            a.out.parent.mkdir(parents=True, exist_ok=True)
            temp = a.out.with_suffix(a.out.suffix + '.tmp')
            temp.write_text(data); temp.replace(a.out)
        else:
            print(data, end='')


if __name__ == '__main__':
    main()
