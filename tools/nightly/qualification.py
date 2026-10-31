"""Fail-closed authorization of unattended LAB execution from stored, version-matched E2E evidence."""
import json
import sqlite3
from pathlib import Path

import optimization_archive

GATES = ('aimdk_alignment','sensor_parity','nav2_marking_clearing','official_mc_mujoco',
         'frame_anchor','raw_collision_contacts','short_d1_route','resource_profile',
         'stop_resume','claude_cli_task','codex_cli_task','runtime_e2e')
# These scopes are not prerequisites for the first bounded stationary run or baseline E2E.
SCOPED_GATES = {'dynamic_office':('dynamic_obstacle','blocked_route','missing_scan'),
                'reset_freshness':('odometry_reset','localization_failure','sensor_delays_dropouts'),
                'stress':('cpu_contention','collision','recovery')}


def check(db, receipt_path, root, integration_sha, robot_sha, map_sha, stack_digest, simulation_digest):
    path = Path(receipt_path)
    if not path.is_file() or path.is_symlink():
        return ['missing_runtime_e2e_receipt']
    try:
        receipt = json.loads(path.read_text())
        rid = receipt['archive_run_id']
        row = db.execute('SELECT payload FROM optimization_runs WHERE run_id=?', (rid,)).fetchone()
        if row is None:
            return ['qualification_archive_record_missing']
        record = json.loads(row['payload'])
        if not isinstance(record, dict):
            return ['invalid_qualification_record']
        errors = optimization_archive.failures(db, record, Path(root))
        if record.get('fidelity') != 'L3':
            errors.append('official_l3_required')
        expected = {'git_sha':integration_sha, 'robot_sha':robot_sha, 'map_sha256':map_sha,
                    'stack_image_digest':stack_digest, 'simulation_image_digest':simulation_digest}
        errors.extend('stale_or_unknown:' + k for k,v in expected.items() if not v or record.get(k)!=v)
        gates = record.get('runtime_gates', {})
        errors.extend('gate:' + k for k in GATES if not isinstance(gates,dict) or gates.get(k) is not True)
        evidence = record.get('cli_results', {})
        for provider in ('claude','codex'):
            key = evidence.get(provider) if isinstance(evidence,dict) else None
            stored = db.execute("SELECT t.owner,t.phase,t.state,r.verdict,r.status,r.git_sha FROM results r JOIN tasks t ON t.name=r.task WHERE r.result_key=?", (key,)).fetchone()
            if (not stored or stored['owner']!=provider or stored['phase']!='VERIFIED' or stored['state']!='COMPLETED' or
                    stored['verdict']!='PASS' or stored['status']!='STORED' or stored['git_sha']!=integration_sha):
                errors.append('missing_verified_cli_task:' + provider)
        return sorted(set(errors))
    except (ValueError, KeyError, TypeError, OSError, sqlite3.Error) as exc:
        return ['invalid_runtime_e2e_receipt:' + type(exc).__name__]


def first_l3_errors(report):
    """Prerequisites for one bounded run, not a receipt for all future navigation scenarios."""
    required=('sim.official_image_id','tests.frame_anchor','baseline.integration_sha256','contract.robot_env_vs_snapshot','contract.lab_env','map.host_sha256',
              'sim.stack_lite_healthy','sim.image_patch_is_documented','tf.owners','sensor.costmap_contract',
              'isolation.network_internal','isolation.no_host_network','isolation.no_default_route')
    checks={r['name']:r['status'] for r in report.get('checks',[])}
    return ['preflight:'+k for k in required if checks.get(k)!='PASS']
