#!/usr/bin/env python3
"""Terminal bootstrap of the existing lab; no second task runner."""
import argparse
import fcntl
import json
import subprocess
import sys
from pathlib import Path

import lab
import nq
import preflight
import qualification

ROOT, RUN = nq.ROOT, nq.RUN
REVIEWED_ROBOT = '1212d7eda003022d74c6ecc4a2c745fd48f57c2f'
LEGACY = {
    'eng_l2_doorway_stall': 'Historical P20 investigation retained; new P23 reproduction is lab_p1_p23_l2_v1. No blind retry.',
    'verify_worker_kill': 'Expected fault injection exit -9; partial evidence retained. Do not retry as engineering.',
    'make_nightly_20261030': 'Historical vendor/build failure; replaced by rebuilt image preflight. No blind retry.',
    'make_nightly_20261030_r2': 'Historical vendor/DNS build failure; replaced by rebuilt image preflight. No blind retry.',
}
CAMPAIGN = (
    ('lab_p0_amcl_initialpose_v1', 1, 'localization',
     'AMCL initialpose instability: first establish the exact 2026-10-30 hardware episodes showing changes up to 1.72 m and 35.73 degrees from tracked evidence. '
     'Use published origin/robot/stable backend, D1 map, actual_parameters.json and P23. Separate saved-data ROS replay from full AMCL sim E2E; never use truth localization as AMCL evidence. '
     'Reproduce before fixing. Keep T1/T5, collision gates and tolerances intact. Record unavailable data or Nav2 version compatibility as UNVERIFIED. Do not combine localization, MC dynamics and costmap changes.'),
    ('lab_p1_p23_l2_v1', 20, 'mc_dynamics',
     'Reproduce the historical P20 L2 stall with current P23 in rebuilt official MC/MuJoCo. Read docs/nightly/CODEX_STALL_AB.md and tracked stall evidence; '
     'preserve failed results. Separate MC dynamics from localization/costmap experiments. Record raw world/contact/frame evidence, not just map clearance. No safety relaxation.'),
    ('lab_p1_frame_runtime_v1', 21, 'tf',
     'Qualify the already integrated frame-anchor fix in rebuilt official MuJoCo runtime, using raw HAL/world/contact and plant/world registration evidence. Offline pytest alone is insufficient. '
     'Record frame consistency across starts and independent evidence; do not change localization, dynamics and costmap together.'),
    ('lab_p2_odom_reset_v1', 40, 'tf', 'Qualify odometry reset in simulation with explicit reset evidence; preserve pose continuity and fail closed on unexplained reset. No hardware actions.'),
    ('lab_p2_t1_freshness_v1', 41, 'safety', 'Qualify T1 freshness in simulation with stale-input injection. Preserve thresholds and guards; retain failure evidence.'),
    ('lab_p2_collision_doorway_v1', 42, 'collision', 'Qualify collision/doorway behavior with real footprint and unknown treated as obstacle. Preserve collision gates and tolerances. Physical doorway widths remain unvalidated.'),
)


def call(*args):
    return subprocess.call(list(args), cwd=ROOT)


def prepare_campaign(db):
    """Idempotent names; keep legacy failures and never silently replay them."""
    for name, why in LEGACY.items():
        key = 'bootstrap_review:' + name
        if nq.kv_get(db, key) is None:
            row = db.execute('SELECT state,reason FROM tasks WHERE name=?', (name,)).fetchone()
            nq.kv_set(db, key, dict(previous=dict(row) if row else None, classification=why))
            nq.ev(db, name, 'BOOTSTRAP_CLASSIFIED', why)
    # Only the explicitly reviewed snapshot can migrate the legacy pointer. A future robot tip
    # must go through normal robot_sync; do not mark arbitrary new robot findings processed.
    tip = lab.gout('rev-parse', 'origin/robot/stable')
    if (tip == REVIEWED_ROBOT and not nq.kv_get(db, 'last_robot_processed_sha')
            and preflight.integration_baseline_matches()):
        nq.kv_set(db, 'bootstrap_legacy_robot_pointer', nq.kv_get(db, 'last_robot_integrated_sha'))
        nq.kv_set(db, 'last_robot_processed_sha', tip)
        nq.kv_set(db, 'last_robot_integrated_sha', tip)
        nq.ev(db, '*', 'BOOTSTRAP_ROBOT_REVIEW', 'Reviewed snapshot files verified on integration; explicit campaign replaces duplicate historical parity/blocker tasks: ' + tip)
    for name, priority, kind, prompt in CAMPAIGN:
        made = nq.enqueue(db, name, 'true', kind='engineering', priority=priority, resource='sim', timeout_s=3600, heartbeat_s=900,
                          repro='--agents=claude,codex ' + prompt + ' Work only in your isolated attempt. Save reproducible commands, hashed evidence and an engineering result using the provided result contract. '
                          'The orchestrator independently reruns tests and gates the merged tree. Never claim hardware qualification or deploy.')
        if made:
            db.execute('UPDATE tasks SET task_type=?, complexity=?, risk=?, requires_result=1 WHERE name=?', (kind, 'high', 'high', name))
    for name, priority, kind, _prompt in CAMPAIGN[:3]:
        review = name + '_review'
        if nq.enqueue(db, review, 'true', kind='engineering', priority=priority + 1, resource='sim', deps=[name],
                      repro='--agents=codex,claude Independently review ' + name + '. Read the stored result and tracked evidence, rerun the reproduction/regression on its verified SHA; '
                      'check actual AMCL versus truth, map/frame registration, safety gates and unsupported conclusions. Save a hashed review with verifiable tests. '
                      'Do not claim hardware qualification. Report missing inputs as UNVERIFIED.'):
            db.execute('UPDATE tasks SET task_type=?, complexity=?, risk=?, requires_result=1, base_from=? WHERE name=?',
                       (kind, 'high', 'high', name, review))
    nq.checkpoint(db, 'labctl campaign prepared (idempotent)')


def doctor():
    # Uses disposable preflight project only. No agents or daemon are started here.
    return call(sys.executable, str(ROOT / 'tools/nightly/preflight.py'), 'all', '--tests')


def runtime_qualification_errors(db):
    contract = preflight.env_file(ROOT / 'digital_twin/scripts/contract/robot.env')
    def image(name):
        rc, out = lab.sh(['docker','image','inspect',name,'--format','{{.Id}}'])
        return out.strip() if rc == 0 else None
    return qualification.check(db, RUN / 'nightly_qualification.json', ROOT,
                               nq.mergeq.integration_head(ROOT), lab.gout('rev-parse','origin/robot/stable'),
                               contract['ROBOT_MAP_SHA256'], image('x2-twin-stack:nightly'), image('x2-twin-sim:nightly'))


def start():
    if lab.sh('systemctl --user is-active qazbot-nightly.service')[1].strip() == 'active':
        print('Daemon already active; no duplicate orchestrator or campaign launched. Use status.')
        return 0
    # Sync the control checkout before the image is built. Integration itself must never be checked out.
    if nq.mergeq.checked_out_at(ROOT, 'refs/heads/' + nq.mergeq.branch_name()):
        print('Refused: lab/stable is checked out. Use the separate lab/control checkout.', file=sys.stderr)
        return 1
    log = []
    err = lab.sync_control(log)
    print('\n'.join(log), flush=True)
    if err:
        print(err, file=sys.stderr)
        return 1
    rc = doctor()
    if rc:
        print('Preflight failed; no agents, daemon or timer started. See run/nightly/preflight/latest.md.', flush=True)
        return rc
    # Prevent a ref move during preflight from certifying an image built from old code.
    report = json.loads((RUN / 'preflight/latest.json').read_text())
    if (report['git_head'] != lab.gout('rev-parse', 'HEAD')
            or report['integration'] != nq.mergeq.integration_head(ROOT)):
        print('Git changed during preflight; start refused. Run start again.', file=sys.stderr)
        return 1
    db = nq.connect()
    errors = runtime_qualification_errors(db)
    if errors:
        db.close()
        print('Autonomous start refused: official L3/runtime E2E is unqualified: ' + ', '.join(errors), file=sys.stderr)
        return 1
    db.execute('BEGIN IMMEDIATE')
    try:
        prepare_campaign(db)
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()
    return call(sys.executable, str(ROOT / 'tools/nightly/lab.py'), 'resume', '--no-fetch')


def status():
    call(sys.executable, str(ROOT / 'tools/nightly/lab.py'), 'status')
    call(sys.executable, str(ROOT / 'tools/nightly/nq.py'), 'status')
    call(sys.executable, str(ROOT / 'tools/nightly/nq.py'), 'results')
    rc, image = lab.sh(['docker', 'image', 'inspect', 'x2-twin-stack:nightly', '--format', '{{.Id}} {{.Created}}'])
    print('Docker image:', image)
    print('Lab map/parameters:', preflight.env_file(ROOT / 'digital_twin/.env.lab'))
    p = RUN / 'preflight/latest.json'
    if p.exists():
        d = json.loads(p.read_text())
        print('Last observed parity (historical, not a live probe):', d['stamp'], d['verdict'], 'SHA', d['git_head'])
        for r in d['checks']:
            if r['name'].startswith(('live.', 'tf.', 'sim.map', 'isolation.')):
                print(r['status'], r['name'], r['detail'])
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command', choices=('doctor', 'start', 'stop', 'status'))
    a = ap.parse_args()
    RUN.mkdir(parents=True, exist_ok=True)
    # Serializes terminal lifecycle commands only; nq.py remains the sole orchestrator.
    with (RUN / 'labctl.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Another labctl command is running.', file=sys.stderr)
            return 2
        if a.command == 'stop':
            return call(sys.executable, str(ROOT / 'tools/nightly/lab.py'), 'stop', '--no-notify')
        return {'doctor': doctor, 'start': start, 'status': status}[a.command]()


if __name__ == '__main__':
    sys.exit(main())
