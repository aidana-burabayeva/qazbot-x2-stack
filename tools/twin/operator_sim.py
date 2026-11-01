#!/usr/bin/env python3
"""Operator-driven digital twin session: RViz 2D Pose Estimate -> READY -> 2D Goal Pose -> motion of the X2 in official MC + MuJoCo.

    tools/twin/operator_sim.sh start [--start-pose X,Y,YAW] [--mujoco-gui] [--no-rviz] [--hours H]
    tools/twin/operator_sim.sh status | cancel | resume
    tools/twin/operator_sim.sh stop

Safety model (see docs/OPERATOR_SIM.md):
  * simulation only. No SSH, no physical-robot address, no host-network container: the compose network is `internal`.
  * a separate, explicitly operator-confirmed capsule (mode OPERATOR_SIM, scope interactive_operator_mc, qualifies=false, <= 8 h).
    It never touches the Nightly queue authorization (queue.db is not opened for writing except the evidence hand-off through the
    existing workflow.feedback entry), never yields SIM_VERIFIED and never opens L3/FULL_NIGHTLY.
  * separate compose project (x2-operator, private internal network, ROS domain 87 as the twin requires), evidence directory and GUI containers; the official MC is arbitrated with
    Nightly L3 by the same official-mc.lock inside the sim container.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
TW = ROOT / 'digital_twin'
NIGHTLY_ROOT = Path(os.environ.get('OPERATOR_NIGHTLY_ROOT', ROOT))
RUN = Path(os.environ.get('OPERATOR_RUN', NIGHTLY_ROOT / 'run/operator'))
LOCKS = Path(os.environ.get('OPERATOR_LOCK_DIR', NIGHTLY_ROOT / 'run/nightly/l3-locks'))
PROJECT = 'x2-operator'
MAP = 'd1_qazbot_20261029'
PHRASE = 'I AUTHORIZE ONE OPERATOR SIMULATION SESSION'
STATE = RUN / 'state.json'
MIN_MEM_GIB, MAX_HOURS = 24, 8
NIGHTLY_DISK_FLOOR_GIB, EVIDENCE_GIB, VOLUME_SEED_GIB = 5.0, 1.0, 1.5   # Nightly's own floor must survive an operator session
SIM_TAG = os.environ.get('OPERATOR_SIM_TAG', 'operator')
STACK_TAG = os.environ.get('OPERATOR_STACK_TAG', 'nightly')
LIMITS = dict(SIM_CPUS='4', SIM_MEM='8g', STACK_CPUS='4', STACK_MEM='6g')


def env_file(path):
    out = {}
    for ln in Path(path).read_text().splitlines():
        m = re.match(r'^([A-Z0-9_]+)=(.*)$', ln.strip())
        if m:
            out[m.group(1)] = m.group(2)
    return out


def run(cmd, timeout=120, env=None, cwd=None, check=False):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd, env=env)
    if check and r.returncode:
        raise RuntimeError(f'{cmd[:3]} failed: {(r.stdout + r.stderr)[-400:]}')
    return r.returncode, (r.stdout + r.stderr).strip()


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def tree_sha(root):
    h = hashlib.sha256()
    for p in sorted(Path(root).rglob('*')):
        if p.is_file() and not p.is_symlink():
            h.update(str(p.relative_to(root)).encode() + b'\0' + sha256_file(p).encode() + b'\n')
    return h.hexdigest()


# ------------------------------------------------------------------------------------------------ preflight
def parse_pose(text):
    try:
        x, y, yaw = (float(v) for v in text.split(','))
    except ValueError:
        raise SystemExit('--start-pose must be X,Y,YAW (metres, metres, radians)')
    if not all(abs(v) < 1e3 for v in (x, y, yaw)):
        raise SystemExit('--start-pose out of range')
    return x, y, yaw


def check_resources(meminfo='/proc/meminfo', disk_path=None, need_mem=MIN_MEM_GIB, need_disk=None):
    """Pure helper so tests can drive it. Returns [(name, status, detail)]."""
    out = []
    mem = {k: int(v.split()[0]) for k, v in (ln.split(':', 1) for ln in Path(meminfo).read_text().splitlines())}
    avail = mem['MemAvailable'] / 2**20
    out.append(('resources.ram', 'PASS' if avail >= need_mem else 'FAIL', f'{avail:.1f} GiB available, need {need_mem} GiB (containers capped at 20 GiB)'))
    if need_disk is None:
        seeded = run(['docker', 'volume', 'inspect', f'{PROJECT}_mc_runtime'])[0] == 0
        need_disk = NIGHTLY_DISK_FLOOR_GIB + EVIDENCE_GIB + (0 if seeded else VOLUME_SEED_GIB)
    base = Path(disk_path) if disk_path else RUN.parent
    disk = shutil.disk_usage(base if base.exists() else Path('/')).free / 2**30
    out.append(('resources.disk', 'PASS' if disk >= need_disk else 'FAIL',
                f'{disk:.1f} GiB free, need {need_disk:.1f} GiB (Nightly floor {NIGHTLY_DISK_FLOOR_GIB:.0f} + evidence {EVIDENCE_GIB:.0f}'
                f'{"" if need_disk < NIGHTLY_DISK_FLOOR_GIB + EVIDENCE_GIB + 0.1 else " + volume seed " + str(VOLUME_SEED_GIB)})'))
    load = os.getloadavg()[0]
    cores = os.cpu_count() or 1
    out.append(('resources.cpu', 'PASS' if load <= cores - 6 else 'FAIL', f'load1 {load:.1f} on {cores} cores; sim+stack+gui need ~10'))
    return out


def mc_lock_free():
    LOCKS.mkdir(parents=True, exist_ok=True)
    fh = (LOCKS / 'official-mc.lock').open('a')
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fh, fcntl.LOCK_UN)
        return True
    except OSError:
        return False
    finally:
        fh.close()


def running_sims():
    rc, text = run(['docker', 'ps', '--format', '{{.Names}}\t{{.Image}}\t{{.Label "com.docker.compose.project"}}'])
    return [ln.split('\t') for ln in text.splitlines() if '\tx2-twin-sim:' in ln] if rc == 0 else None


def preflight(args):
    res = []
    add = lambda n, s, d: res.append((n, s, d))
    rc, text = run(['docker', 'compose', 'version', '--short'])
    add('docker.compose', 'PASS' if rc == 0 else 'FAIL', text[:60])
    for repo, tag in (('x2-twin-sim', SIM_TAG), ('x2-twin-stack', STACK_TAG)):
        rc, out = run(['docker', 'image', 'inspect', f'{repo}:{tag}', '--format', '{{.Id}}'])
        add(f'image.{repo}', 'PASS' if rc == 0 else 'FAIL', out[:80] if rc == 0 else f'{repo}:{tag} missing (make build / operator image build)')
    if not args.no_rviz:
        rc, out = run(['docker', 'image', 'inspect', f'x2-twin-operator-rviz:{STACK_TAG}', '--format', '{{.Id}}'])
        add('image.rviz', 'PASS' if rc == 0 else 'FAIL', out[:80] if rc == 0 else 'x2-twin-operator-rviz missing (operator_sim.sh build-rviz)')
    sims = running_sims()
    add('mc.exclusive', 'PASS' if sims == [] else 'FAIL', 'no official MC container running' if sims == [] else f'official MC already running: {sims}')
    add('mc.lock', 'PASS' if mc_lock_free() else 'FAIL', str(LOCKS / 'official-mc.lock'))
    st = load_state()
    add('session.single', 'PASS' if not st or st.get('status') != 'RUNNING' else 'FAIL', 'no operator session' if not st or st.get('status') != 'RUNNING' else f"session {st.get('session_id')} is running")
    res += check_resources()
    mp = ROOT / 'ws/src/maps' / MAP
    try:
        man = json.loads((mp / 'manifest.json').read_text())
        ok = sha256_file(mp / 'map.pgm') == man['pgm_sha256']
        add('map.d1', 'PASS' if ok else 'FAIL', f"{MAP} pgm sha {man['pgm_sha256'][:12]}")
    except (OSError, KeyError, ValueError) as exc:
        add('map.d1', 'FAIL', str(exc))
    try:
        env = env_file(TW / '.env.lab')
        vendor = Path(env['TWIN_VENDOR'])
        ident = {n: sha256_file(vendor / n) for n in ('mc/bin/aima-mc-app-main', 'sim/bin/aima-sim-app')}
        add('vendor.mc_sim', 'PASS', 'official MC/MuJoCo binaries present ' + ','.join(v[:8] for v in ident.values()))
    except (OSError, KeyError) as exc:
        add('vendor.mc_sim', 'FAIL', str(exc))
    disp = os.environ.get('DISPLAY', '')
    sock = Path('/tmp/.X11-unix') / ('X' + disp.split(':')[-1].split('.')[0]) if disp else None
    gui = not args.no_rviz or args.mujoco_gui
    add('gui.display', 'PASS' if (not gui or (sock and sock.exists())) else 'FAIL', f'DISPLAY={disp or "unset"}')
    rc, out = run(['git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no'])
    add('git.tree', 'PASS' if not out else 'WARN', 'clean' if not out else f'{len(out.splitlines())} tracked file(s) modified (recorded in the session manifest)')
    return res


def print_table(res):
    for n, s, d in res:
        print(f'  [{s:^6}] {n}: {d}')
    return not any(s == 'FAIL' for _, s, _ in res)


# ------------------------------------------------------------------------------------------------ state
def load_state():
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return None


def save_state(d):
    RUN.mkdir(parents=True, exist_ok=True)
    tmp = STATE.with_suffix('.tmp')
    tmp.write_text(json.dumps(d, indent=2))
    os.replace(tmp, STATE)


# ------------------------------------------------------------------------------------------------ capsule / snapshot
def make_capsule(session_id, source_sha, hours, now=None):
    now = time.time() if now is None else now
    if not 0 < hours <= MAX_HOURS:
        raise SystemExit(f'--hours must be in (0, {MAX_HOURS}]')
    return dict(mode='OPERATOR_SIM', scope='interactive_operator_mc', session_id=session_id, source_sha=source_sha, issued_by='operator_sim',
                qualifies=False, issued=now, expires=now + hours * 3600, l3_workers=1, map=MAP, mc='official')


def authorize(args):
    if args.authorize == PHRASE:
        return True
    if args.authorize:
        raise SystemExit('wrong authorization phrase')
    if sys.stdin.isatty():
        print(f'Official MC + MuJoCo will start for an interactive SIMULATION session (never a physical robot).\nType exactly: {PHRASE}')
        return input('> ').strip() == PHRASE
    raise SystemExit('non-interactive start requires --authorize "' + PHRASE + '" given by the operator')


def make_snapshot(sdir, session_id, start_pose, extra):
    """Immutable manifest of everything that defines the experiment; operator scripts are copied read-only and mounted from the copy."""
    sdir.mkdir(parents=True)
    ops = sdir / 'operator'
    shutil.copytree(ROOT / 'tools/twin/operator', ops)
    env = env_file(TW / '.env.lab')
    rc, sha = run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'])
    _, dirty = run(['git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no'])
    images = {}
    for repo, tag in (('x2-twin-sim', SIM_TAG), ('x2-twin-stack', STACK_TAG), ('x2-twin-operator-rviz', STACK_TAG)):
        images[repo] = run(['docker', 'image', 'inspect', f'{repo}:{tag}', '--format', '{{.Id}}'])[1]
    _, baked = run(['docker', 'run', '--rm', '--entrypoint', 'bash', f'x2-twin-stack:{STACK_TAG}', '-lc',
                    'cd /opt/x2twin/stack && find navigation lio maps -type f | sort | xargs sha256sum | sha256sum'], timeout=120)
    mp = ROOT / 'ws/src/maps' / MAP
    vendor = Path(env['TWIN_VENDOR'])
    manifest = dict(
        session_id=session_id, created=time.time(), git_sha=sha, git_dirty=dirty.splitlines(), start_pose=start_pose,
        campaign_params={f: sha256_file(TW / f) for f in ('.env.lab', 'ws/src/x2_twin_bringup/config/twin.yaml', 'docker/compose.yaml',
                                                           'docker/compose.operator.yaml', 'docker/sim/check_authorization.py')
                         if (TW / f).exists()},
        map=dict(name=MAP, pgm_sha256=sha256_file(mp / 'map.pgm'), yaml_sha256=sha256_file(mp / 'map.yaml')),
        images=images, stack_source_in_image_sha256=baked.split()[0] if baked else None,
        vendor={n: sha256_file(vendor / n) for n in ('mc/bin/aima-mc-app-main', 'sim/bin/aima-sim-app')},
        operator_scripts_tree_sha256=tree_sha(ops), ros_domain_id=87, compose_project=PROJECT, limits=LIMITS, **extra)
    (sdir / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True))
    for p in list(sdir.rglob('*')):
        if p.is_file():
            p.chmod(0o444)
    for p in sorted(sdir.rglob('*'), reverse=True):
        if p.is_dir():
            p.chmod(0o555)
    sdir.chmod(0o555)
    return manifest


def compose_cmd(profile_gui):
    cmd = ['docker', 'compose', '-p', PROJECT, '-f', str(TW / 'docker/compose.yaml'), '-f', str(TW / 'docker/compose.operator.yaml'),
           '--env-file', str(TW / '.env.lab')]
    return cmd + (['--profile', 'gui'] if profile_gui else [])


def compose_env(state):
    base = env_file(TW / '.env.lab')
    return {**os.environ, **base, **LIMITS, 'TWIN_VERSION': STACK_TAG, 'TWIN_MAP': MAP, 'TWIN_MAP_DIR': str(ROOT / 'ws/src/maps' / MAP),
            'TWIN_BACKEND': str(ROOT), 'TWIN_LOCALIZATION': 'amcl', 'TWIN_MC': 'official', 'TWIN_START': state['start_pose_text'],
            'TWIN_GUI': '1' if state['mujoco_gui'] else '0', 'NIGHTLY_WORKER_AUTH': state['capsule'], 'NIGHTLY_LOCK_DIR': str(LOCKS),
            'OPERATOR_SNAPSHOT_DIR': state['snapshot'], 'OPERATOR_EVIDENCE_DIR': state['evidence'], 'OPERATOR_DISPLAY': os.environ.get('DISPLAY', ''),
            'OPERATOR_X11_DIR': '/tmp/.X11-unix' if state['mujoco_gui'] or state['rviz'] else state['empty_x11'],
            'OPERATOR_SIM_TAG': SIM_TAG, 'OPERATOR_STACK_TAG': STACK_TAG}


def dc(state, *args, timeout=300):
    return run(compose_cmd(state['rviz']) + list(args), timeout=timeout, env=compose_env(state), cwd=TW)


# ------------------------------------------------------------------------------------------------ commands
def cmd_start(args):
    print('== preflight')
    res = preflight(args)
    if not print_table(res):
        print('REFUSED: preflight failed; nothing was started.')
        return 2
    if not authorize(args):
        print('REFUSED: not authorized.')
        return 2
    x, y, yaw = parse_pose(args.start_pose)
    session_id = 'op_' + time.strftime('%Y%m%dT%H%M%S')
    sdir = RUN / session_id
    sdir.mkdir(parents=True)
    rc, source_sha = run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'])
    capsule = sdir / 'worker.json'
    capsule.write_text(json.dumps(make_capsule(session_id, source_sha, args.hours)))
    evidence, empty = sdir / 'evidence', sdir / 'no-x11'
    evidence.mkdir()
    empty.mkdir()
    evidence.chmod(0o777)
    manifest = make_snapshot(sdir / 'snapshot', session_id, [x, y, yaw], dict(rviz=not args.no_rviz, mujoco_gui=args.mujoco_gui))
    state = dict(session_id=session_id, status='STARTING', started=time.time(), capsule=str(capsule), snapshot=str(sdir / 'snapshot'),
                 evidence=str(evidence), empty_x11=str(empty), start_pose=[x, y, yaw], start_pose_text=f'{x},{y},{yaw}',
                 mujoco_gui=args.mujoco_gui, rviz=not args.no_rviz, expires=json.loads(capsule.read_text())['expires'])
    save_state(state)
    print(f'== session {session_id}  source {source_sha[:12]}  images sim {manifest["images"]["x2-twin-sim"][7:19]} stack {manifest["images"]["x2-twin-stack"][7:19]}')
    rc, out = dc(state, 'up', '-d', '--no-build', '--wait', '--wait-timeout', '300', 'sim', 'stack', timeout=420)
    (evidence / 'startup.log').write_text(out)
    if rc:
        print('FAILED: official MC/stack startup:\n' + out[-1500:])
        state['status'] = 'FAILED_START'
        save_state(state)
        failure_logs(state)
        cleanup(state)
        return 1
    rc, out = dc(state, 'up', '-d', '--no-build', 'panel', 'recorder', *(['rviz'] if state['rviz'] else []))
    (evidence / 'startup_gui.log').write_text(out)
    if rc:
        print('FAILED: operator containers:\n' + out[-1500:])
        state['status'] = 'FAILED_START'
        save_state(state)
        failure_logs(state)
        cleanup(state)
        return 1
    state['status'] = 'RUNNING'
    save_state(state)
    print(INSTRUCTIONS.format(sid=session_id, evidence=evidence, hours=args.hours))
    return 0


INSTRUCTIONS = """
OPERATOR SESSION {sid} IS RUNNING  (simulation only; official MC + MuJoCo; no physical robot)
  1. RViz: tool "2D Pose Estimate" -> click+drag the robot's pose on the D1 map. Nothing is seeded for you.
  2. Wait until the panel (top-left text in RViz) shows  TRUST: READY  (a seed that does not fit the scan is flagged there).
  3. RViz: tool "2D Goal Pose" (-> /qazbot_d1_map/goal_pose, handled by the Goal Coordinator exactly as on the X2) -> click a goal.
  4. Watch the X2 move (RViz, and MuJoCo if --mujoco-gui). Cancel: operator_sim.sh cancel (then `resume` before the next goal).   Stop the session: operator_sim.sh stop
  Evidence (live): {evidence}   Session limit: {hours} h.
"""


def failure_logs(state):
    """A failed start must leave its evidence: container logs and health state are saved before the project is torn down."""
    ev = Path(state['evidence'])
    ev.joinpath('failed_start_logs.txt').write_text(dc(state, 'logs', '--no-color', timeout=90)[1])
    ev.joinpath('failed_start_ps.txt').write_text(dc(state, 'ps', '-a', timeout=30)[1])
    for svc in ('sim', 'stack'):
        rc, text = run(['docker', 'inspect', f'{PROJECT}-{svc}-1', '--format', '{{json .State.Health}}'])
        ev.joinpath(f'failed_start_health_{svc}.json').write_text(text)


def cleanup(state):
    dc(state, 'down', '--remove-orphans', timeout=120)
    Path(state['capsule']).exists() and os.chmod(state['capsule'], 0o644)
    Path(state['capsule']).unlink(missing_ok=True)   # revokes the capsule; evidence/snapshot/volumes are kept


def sh_exec(state, service, script, timeout=60):
    return dc(state, 'exec', '-T', service, 'bash', '-lc', 'source /opt/ros/humble/setup.bash; source /opt/x2twin/overlay/setup.bash; ' + script, timeout=timeout)


def cmd_cancel(args):
    st = load_state()
    if not st or st.get('status') != 'RUNNING':
        print('no running operator session')
        return 1
    rc, out = sh_exec(st, 'panel', 'ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger', timeout=30)
    print(out[-400:])
    return rc


def cmd_resume(args):
    """After `cancel`/a safety stop the Goal Coordinator stays latched until the operator clears it (new goals are rejected meanwhile)."""
    st = load_state()
    if not st or st.get('status') != 'RUNNING':
        print('no running operator session')
        return 1
    rc, out = sh_exec(st, 'panel', 'ros2 service call /qazbot_d1_map/goal_coordinator/resume std_srvs/srv/Trigger', timeout=30)
    print(out[-400:])
    return rc


def cmd_status(args):
    st = load_state()
    if not st:
        print('no operator session recorded')
        return 1
    print(f"session {st['session_id']}  status {st['status']}  expires in {int(st.get('expires', 0) - time.time())} s")
    if st['status'] != 'RUNNING':
        return 0
    _, ps = dc(st, 'ps', '--format', '{{.Service}} {{.State}} {{.Health}}', timeout=30)
    print(ps)
    live = Path(st['evidence']) / 'live_status.json'
    if live.exists():
        d = json.loads(live.read_text())
        print(json.dumps({k: d.get(k) for k in ('trust', 'seed_diag', 'goal', 'adapter')}, indent=1, default=str)[:1500])
    return 0


def cmd_stop(args):
    st = load_state()
    if not st or st.get('status') not in ('RUNNING', 'FAILED_START', 'STARTING'):
        print('no running operator session')
        return 1
    ev = Path(st['evidence'])
    print('== cancel any active goal')
    dc(st, 'exec', '-T', 'panel', 'bash', '-lc', 'source /opt/ros/humble/setup.bash; source /opt/x2twin/overlay/setup.bash; '
       'timeout 15 ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger', timeout=30)
    print('== stop recorder/panel (bag closes cleanly), collect MC/sim logs')
    dc(st, 'stop', '-t', '40', 'recorder', 'panel', 'rviz', timeout=120)
    for name in ('mc.log', 'sim.log', 'world.json'):
        rc, text = dc(st, 'exec', '-T', 'sim', 'cat', '/var/log/twin/' + name, timeout=30)
        (ev / name).write_text(text)
    rc, text = dc(st, 'logs', '--no-color', timeout=60)
    (ev / 'containers.log').write_text(text)
    rc, text = run(['docker', 'network', 'inspect', f'{PROJECT}_twin', '--format', '{{json .}}'])
    (ev / 'network.json').write_text(text)
    print('== teardown (project x2-operator only)')
    cleanup(st)
    st['status'] = 'STOPPED'
    st['stopped'] = time.time()
    save_state(st)
    report = package_evidence(st)
    print(json.dumps(report, indent=1)[:1800])
    if not args.no_handoff:
        handoff(st, report)
    return 0


def package_evidence(st):
    """Analyse (tools/twin/operator/analyze_session.py, pure python), then write a sha256 manifest and make the evidence read-only."""
    ev = Path(st['evidence'])
    sys.path.insert(0, str(ROOT / 'tools/twin/operator'))
    import analyze_session
    report = analyze_session.analyze(ev, ROOT / 'ws/src/maps' / MAP / 'map.yaml')
    report.update(session_id=st['session_id'], scope='OPERATOR_INTERACTIVE', verification='UNVERIFIED_OPERATOR_SESSION',
                  qualifies=False, source_manifest=str(Path(st['snapshot']) / 'manifest.json'))
    (ev / 'analysis.json').write_text(json.dumps(report, indent=2, default=str))
    files = {str(p.relative_to(ev)): sha256_file(p) for p in sorted(ev.rglob('*')) if p.is_file() and p.name != 'SHA256SUMS.json'}
    (ev / 'SHA256SUMS.json').write_text(json.dumps(files, indent=1, sort_keys=True))
    for p in ev.rglob('*'):
        if p.is_file():
            p.chmod(0o444)
    report['evidence'] = str(ev)
    report['files'] = len(files)
    return report


def handoff(st, report):
    """Existing Nightly workflow only: workflow.py feedback -> analyze/ingest (clusters, dedups, creates producer/review tasks)."""
    fb = dict(report.get('nightly_feedback') or {})
    if not fb:
        print('hand-off: nothing to report (no defect signature)')
        return
    fb.update(experiment_id=st['session_id'], evidence=str(Path(st['evidence']) / 'analysis.json'), fidelity='OPERATOR_INTERACTIVE',
              hypothesis_status='HYPOTHESIS')
    path = Path(st['evidence']).parent / 'nightly_feedback.json'
    path.write_text(json.dumps(fb, indent=1, default=str))
    rc, out = run([sys.executable, str(NIGHTLY_ROOT / 'tools/nightly/workflow.py'), 'feedback', '--file', str(path)], cwd=NIGHTLY_ROOT,
                  env={**os.environ, 'QAZBOT_ROLE': 'LAB'}, timeout=120)
    print('hand-off to Nightly workflow:', 'ok' if rc == 0 else f'FAILED rc={rc}', out[-300:])


def cmd_build_rviz(args):
    env = {**os.environ, 'TWIN_VERSION': STACK_TAG, 'TWIN_BACKEND': str(ROOT), 'OPERATOR_SNAPSHOT_DIR': '/nonexistent',
           'OPERATOR_EVIDENCE_DIR': '/nonexistent', 'TWIN_MAP': MAP, 'TWIN_MAP_DIR': str(ROOT / 'ws/src/maps' / MAP), 'OPERATOR_X11_DIR': '/tmp'}
    rc, out = run(compose_cmd(True) + ['build', 'rviz'], env=env, cwd=TW, timeout=1800)
    print(out[-800:])
    return rc


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('start')
    s.add_argument('--start-pose', default='-8.344,-1.213,-2.145', help='X,Y,YAW where the robot stands in the MuJoCo world (map frame)')
    s.add_argument('--mujoco-gui', action='store_true', help='show the MuJoCo window (software GL; check status for missed MC wakeups)')
    s.add_argument('--no-rviz', action='store_true')
    s.add_argument('--hours', type=float, default=4.0)
    s.add_argument('--authorize', default='', help='exact operator phrase for non-interactive start')
    s.set_defaults(fn=cmd_start)
    for name, fn in (('status', cmd_status), ('cancel', cmd_cancel), ('resume', cmd_resume), ('build-rviz', cmd_build_rviz)):
        sub.add_parser(name).set_defaults(fn=fn)
    t = sub.add_parser('stop')
    t.add_argument('--no-handoff', action='store_true')
    t.set_defaults(fn=cmd_stop)
    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
