#!/usr/bin/env python3
"""QazBot lab preflight (SIM ONLY, stdlib + docker CLI). Nothing here touches the physical X2: no ssh, no robot DDS, no MC.

  preflight.py static            git / contract / map / agents / docker / systemd / ssh-scan   (seconds, no containers)
  preflight.py sim               + rebuild stack image, 73 SHA256 inside it, map, P23, live AMCL/Trust params, TF owners, isolation
  preflight.py all [--tests]     static + sim (+ regression tests)

Exit 0 = every check PASS (WARN allowed), 1 = at least one FAIL. A JSON + Markdown report is always written to run/nightly/preflight/.
Status vocabulary: PASS | FAIL | WARN | UNVERIFIED (never promoted to PASS without evidence).
"""
import argparse
import importlib.util
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nq  # noqa: E402
import mergeq  # noqa: E402
import execution  # noqa: E402
import source_state  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'operator_skills'))
import opskills  # noqa: E402

ROOT, RUN = nq.ROOT, nq.RUN
TW = ROOT / 'digital_twin'
ROBOT_REF = os.environ.get('ROBOT_REF', 'origin/robot/stable')
SNAP = 'docs/evidence/x2_twin_20261030/snapshot/actual_parameters.json'
PROJECT = 'x2tp-pre'            # throw-away compose project of the preflight (never the operator's `x2-twin`)
ENVFILE = '.env.lab'
KEYS = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value', 5: 'byte_array_value', 6: 'bool_array_value',
        7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}
# parameters whose divergence is a known, documented simulation difference (never silently ignored: reported as `allowed`)
LIVE_NODES = {'/qazbot_d1_map/amcl': None, '/qazbot_d1_map/localization_trust': None}
# Known, documented sim-vs-robot differences. Anything else that differs is a FAIL. `stricter` = the sim value must be <= the robot's (the gate is never weaker in the twin).
ALLOWED = {
    '/qazbot_d1_map/localization_trust': {
        'map_yaml': ('path', 'robot path /home/agi/..., twin mounts the same map at /opt/x2twin/stack/maps/ (sha256 verified separately)'),
        'odom_source': ('exact', 'twin plant publishes odom->base_footprint (kinematic_plant); the robot runs native_odom_bridge (native_imu). Plant is the sim substitute, not a code difference'),
        'odom_topic': ('exact', 'consequence of odom_source: /qazbot_lio/odom_level in the twin vs /qazbot_native/odom on the robot'),
        'jump_xy_max': ('stricter', 'robot live T5 = 1.0 m (operator CLI -p jump_xy_max:=1.0, nav_session_20261029/start.sh); code default and twin = 0.10. Twin is NOT relaxed'),
        'jump_yaw_max': ('stricter', 'robot live T5 = 0.5 rad (operator CLI); code default and twin = 0.10. Twin is NOT relaxed'),
    }}

RES = []
START_HEAD = START_INTEGRATION = None
MODE = execution.FULL


def add(name, status, detail='', **extra):
    RES.append(dict(name=name, status=status, detail=str(detail)[:600], **extra))
    print(f'  [{status:^10}] {name}: {str(detail)[:200]}', flush=True)


def sh(cmd, timeout=120, cwd=None, env=None):
    r = subprocess.run(cmd if isinstance(cmd, list) else ['bash', '-lc', cmd], capture_output=True, text=True, timeout=timeout,
                       cwd=cwd or ROOT, env={**os.environ, **(env or {})})
    return r.returncode, (r.stdout + r.stderr).strip()


def git(*a, timeout=60):
    return sh(['git', '-C', str(ROOT), *a], timeout)


def gout(*a):
    rc, o = git(*a)
    return o if rc == 0 else ''


def dc(*a, timeout=900, env=None):
    """docker compose against the lab env file, in its own throw-away project."""
    return sh(['docker', 'compose', '-p', PROJECT, '-f', 'docker/compose.yaml', '--env-file', ENVFILE, *a], timeout=timeout, cwd=TW,
              env={**env_file(TW / ENVFILE), 'TWIN_BACKEND': str(ROOT), 'TWIN_MAP_DIR': str(ROOT / 'ws/src/maps' / env_file(TW / ENVFILE)['TWIN_MAP']), 'TWIN_LOCALIZATION': 'amcl', 'TWIN_VERSION': 'nightly', **(env or {})})


def robot_params():
    if not source_state.hardware_ref(ROBOT_REF):
        raise RuntimeError('ROBOT_REF must identify installed hardware, never preservation/candidate evidence')
    rc, o = git('show', f'{ROBOT_REF}:{SNAP}')
    if rc != 0:
        raise RuntimeError(f'{ROBOT_REF}:{SNAP} unavailable: {o[-100:]}')
    return json.loads(o)['nodes']


def pval(p):
    return p.get(KEYS.get(p.get('type'), ''), None)


def env_file(path):
    d = {}
    for ln in Path(path).read_text().splitlines():
        m = re.match(r'^([A-Z0-9_]+)=(.*)$', ln.strip())
        if m:
            d[m.group(1)] = m.group(2)
    return d


# ------------------------------------------------------------------ static

def check_git():
    br = gout('rev-parse', '--abbrev-ref', 'HEAD')
    if not br.startswith('lab/') or br == mergeq.branch_name():
        add('git.control_branch', 'FAIL', f'checkout is on {br!r}; it must be a control branch (lab/control), never {mergeq.branch_name()}')
    else:
        add('git.control_branch', 'PASS', br)
    co = mergeq.checked_out_at(ROOT, f'refs/heads/{mergeq.branch_name()}')
    add('git.integration_not_checked_out', 'FAIL' if co else 'PASS',
        f'{mergeq.branch_name()} is checked out in {co}: the merge queue would leave that tree stale' if co else f'{mergeq.branch_name()} is not checked out in any worktree')
    url=gout('config','--get','remote.origin.url')
    if not (url.startswith('https://github.com/') or url.startswith('git@github.com:')):
        add('git.remote_scope','FAIL','origin is not an approved GitHub source; no fetch attempted')
        return
    rc, o = git('fetch', '--prune', 'origin', timeout=120)
    add('git.fetch', 'PASS' if rc == 0 else ('WARN' if MODE==execution.ENGINEERING else 'FAIL'), 'origin fetched' if rc == 0 else f'offline? {o[-100:]}')
    ih = gout('rev-parse', '-q', '--verify', f'refs/heads/{mergeq.branch_name()}')
    if not ih:
        add('git.integration_branch', 'FAIL', f'{mergeq.branch_name()} missing')
    else:
        contains = git('merge-base', '--is-ancestor', ih, 'HEAD')[0] == 0
        add('git.integration_in_control', 'PASS' if contains else 'WARN', f'integration {ih[:9]} ' + ('contained in control HEAD' if contains else 'is ahead of control HEAD: start will merge it (resume sync)'))
        ro = gout('rev-parse', '-q', '--verify', f'refs/remotes/origin/{mergeq.branch_name()}')
        if ro and ro != ih:
            anc = git('merge-base', '--is-ancestor', ro, ih)[0] == 0
            add('git.integration_vs_origin', 'WARN' if anc or MODE==execution.ENGINEERING else 'FAIL', 'local ahead of origin (publishes at stop)' if anc else f'origin {ro[:9]} not an ancestor of local {ih[:9]}: diverged')
        else:
            add('git.integration_vs_origin', 'PASS', f'{ih[:9]} == origin')
    dirty = [ln for ln in gout('status', '--porcelain').splitlines() if ln.strip()]
    add('git.tracked_clean', 'FAIL' if dirty else 'PASS', f'{len(dirty)} tracked/untracked modification(s): {dirty[:3]}' if dirty else 'no tracked or untracked modifications')
    add('git.robot_ref', 'PASS' if source_state.hardware_ref(ROBOT_REF) and gout('rev-parse', '-q', '--verify', ROBOT_REF) else 'FAIL', f'{ROBOT_REF} {gout("rev-parse", "--short", ROBOT_REF)}')


def integration_baseline_matches():
    """Check Git blobs, including maps, on the ref attempts actually start from."""
    import hashlib
    manifest = ROOT / 'docs/evidence/x2_twin_20261030/robot_baseline_manifest.json'
    items = json.loads(manifest.read_text())['files']
    ref = mergeq.integration_head(ROOT)
    for item in items:
        r = subprocess.run(['git', '-C', str(ROOT), 'show', f'{ref}:{item["path"]}'], capture_output=True)
        if r.returncode or hashlib.sha256(r.stdout).hexdigest() != item['sha256']:
            return False
    return len(items) == 73


def check_baseline():
    evidence = source_state.preservation_status(ROOT, mergeq.integration_head(ROOT))
    add('baseline.preservation_import', 'PASS' if evidence['status'] == 'SOURCE_MATCH' else 'WARN', json.dumps(evidence))
    ok = integration_baseline_matches()
    add('baseline.integration_sha256', 'PASS' if ok else 'FAIL', '73/73 integration Git blobs match snapshot' if ok else 'integration branch differs from captured robot files')
    rc, o = sh('digital_twin/scripts/contract/check_published.sh', env=None)
    add('baseline.published_in_sync', 'PASS' if rc == 0 else 'FAIL', o.splitlines()[-1] if o else rc)
    rc, o = sh([sys.executable, 'tools/twin/verify_robot_baseline.py'])
    add('baseline.manifest_sha256', 'PASS' if rc == 0 and o.strip().startswith(tuple('0123456789')) and o.split('/')[0] == o.split()[0].split('/')[0] and '73/73' in o else 'FAIL', o.splitlines()[-1] if o else rc)


def check_contract():
    try:
        nodes = robot_params()
    except RuntimeError as e:
        add('contract.snapshot', 'FAIL', e)
        return None
    c = env_file(TW / 'scripts/contract/robot.env')
    gc = nodes['/qazbot_d1_map/nav2/controller_server']
    xy, yaw = pval(gc['goal_checker.xy_goal_tolerance']), pval(gc['goal_checker.yaw_goal_tolerance'])
    tr = pval(nodes['/qazbot_d1_map/localization_trust']['expected_map_sha256'])
    ok = float(c['ROBOT_XY_TOL']) == xy and float(c['ROBOT_YAW_TOL']) == yaw and c['ROBOT_MAP_SHA256'] == tr
    add('contract.robot_env_vs_snapshot', 'PASS' if ok else 'FAIL', f'robot.env xy={c["ROBOT_XY_TOL"]} yaw={c["ROBOT_YAW_TOL"]} sha={c["ROBOT_MAP_SHA256"][:12]} vs snapshot xy={xy} yaw={yaw} trust={tr[:12]}')
    lab = env_file(TW / ENVFILE)
    ok = lab.get('TWIN_MAP') == c['ROBOT_MAP'] and float(lab['TWIN_XY_TOL']) == xy and float(lab['TWIN_YAW_TOL']) == yaw
    add('contract.lab_env', 'PASS' if ok else 'FAIL', f'{ENVFILE}: map={lab.get("TWIN_MAP")} xy={lab.get("TWIN_XY_TOL")} yaw={lab.get("TWIN_YAW_TOL")}')
    pgm = ROOT / 'ws/src/maps' / c['ROBOT_MAP'] / 'map.pgm'
    import hashlib
    sha = hashlib.sha256(pgm.read_bytes()).hexdigest() if pgm.exists() else None
    add('map.host_sha256', 'PASS' if sha == c['ROBOT_MAP_SHA256'] else 'FAIL', f'{pgm.relative_to(ROOT)} {sha}')
    return nodes


def check_p23():
    f = ROOT / 'ws/src/navigation/x2_motion_adapter.py'
    t = f.read_text() if f.exists() else ''
    ok = 'P23: pause, not reset' in t and 'P23 2026-10-30' in t
    add('motion_adapter.p23_host', 'PASS' if ok else 'FAIL', 'P23 markers present in ws/src/navigation/x2_motion_adapter.py' if ok else 'P23 markers missing')


def check_ssh_scan():
    files = [p for g in ('tools/nightly/*.py', 'tools/nightly/*.sh', 'tools/nightly/systemd/*', 'digital_twin/scripts/nightly/*.sh', 'digital_twin/scripts/nightly/*.py') for p in ROOT.glob(g)]
    pat = re.compile(r'(?<![\w-])(ssh|scp|sftp)\s+(-\w|[\w.-]+@)|check_robot\.sh|BatchMode=yes|192\.168\.1\.72|agi@')
    hits = []
    for f in files:
        if f.name in ('preflight.py',):
            continue
        for i, ln in enumerate(f.read_text(errors='ignore').splitlines(), 1):
            s = ln.strip()
            if pat.search(ln) and not s.startswith('#') and 'refused' not in s and 'operator' not in s.lower() and 'Never' not in s and 'never' not in s:
                hits.append(f'{f.relative_to(ROOT)}:{i}')
    add('isolation.no_ssh_in_lab_chain', 'FAIL' if hits else 'PASS', f'robot/ssh invocation(s): {hits[:6]}' if hits else f'{len(files)} lab-chain files scanned: no ssh/scp/robot address/check_robot.sh')


def check_agents():
    providers = ('codex',) if os.environ.get('NIGHTLY_CAMPAIGN') == execution.X2_CAMPAIGN else ('claude', 'codex')
    for a in providers:
        try:
            v = nq.agents.ADAPTERS[a].version()
        except Exception as e:  # noqa: BLE001
            v = None
            add(f'agents.{a}', 'FAIL', f'adapter error {e}')
            continue
        if not v:
            add(f'agents.{a}', 'FAIL', 'CLI not installed')
            continue
        cmd = 'claude auth status' if a == 'claude' else 'codex login status'
        rc, o = sh(cmd, timeout=30)
        logged = ('"loggedIn": true' in o) if a == 'claude' else ('Logged in' in o)
        add(f'agents.{a}', 'PASS' if rc == 0 and logged else 'FAIL', f'{v} {"authenticated" if logged else "NOT authenticated: " + o[-80:]}')


def check_docker():
    rc, o = sh("docker info --format '{{.ServerVersion}}'", timeout=30)
    add('docker.daemon', 'PASS' if rc == 0 else 'FAIL', o[-80:])
    free = shutil.disk_usage(ROOT).free // 2**30
    add('host.disk_free_gb', 'PASS' if free >= 10 else 'WARN' if free >= 5 else 'FAIL', f'{free} GB free (run_nightly refuses < 5; WARN < 10)')
    ps_rc, o = sh("docker ps --format '{{.Names}}'")
    foreign = [n for n in o.split() if n.startswith('x2-twin') or n.startswith('x2tn') or n.startswith('x2tp')]
    add('docker.no_stray_twin', 'FAIL' if foreign or ps_rc else 'PASS', f'running twin containers: {foreign}' if foreign else 'no twin containers running')


def check_systemd():
    rc, o = sh('systemctl --user show-environment >/dev/null 2>&1 && echo ok')
    add('systemd.user_manager', 'PASS' if o.strip() == 'ok' else 'FAIL', o[-80:] or 'systemctl --user unavailable')
    _rc, o = sh('loginctl show-user $USER -p Linger')
    add('systemd.linger', 'PASS' if 'yes' in o else 'WARN', o or 'unknown')
    for u in ('qazbot-nightly.service', 'qazbot-nightly-enqueue.timer'):
        a = sh(f'systemctl --user is-active {u}')[1].strip()
        add(f'systemd.{u}', 'FAIL' if 'Failed to connect' in a else 'PASS', f'{a} (start enables the daemon first, the timer last)')
    t = (ROOT / 'tools/nightly/systemd/qazbot-nightly-enqueue.timer').read_text()
    add('systemd.timer_no_catchup', 'PASS' if 'Persistent=false' in t else 'FAIL', 'Persistent=false: enabling the timer cannot trigger a missed nightly' if 'Persistent=false' in t else 'Persistent=true can fire a missed nightly on enable')


def check_role():
    r = opskills.detect_role()
    add('role', 'PASS' if r == 'LAB' else 'FAIL', r)


def run_tests():
    rc, o = sh('source /opt/ros/humble/setup.bash 2>/dev/null; python3 -m pytest -q -p no:cacheprovider tests/nightly tests/motion_adapter', timeout=1500)
    add('tests.regression', 'PASS' if rc == 0 else 'FAIL', o.splitlines()[-1] if o else rc)


def vendor_identity():
    import hashlib
    vendor=Path(env_file(TW / '.env.lab')['TWIN_VENDOR'])
    return {name:hashlib.sha256((vendor/name).read_bytes()).hexdigest() for name in
            ('mc/bin/aima-mc-app-main','sim/bin/aima-sim-app','sim/lib/libaima-sim-module-mujoco.so.0.0.0','sim/lib/libmujoco.so.3.3.7')}


def stage_static():
    print('== static')
    check_role(); check_git(); check_baseline(); nodes = check_contract();
    try:
        ids=vendor_identity();add('vendor.identity','PASS','MC/MuJoCo binary hashes measured',hashes=ids)
    except OSError as exc:
        add('vendor.identity','FAIL',str(exc))
    check_p23(); check_ssh_scan(); check_agents(); check_docker(); check_systemd()
    return nodes


def stage_engineering():
    check_role(); check_git(); check_ssh_scan(); check_agents(); check_systemd()
    free=shutil.disk_usage(ROOT).free//2**30
    add('engineering.disk','PASS' if free>=5 else 'FAIL',f'{free} GiB free; existing 5 GiB floor preserved')
    rc,out=sh(['bwrap','--die-with-parent','--unshare-pid','--unshare-ipc','--ro-bind','/','/',
               '--proc','/proc','--dev','/dev','--tmpfs','/run','--tmpfs','/tmp','--ro-bind-try','/run/systemd/resolve','/run/systemd/resolve',
               '--','true'])
    add('engineering.namespace_isolation','PASS' if rc==0 else 'FAIL',out or 'read-only host and hidden runtime sockets')
    rc,out=sh("docker info --format '{{.ServerVersion}}'",timeout=20)
    add('engineering.docker_optional','PASS' if rc==0 else 'WARN',out[-120:]+' (not required for offline Engineering)')
    # Source parity is diagnostic here: verified LAB source fixes may intentionally differ from installed robot.
    rc,out=sh([sys.executable,'tools/twin/verify_robot_baseline.py'])
    add('engineering.robot_parity','PASS' if rc==0 else 'WARN',out[-150:])
    rc,out=sh('source /opt/ros/humble/setup.bash 2>/dev/null; python3 -m pytest -q -p no:cacheprovider tests/nightly/test_execution_modes.py tests/nightly/test_labctl.py tests/motion_adapter',timeout=300)
    add('engineering.basic_tests','PASS' if rc==0 else 'FAIL',out[-220:])


# ------------------------------------------------------------------ sim

def in_stack(script, timeout=90):
    """Run a bash snippet inside the running stack-lite container with ROS sourced."""
    return dc('exec', '-T', 'stack-lite', 'bash', '-c', 'source /opt/ros/${ROS_DISTRO}/setup.bash; source /opt/x2twin/overlay/setup.bash; ' + script, timeout=timeout)


def flat(d, pre=''):
    out = {}
    for k, v in d.items():
        out.update(flat(v, f'{pre}{k}.') if isinstance(v, dict) else {f'{pre}{k}': v})
    return out


def allowed_difference(node, key, robot, sim):
    """Accept only the measured substitutions; unknown values fail closed."""
    if node != '/qazbot_d1_map/localization_trust':
        return False
    if key in ('jump_xy_max', 'jump_yaw_max'):
        return (type(robot) in (int, float) and type(sim) in (int, float)
                and math.isfinite(robot) and math.isfinite(sim) and 0 < sim <= min(robot, 0.10))
    if key == 'odom_source':
        return (robot, sim) == ('native_imu', 'lio')
    if key == 'odom_topic':
        return (robot, sim) == ('/qazbot_native/odom', '/qazbot_lio/odom_level')
    if key == 'map_yaml':
        name = env_file(TW / 'scripts/contract/robot.env')['ROBOT_MAP']
        return (robot == f'/home/agi/control_backend/ws/src/maps/{name}/map.yaml'
                and sim == f'/opt/x2twin/stack/maps/{name}/map.yaml')
    return False


def live_parity(nodes):
    import yaml  # PyYAML ships with the ROS-capable host python; fall back to UNVERIFIED if absent
    skip = lambda k: k in {'use_sim_time', 'start_type_description_service'} or k.startswith('qos_overrides.')
    num = lambda a, b: isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool) and abs(a - b) <= 1e-9 * max(1, abs(a), abs(b))
    for node in LIVE_NODES:
        rc, o = in_stack(f'ros2 param dump {node} --print 2>/dev/null', timeout=60)
        o = '\n'.join(ln for ln in o.splitlines() if not ln.startswith('WARNING'))
        if rc != 0 or not o.strip():
            add(f'live.params.{node}', 'FAIL', f'node absent / dump failed: {o[-100:]}')
            continue
        y = yaml.safe_load(o)
        sim = {k: v for k, v in flat(next(iter(y.values()))['ros__parameters']).items() if not skip(k)}
        rob = {k: pval(v) for k, v in nodes[node].items() if isinstance(v, dict) and 'type' in v and not skip(k)}
        diff = {k: dict(robot=rob[k], sim=sim[k]) for k in rob if k in sim and rob[k] != sim[k] and not num(rob[k], sim[k]) and not (rob[k] == [] and sim[k] in ([], None))}
        allowed, bad = {}, {}
        for k, dv in diff.items():
            rule = ALLOWED.get(node, {}).get(k)
            if rule and allowed_difference(node, k, dv['robot'], dv['sim']):
                allowed[k] = dict(dv, why=rule[1])
            else:
                bad[k] = dv
        missing = sorted(k for k in rob if k not in sim)
        add(f'live.params.{node}', 'PASS' if not bad and not missing else 'FAIL', f'compared {len([k for k in rob if k in sim])} params, unexplained differ={json.dumps(bad, default=str)[:260]}, documented differences={sorted(allowed)}',
            only_robot=missing, only_sim=sorted(k for k in sim if k not in rob)[:20], differ=bad, allowed=allowed)
    # key AMCL knobs explicitly (P23 profile) so a dropped override can never hide behind "only_sim"
    want = {'max_beams': 180, 'alpha1': 0.3, 'alpha2': 0.2, 'alpha3': 0.15, 'alpha4': 0.15, 'alpha5': 0.15}
    got = {}
    for k in want:
        _rc, o = in_stack(f'ros2 param get /qazbot_d1_map/amcl {k}', timeout=30)
        m = re.search(r'(-?\d+(\.\d+)?)\s*$', o)
        got[k] = float(m.group(1)) if m else None
    add('live.amcl_p23_profile', 'PASS' if all(got[k] == want[k] for k in want) else 'FAIL', f'got={got} want={want}')


def ownership_report_ok(doc, odom_owner='/kinematic_plant'):
    required = {'map->odom': '/qazbot_d1_map/amcl', 'odom->base_footprint': odom_owner}
    for edge, owner in required.items():
        rows = doc.get('owners', {}).get(edge, [])
        if len(rows) != 1:
            return False
        row = rows[0]
        gid = row.get('gid', '')
        if (not re.fullmatch('[0-9a-f]{48}', gid) or set(gid) == {'0'} or
                type(row.get('samples')) is not int or row['samples'] < 3 or row.get('nodes') != [owner]):
            return False
    return doc.get('pass') is True


def tf_owners():
    binary = '/opt/x2twin/overlay/lib/qazbot_tf_probe/tf_owner_probe'
    # Actual DDS self-tests use a private topic; a duplicate ROS node name must not hide two GIDs.
    for case, expected_rc in (('single', 0), ('duplicate', 1), ('same-name', 1), ('missing', 1)):
        rc, out = in_stack('QAZBOT_SIM=1 ' + binary + ' ' + case, timeout=20)
        match = re.search(r'TFJSON(.*)', out)
        doc = json.loads(match.group(1)) if match else {}
        wanted = case == 'single'
        good = rc == expected_rc and doc.get('pass') is wanted
        if case in ('duplicate', 'same-name'):
            good = good and len(doc.get('owners', {}).get('map->odom', [])) == 2
        if case == 'missing':
            good = good and all(not v for v in doc.get('owners', {}).values()) and len(doc.get('owners', {})) == 2
        add('tf.dds_fixture.' + case, 'PASS' if good else 'FAIL', f'rc={rc} native Humble MessageInfo; {out[-200:]}')
        if not good:
            add('tf.owners', 'FAIL', 'native DDS diagnostic self-test failed; ownership not qualified')
            return
    start = env_file(TW / ENVFILE)['TWIN_START']
    import shlex
    rc, out = in_stack('export QAZBOT_SIM=1 TWIN_ODOM_OWNER=/kinematic_plant TWIN_START=' + shlex.quote(start) + '; ' + binary, timeout=60)
    match = re.search(r'TFJSON(.*)', out)
    doc = json.loads(match.group(1)) if match else {}
    ok = rc == 0 and ownership_report_ok(doc)
    add('tf.owners', 'PASS' if ok else 'FAIL', 'native per-message DDS GIDs, 28s observation, >=3 samples per edge', observation=doc, error=out[-200:] if not ok else '')


def costmap_sensor_contract():
    script = (ROOT / 'tools/nightly/costmap_sensor_probe.py').read_text()
    rc, output = in_stack("export QAZBOT_SIM=1; python3 - <<'QAZBOT_SENSOR_PROBE'\n" + script + "\nQAZBOT_SENSOR_PROBE", timeout=30)
    match = re.search(r'SENSORJSON(.*)', output)
    doc = json.loads(match.group(1)) if match else {}
    add('sensor.costmap_contract', 'PASS' if rc == 0 and doc.get('passed') is True else 'FAIL',
        doc if doc else output[-200:], observation=doc)


def isolation(domain_expected='87'):
    rc, o = sh(f"docker network inspect {PROJECT}_twin --format '{{{{.Internal}}}}'")
    internal = o.strip() == 'true'
    add('isolation.network_internal', 'PASS' if internal else 'FAIL', f'{PROJECT}_twin Internal={o.strip()}')
    if not internal:
        return False
    rc, o = in_stack('echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID LOCALHOST=${ROS_LOCALHOST_ONLY:-} RMW=$RMW_IMPLEMENTATION; ip -4 route 2>/dev/null || cat /proc/net/route | head -5')
    add('isolation.ros_domain', 'PASS' if f'ROS_DOMAIN_ID={domain_expected}' in o else 'FAIL', o.splitlines()[0] if o else '')
    default = bool(re.search(r'^default ', o, re.M)) or bool(re.search(r'^\S+\s+00000000\s+[0-9A-F]{8}\s+\d+\s+\d+\s+\d+\s+0\s+00000000', o, re.M))
    add('isolation.no_default_route', 'FAIL' if default else 'PASS', 'container has a default route' if default else 'no default route inside the container (internal network)')
    rc, o = sh("docker inspect $(docker ps -q --filter label=com.docker.compose.project=" + PROJECT + ") --format '{{.Name}} net={{.HostConfig.NetworkMode}} ports={{len .NetworkSettings.Ports}}'")
    bad = [ln for ln in o.splitlines() if 'net=host' in ln]
    add('isolation.no_host_network', 'FAIL' if bad else 'PASS', bad[:2] or o.splitlines()[:2])
    host_dom = os.environ.get('ROS_DOMAIN_ID', '(unset)')
    add('isolation.host_domain', 'PASS' if host_dom != domain_expected else 'WARN', f'host ROS_DOMAIN_ID={host_dom}; the twin domain {domain_expected} lives on an internal docker network only')
    return True


def stage_sim(nodes):
    print('== sim (project %s, image tag nightly)' % PROJECT)
    c = env_file(TW / 'scripts/contract/robot.env')
    sh(f'docker ps -aq --filter label=com.docker.compose.project={PROJECT} | xargs -r docker rm -f; docker network rm {PROJECT}_twin 2>/dev/null; true')  # leftovers of a crashed run
    rc, o = dc('build', 'sim', 'stack', timeout=1800)
    add('sim.build_stack_image', 'PASS' if rc == 0 else 'FAIL', 'x2-twin-stack:nightly rebuilt' if rc == 0 else o[-200:])
    if rc != 0:
        return
    sim_rc,sim_id=sh(['docker','image','inspect','x2-twin-sim:nightly','--format','{{.Id}}'])
    add('sim.official_image_id','PASS' if sim_rc==0 else 'FAIL',sim_id)
    rc, created = sh("docker image inspect -f '{{.Id}} {{.Created}}' x2-twin-stack:nightly")
    add('sim.image_id', 'PASS', created[:90])
    # 73/73 SHA256 inside the image (nav_control.launch.py is patched on purpose by the Dockerfile: it must be the ONLY difference, and only by the two documented hooks)
    rc, o = sh(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'python3', '-v', f'{ROOT}/tools/twin:/v/tools/twin:ro',
                '-v', f'{ROOT}/docs/evidence/x2_twin_20261030:/v/docs/evidence/x2_twin_20261030:ro', 'x2-twin-stack:nightly',
                '/v/tools/twin/verify_robot_baseline.py', '--root', '/v', '--stack-dir', '/opt/x2twin/stack'], timeout=120)
    diffs = [ln for ln in o.splitlines() if ln.startswith(('DIFFERENT', 'MISSING'))]
    only_patch = len(diffs) == 1 and 'navigation/nav_control.launch.py' in diffs[0] and diffs[0].startswith('DIFFERENT')
    add('sim.image_sha256_73', 'PASS' if (rc == 0 or only_patch) else 'FAIL', f'{o.splitlines()[-1] if o else rc}; differing: {[d.split("/")[-1] for d in diffs]}',
        expected_sim_patch='navigation/nav_control.launch.py' if only_patch else None)
    if only_patch:
        rc, o = sh(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'cat', 'x2-twin-stack:nightly', '/opt/x2twin/stack/navigation/nav_control.launch.py'])
        host = (ROOT / 'ws/src/navigation/nav_control.launch.py').read_text().splitlines()
        img = o.splitlines()
        import difflib
        d = [ln for ln in difflib.unified_diff(host, img, lineterm='', n=0) if ln[:1] in '+-' and ln[:3] not in ('+++', '---')]
        spec = importlib.util.spec_from_file_location('nav_control_patch', TW / 'docker/nav_control_patch.py')
        patch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(patch)
        expected = patch.transform((ROOT / 'ws/src/navigation/nav_control.launch.py').read_text())
        import hashlib
        hash_rc, hash_out = sh(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'sha256sum', 'x2-twin-stack:nightly', '/opt/x2twin/stack/navigation/nav_control.launch.py'])
        ok = rc == 0 and hash_rc == 0 and bool(hash_out.split()) and hash_out.split()[0] == hashlib.sha256(expected.encode()).hexdigest()
        add('sim.image_patch_is_documented', 'PASS' if ok else 'FAIL', f'{len(d)} changed line(s) in the image copy of nav_control.launch.py (Dockerfile SIM-only hooks: TWIN_SANDBOX guard, I1 inf_is_valid)', diff=d[:8])
    try:
        rc, o = dc('up', '-d', '--wait', '--wait-timeout', '240', 'stack-lite', timeout=330)
        add('sim.stack_lite_healthy', 'PASS' if rc == 0 else 'FAIL', 'stack-lite healthy (AMCL, Trust, adapter, coordinator, Nav2 on the graph)' if rc == 0 else o[-250:])
        if rc != 0:
            return
        rc, o = in_stack(f'sha256sum /opt/x2twin/stack/maps/{c["ROBOT_MAP"]}/map.pgm')
        sha = o.split()[0] if o else ''
        add('sim.map_in_container', 'PASS' if sha == c['ROBOT_MAP_SHA256'] else 'FAIL', f'{c["ROBOT_MAP"]} {sha[:16]}')
        rc, o = in_stack('ros2 param get /qazbot_d1_map/map_server yaml_filename; ros2 param get /qazbot_d1_map/localization_trust expected_map_sha256', timeout=60)
        add('sim.live_map_server_and_trust_map', 'PASS' if c['ROBOT_MAP'] in o and c['ROBOT_MAP_SHA256'] in o else 'FAIL', o.replace('\n', ' | ')[-220:])
        rc, o = in_stack('ros2 param get /qazbot_d1_map/nav2/controller_server goal_checker.xy_goal_tolerance; ros2 param get /qazbot_d1_map/nav2/controller_server goal_checker.yaw_goal_tolerance', timeout=60)
        got_tol = [float(v) for v in re.findall(r'Double value is: ([-+0-9.eE]+)', o)]
        add('sim.live_goal_tolerance', 'PASS' if rc == 0 and got_tol == [float(c['ROBOT_XY_TOL']), float(c['ROBOT_YAW_TOL'])] else 'FAIL', o.replace('\n', ' | ')[-160:])
        rc, o = in_stack("grep -c 'P23: pause, not reset' /opt/x2twin/stack/navigation/x2_motion_adapter.py; pgrep -af x2_motion_adapter | head -2")
        add('sim.p23_in_container', 'PASS' if o.splitlines()[0].strip() not in ('0', '') and 'x2_motion_adapter' in o else 'FAIL', o.replace('\n', ' | ')[:200])
        for fn, args in ((live_parity, (nodes,)), (tf_owners, ()), (costmap_sensor_contract, ()), (isolation, ())):
            try:
                fn(*args)
            except Exception as e:  # noqa: BLE001 - a crashed check is a FAIL in the report, never a silent abort
                add(f'sim.{fn.__name__}', 'FAIL', f'{type(e).__name__}: {e}')
        rc, o = in_stack("ros2 pkg xml nav2_amcl | sed -n 's/.*<version>\\(.*\\)<\\/version>.*/\\1/p' | head -1")
        add('sim.nav2_version', 'UNVERIFIED', f'twin nav2_amcl {o.strip()} vs robot 1.1.18 (snapshot); compatible regression tests not yet run: stays UNVERIFIED')
    finally:
        d_rc, d_o = dc('down', '-v', '--remove-orphans', timeout=180)
        sh(f'docker ps -aq --filter label=com.docker.compose.project={PROJECT} | xargs -r docker rm -f; docker network rm {PROJECT}_twin 2>/dev/null; true')
        add('sim.teardown', 'PASS' if d_rc == 0 else 'FAIL', f'{PROJECT} down rc={d_rc}')


def report(kind, started):
    d = RUN / 'preflight'
    d.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%dT%H%M%S')
    verdict = 'FAIL' if any(r['status'] == 'FAIL' for r in RES) else 'PASS'
    if START_HEAD and (gout('rev-parse', 'HEAD') != START_HEAD or mergeq.integration_head(ROOT) != START_INTEGRATION):
        add('git.changed_during_preflight', 'FAIL', 'Git refs moved during checks; rebuild and recheck')
        verdict = 'FAIL'
    doc = dict(kind=kind, mode=MODE, campaign_scope=os.environ.get('NIGHTLY_CAMPAIGN'), vendor_identity=next((r.get('hashes') for r in RES if r['name']=='vendor.identity'),{}), images={key:next((r['detail'].split()[0] for r in RES if r['name']==name and r['status']=='PASS' and r['detail'].split()),None) for key,name in (('stack','sim.image_id'),('sim','sim.official_image_id'))}, verdict=verdict, stamp=stamp, started=started, seconds=round(time.time() - started, 1), git_head=START_HEAD or gout('rev-parse', 'HEAD'), code_dirty=bool(gout('status', '--porcelain')),
               integration=START_INTEGRATION or gout('rev-parse', f'refs/heads/{mergeq.branch_name()}'), robot_ref=ROBOT_REF, robot_sha=gout('rev-parse', ROBOT_REF), checks=RES)
    (d / f'{stamp}_{kind}.json').write_text(json.dumps(doc, indent=1, default=str))
    (d / 'latest.json').write_text(json.dumps(doc, indent=1, default=str))
    md = [f'# Preflight {kind} {stamp}: {verdict}', '', f'control `{doc["git_head"][:9]}`, integration `{doc["integration"][:9]}`, robot ref `{doc["robot_sha"][:9]}`', '', '| status | check | detail |', '|---|---|---|']
    md += [f'| {r["status"]} | `{r["name"]}` | {r["detail"][:160].replace("|", "/")} |' for r in RES]
    (d / f'{stamp}_{kind}.md').write_text('\n'.join(md) + '\n')
    (d / 'latest.md').write_text('\n'.join(md) + '\n')
    n = {s: sum(1 for r in RES if r['status'] == s) for s in ('PASS', 'WARN', 'UNVERIFIED', 'FAIL')}
    print(f'== verdict {verdict} {n}  report: {d / (stamp + "_" + kind + ".md")}')
    return verdict


def main():
    global START_HEAD, START_INTEGRATION, MODE
    START_HEAD, START_INTEGRATION = gout('rev-parse', 'HEAD'), mergeq.integration_head(ROOT)
    ap = argparse.ArgumentParser()
    ap.add_argument('stage', choices=('static', 'sim', 'all'))
    ap.add_argument('--tests', action='store_true')
    ap.add_argument('--mode', choices=execution.MODES, default=execution.FULL)
    a = ap.parse_args()
    MODE = a.mode
    t0 = time.time()
    try:
        return run_stage(a, t0)
    except Exception as e:
        add('preflight.exception', 'FAIL', f'{type(e).__name__}: {e}')
        report(a.stage, t0)
        return 1


def run_stage(a, t0):
    if MODE==execution.ENGINEERING:
        stage_engineering()
        return 0 if report(a.stage,t0)=='PASS' else 1
    nodes = stage_static() if a.stage in ('static', 'all') else check_contract()
    if a.stage in ('sim', 'all'):
        fails = [r for r in RES if r['status'] == 'FAIL']
        if fails and a.stage == 'all':
            add('sim.skipped', 'WARN', 'static FAIL: containers are not started')
        elif nodes:
            stage_sim(nodes)
    if MODE==execution.L3 and not any(r['status']=='FAIL' for r in RES):
        rc,out=sh('PYTHONPATH=digital_twin/ws/src/x2_twin_sim python3 -m pytest -q -p no:cacheprovider digital_twin/ws/src/x2_twin_sim/test/test_frame_anchor.py',timeout=60)
        add('tests.frame_anchor','PASS' if rc==0 else 'FAIL',out[-150:])
    if a.tests:
        if any(r['status'] == 'FAIL' for r in RES):
            add('tests.skipped', 'WARN', 'preflight already failed; regression can be run independently')
        else:
            run_tests()
    return 0 if report(a.stage, t0) == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
