"""Operator session CLI: capsule, authorization, preflight pure parts, and the isolation properties of the compose project."""
import ast
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/twin'))
import operator_sim as o  # noqa: E402

CLI_SRC = (ROOT / 'tools/twin/operator_sim.py').read_text()


def test_capsule_is_operator_scoped_non_qualifying_and_bounded():
    c = o.make_capsule('op_x', 'a' * 40, 2.0, now=1000.0)
    assert c['mode'] == 'OPERATOR_SIM' and c['scope'] == 'interactive_operator_mc' and c['qualifies'] is False
    assert c['expires'] - c['issued'] == 7200 and c['l3_workers'] == 1 and c['mc'] == 'official' and c['map'] == 'd1_qazbot_20261029'
    assert 'campaign_sha' not in c
    for bad in (0, -1, o.MAX_HOURS + 0.1):
        with pytest.raises(SystemExit):
            o.make_capsule('op_x', 'a' * 40, bad)


def test_capsule_passes_the_container_guard_but_nightly_capsule_does_not_pass_as_operator():
    import importlib.util
    spec = importlib.util.spec_from_file_location('ca', ROOT / 'digital_twin/docker/sim/check_authorization.py')
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    env = {'TWIN_MC': 'official', 'TWIN_MAP': 'd1_qazbot_20261029'}
    assert g.validate(o.make_capsule('op_x', 'a' * 40, 1.0), env) == 'OPERATOR_SIM'


def test_authorization_requires_exact_phrase_or_tty(monkeypatch):
    class A:
        authorize = ''
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    with pytest.raises(SystemExit):
        o.authorize(A)
    A.authorize = 'yes'
    with pytest.raises(SystemExit):
        o.authorize(A)
    A.authorize = o.PHRASE
    assert o.authorize(A) is True


def test_pose_parsing():
    assert o.parse_pose('-8.3,-1.2,-2.1') == (-8.3, -1.2, -2.1)
    for bad in ('1,2', 'a,b,c', '1e9,0,0'):
        with pytest.raises(SystemExit):
            o.parse_pose(bad)


def test_resource_guard(tmp_path):
    mi = tmp_path / 'meminfo'
    mi.write_text('MemTotal: 66000000 kB\nMemAvailable: 8000000 kB\n')
    res = {n: s for n, s, _ in o.check_resources(str(mi), disk_path=str(tmp_path), need_disk=0.1)}
    assert res['resources.ram'] == 'FAIL' and res['resources.disk'] == 'PASS'
    mi.write_text('MemTotal: 66000000 kB\nMemAvailable: 60000000 kB\n')
    res = {n: s for n, s, _ in o.check_resources(str(mi), disk_path=str(tmp_path), need_disk=10 ** 6)}
    assert res['resources.ram'] == 'PASS' and res['resources.disk'] == 'FAIL'


def test_disk_threshold_keeps_nightlys_own_floor():
    assert o.NIGHTLY_DISK_FLOOR_GIB >= 5.0 and o.EVIDENCE_GIB > 0


def test_cli_has_no_robot_access_paths():
    tree = ast.parse(CLI_SRC)
    called = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in ('run', 'Popen', 'check_output', 'call'):
            for a in n.args[:1]:
                if isinstance(a, (ast.List, ast.Tuple)) and a.elts and isinstance(a.elts[0], ast.Constant):
                    called.append(str(a.elts[0].value))
    assert not {'ssh', 'scp', 'sftp', 'rsync', 'sshpass', 'nc', 'telnet'} & set(called), called
    assert '192.168.' not in CLI_SRC and 'robot66' not in CLI_SRC


@pytest.mark.skipif(not shutil.which('docker'), reason='docker CLI needed to render the compose config')
def test_compose_project_is_isolated_from_lan_nightly_and_host(tmp_path):
    st = dict(capsule=str(tmp_path / 'w.json'), snapshot=str(tmp_path), evidence=str(tmp_path), empty_x11=str(tmp_path),
              start_pose_text='-8.344,-1.213,-2.145', mujoco_gui=False, rviz=True)
    r = subprocess.run(o.compose_cmd(True) + ['config', '--format', 'json'], capture_output=True, text=True, env=o.compose_env(st), cwd=o.TW)
    assert r.returncode == 0, r.stderr
    cfg = json.loads(r.stdout)
    assert cfg['name'] == 'x2-operator' and o.PROJECT == 'x2-operator'
    assert cfg['networks']['twin']['internal'] is True and cfg['networks']['twin']['name'] == 'x2-operator_twin'
    for name, svc in cfg['services'].items():
        assert svc.get('network_mode') is None and not svc.get('ports'), name
        assert not svc.get('privileged') and 'ALL' in (svc.get('cap_drop') or []), name
        assert list(svc.get('networks', {'twin': 1})) == ['twin'], name
        assert not any(v.get('source', '').startswith(('/var/run/docker.sock', '/run/docker.sock')) for v in svc.get('volumes', [])), name
        env = svc.get('environment', {})
        assert not any('192.168.' in str(v) for v in env.values()), name
    assert cfg['services']['sim']['image'].startswith('x2-twin-sim:') and cfg['services']['sim']['environment'].get('ROS_DOMAIN_ID', '87') == '87'
    mounts = {v['target']: v for v in cfg['services']['sim']['volumes']}
    assert mounts['/run/qazbot/worker.json']['read_only'] is True
    lock_src = mounts['/run/qazbot/locks']['source']
    assert lock_src.endswith('run/nightly/l3-locks'), lock_src          # same official-mc.lock as the Nightly L3 worker
    # the only writable host mount of the GUI/recorder containers is the session evidence directory
    for svc in ('panel', 'recorder', 'rviz'):
        for v in cfg['services'][svc]['volumes']:
            if v['target'] in ('/results',):
                assert v['source'] == str(tmp_path)
            elif v['target'] != '/tmp/.X11-unix':
                assert v.get('read_only') or v.get('type') == 'tmpfs' or v['target'].startswith('/opt/x2twin/stack/maps'), (svc, v)


def test_operator_start_does_not_import_or_write_the_nightly_queue():
    src = CLI_SRC
    for forbidden in ('import nq', 'queue.db', 'execution_authorization', 'kv_set', 'execution.allowed', 'lab_state'):
        assert forbidden not in src, forbidden
    assert "workflow.py" in src and "'feedback'" in src       # the one sanctioned hand-off
