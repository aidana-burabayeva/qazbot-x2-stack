"""Operator Skills package: integrity, operator-only flag, drift, conflicts, roles, Git-based update, LAB<->ROBOT contract."""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
OPS = REPO / 'tools' / 'operator_skills' / 'opskills.py'
LAB = REPO / 'tools' / 'nightly' / 'lab.py'


def g(cwd, *a):
    return subprocess.run(['git', '-C', str(cwd), *a], capture_output=True, text=True, check=True).stdout.strip()


def ops(root, *a, home=None, **env):
    e = dict(os.environ, OPSKILLS_ROOT=str(root), HOME=str(home or root.parent / 'home'), QAZBOT_ROLE_FILE=str(root.parent / 'norole'), **env)
    return subprocess.run([sys.executable, str(OPS), *a], capture_output=True, text=True, env=e)


@pytest.fixture
def pkg(tmp_path):
    """A throw-away copy of the package in its own git repo with a bare remote."""
    remote = tmp_path / 'r.git'
    subprocess.run(['git', 'init', '-q', '--bare', '-b', 'work', str(remote)], check=True)
    w = tmp_path / 'w'
    subprocess.run(['git', 'clone', '-q', str(remote), str(w)], check=True, capture_output=True)
    g(w, 'config', 'user.email', 'a@a'); g(w, 'config', 'user.name', 'a'); g(w, 'checkout', '-q', '-b', 'work')
    shutil.copytree(REPO / '.claude' / 'skills', w / '.claude' / 'skills')
    (w / 'docs' / 'hardware').mkdir(parents=True)
    g(w, 'add', '-A'); g(w, 'commit', '-qm', 'pkg 1.0.0'); g(w, 'push', '-q', '-u', 'origin', 'work')
    (tmp_path / 'home').mkdir()
    return w, remote, tmp_path


def test_real_tree_validates_for_lab_and_all_control_skills_are_operator_only():
    r = ops(REPO, 'validate', '--role', 'lab', home=REPO.parent / 'nohome')
    assert r.returncode == 0, r.stdout
    for n in ('nightly-stop', 'nightly-resume', 'nightly-status', 'robot-sync-validate'):
        assert 'disable-model-invocation: true' in (REPO / '.claude' / 'skills' / n / 'SKILL.md').read_text()


def test_roles_split_lab_and_robot(pkg):
    w, *_ = pkg
    lab = json.loads(ops(w, 'validate', '--role', 'lab', '--json').stdout)
    rob = json.loads(ops(w, 'validate', '--role', 'robot', '--json').stdout)
    assert lab['active_for_role'] == ['nightly-resume', 'nightly-status', 'nightly-stop'] and rob['active_for_role'] == ['robot-sync-validate']


def test_missing_operator_only_flag_fails(pkg):
    w, *_ = pkg
    p = w / '.claude/skills/nightly-stop/SKILL.md'
    p.write_text(p.read_text().replace('disable-model-invocation: true\n', ''))
    r = ops(w, 'validate', '--role', 'lab')
    assert r.returncode == 1 and 'operator-only' in r.stdout


def test_drift_detected_and_repinned_only_deliberately(pkg):
    w, *_ = pkg
    p = w / '.claude/skills/nightly-status/SKILL.md'
    p.write_text(p.read_text() + '\nextra\n')
    r = ops(w, 'validate', '--role', 'lab')
    assert r.returncode == 1 and 'changed since pinned' in r.stdout
    assert ops(w, 'hashes', '--update').returncode == 0 and ops(w, 'validate', '--role', 'lab').returncode == 0


def test_unlisted_skill_and_user_level_name_conflict_fail(pkg):
    w, _, tmp = pkg
    (w / '.claude/skills/rogue').mkdir(); (w / '.claude/skills/rogue/SKILL.md').write_text('---\nname: rogue\n---\n')
    assert 'neither managed' in ops(w, 'validate', '--role', 'lab').stdout
    shutil.rmtree(w / '.claude/skills/rogue')
    h = tmp / 'home2'; (h / '.claude' / 'commands').mkdir(parents=True); (h / '.claude/commands/nightly-stop.md').write_text('x')
    r = ops(w, 'validate', '--role', 'lab', home=h)
    assert r.returncode == 1 and 'conflicts' in r.stdout


def publish_update(w, remote, tmp, version='1.1.0', contract=None):
    other = tmp / 'pub'
    subprocess.run(['git', 'clone', '-q', '-b', 'work', str(remote), str(other)], check=True, capture_output=True)
    g(other, 'config', 'user.email', 'p@p'); g(other, 'config', 'user.name', 'p')
    mp = other / '.claude/skills/OPERATOR_SKILLS.json'
    m = json.loads(mp.read_text()); m['version'] = version
    if contract:
        m['contract_version'] = contract
    mp.write_text(json.dumps(m)); g(other, 'commit', '-qam', 'bump'); g(other, 'push', '-q')
    g(w, 'fetch', '-q')


def test_update_via_git_ff_only_no_manual_copy(pkg):
    w, remote, tmp = pkg
    publish_update(w, remote, tmp, '1.1.0')
    s = json.loads(ops(w, 'sync').stdout)
    assert s['verdict'] == 'UPDATE_AVAILABLE' and s['note'].startswith('git merge --ff-only')
    g(w, 'merge', '-q', '--ff-only', 'origin/work')
    assert json.loads(ops(w, 'sync').stdout)['verdict'] == 'UP_TO_DATE'


def test_update_with_dirty_or_diverged_checkout_is_not_blind(pkg):
    w, remote, tmp = pkg
    publish_update(w, remote, tmp, '1.1.0')
    (w / '.claude/skills/nightly-stop/SKILL.md').write_text('local edit')
    assert 'do not pull blindly' in json.loads(ops(w, 'sync').stdout)['note']
    g(w, 'checkout', '-q', '--', '.'); (w / 'x.txt').write_text('x'); g(w, 'add', '-A'); g(w, 'commit', '-qm', 'local')
    assert 'do not pull blindly' in json.loads(ops(w, 'sync').stdout)['note']


def test_major_or_contract_change_is_incompatible(pkg):
    w, remote, tmp = pkg
    publish_update(w, remote, tmp, '2.0.0', contract=2)
    assert json.loads(ops(w, 'sync').stdout)['verdict'] == 'INCOMPATIBLE_MAJOR'


def test_handoff_contract_version_mismatch_is_error(pkg):
    w, *_ = pkg
    (w / 'docs/hardware/LAB_TO_ROBOT_HANDOFF.md').write_text('<!-- QAZBOT-CONTRACT {"contract_version": 99} -->\n')
    assert ops(w, 'validate', '--role', 'lab').returncode == 1


@pytest.fixture
def lab(tmp_path):
    remote = tmp_path / 'lr.git'
    subprocess.run(['git', 'init', '-q', '--bare', '-b', 'work', str(remote)], check=True)
    w = tmp_path / 'lw'
    subprocess.run(['git', 'clone', '-q', str(remote), str(w)], check=True, capture_output=True)
    g(w, 'config', 'user.email', 't@t'); g(w, 'config', 'user.name', 't'); g(w, 'checkout', '-q', '-b', 'work')
    (w / 'docs/nightly').mkdir(parents=True); (w / 'docs/nightly/STATUS.md').write_text('#\n'); (w / '.gitignore').write_text('run/\n')
    g(w, 'add', '-A'); g(w, 'commit', '-qm', 'i'); g(w, 'push', '-q', '-u', 'origin', 'work')
    e = dict(os.environ, NIGHTLY_ROOT=str(w), NIGHTLY_RUN=str(w / 'run/nightly'), NIGHTLY_UNIT_DIR=str(tmp_path / 'u'), QAZBOT_ROLE='LAB')
    return w, remote, e


def labrun(e, *a):
    return subprocess.run([sys.executable, str(LAB), *a], capture_output=True, text=True, env=e)


STOP = ('stop', '--no-systemd', '--no-docker', '--no-notify', '--no-tests')


def test_stop_writes_contract_block_and_verification_classes(lab):
    w, remote, e = lab
    assert labrun(e, *STOP).returncode == 0
    t = (w / 'docs/hardware/LAB_TO_ROBOT_HANDOFF.md').read_text()
    m = json.loads(t.split('<!-- QAZBOT-CONTRACT ')[1].split(' -->')[0])
    assert m['contract_version'] == 1 and m['lab_sha'] and 'SIM_VERIFIED' in t and 'HARDWARE_VERIFIED' in t
    assert 'HARDWARE_VERIFIED**: нет' in t  # never invented


def test_robot_role_checkout_refuses_lab_stop_and_resume(lab):
    w, remote, e = lab
    for c in (STOP, ('resume', '--no-systemd')):
        r = labrun(dict(e, QAZBOT_ROLE='ROBOT'), *c)
        assert r.returncode != 0 and 'role is ROBOT' in (r.stdout + r.stderr)


def test_resume_reads_robot_feedback_checkpoints_and_prioritises_blocker(lab, tmp_path):
    w, remote, e = lab
    labrun(e, *STOP)
    other = tmp_path / 'robot'
    subprocess.run(['git', 'clone', '-q', '-b', 'work', str(remote), str(other)], check=True, capture_output=True)
    g(other, 'config', 'user.email', 'r@r'); g(other, 'config', 'user.name', 'r')
    fb = other / 'docs/hardware/ROBOT_TO_LAB_FEEDBACK.md'
    fb.parent.mkdir(parents=True, exist_ok=True)
    fb.write_text('<!-- QAZBOT-CONTRACT {"contract_version": 1, "last_lab_integrated_sha": "aaa111", "last_lab_reviewed_sha": "bbb222"} -->\nTWIN_MISMATCH yaw\n')
    g(other, 'add', '-A'); g(other, 'commit', '-qm', 'robot: feedback'); g(other, 'push', '-q')
    r = labrun(e, 'resume', '--no-systemd')
    assert 'integrated=aaa111' in r.stdout and 'triage_robot_findings' in subprocess.run([sys.executable, str(REPO / 'tools/nightly/nq.py'), 'status'], capture_output=True, text=True, env=e).stdout


def test_no_physical_robot_commands_in_lab_or_lab_skills():
    import re
    bad = re.compile(r'\b(ssh|scp|rsync)\b|ros2 (topic pub|service call)|colcon|systemctl .*x2', re.I)
    for p in [LAB, REPO / 'tools/operator_skills/opskills.py', *(REPO / '.claude/skills').glob('nightly-*/SKILL.md')]:
        for ln in p.read_text().splitlines():
            if bad.search(ln) and 'ssh|scp|rsync' not in ln and 'не ssh' not in ln.lower() and 'ssh' not in ln.lower().replace('не ssh', ''):
                raise AssertionError(f'{p}: {ln}')


def test_unknown_role_also_refuses_lab_stop(lab):
    w, remote, e = lab
    e = dict(e, QAZBOT_ROLE='', QAZBOT_ROLE_FILE=str(w.parent / 'norole'))
    r = labrun(e, *STOP)
    assert r.returncode != 0 and 'role is UNKNOWN' in (r.stdout + r.stderr)


def test_role_subset_install_robot_has_only_robot_skill(pkg):
    w, *_ = pkg
    for n in ('nightly-stop', 'nightly-resume', 'nightly-status'):
        shutil.rmtree(w / '.claude/skills' / n)
    r = json.loads(ops(w, 'validate', '--role', 'robot', '--json').stdout)
    assert r['errors'] == [] and r['active_for_role'] == ['robot-sync-validate'] and len(r['not_installed']) == 3
    assert ops(w, 'validate', '--role', 'lab').returncode == 1  # a LAB checkout missing its own skills is an error


def test_verify_evidence_does_not_quarantine_inflight_publish_of_live_process(tmp_path):
    sys.path.insert(0, str(REPO / 'tools/nightly'))
    import sqlite3, store
    run = tmp_path / 'run'; tmpd = run / 'evidence' / f'.tmp-abc-{os.getpid()}'; tmpd.mkdir(parents=True)
    dead = run / 'evidence' / '.tmp-def-999999999'; dead.mkdir()
    db = sqlite3.connect(':memory:'); db.row_factory = sqlite3.Row; store.init(db)
    rep = store.adopt_orphans(db, run)
    assert tmpd.exists() and not dead.exists() and len(rep['quarantined']) == 1
