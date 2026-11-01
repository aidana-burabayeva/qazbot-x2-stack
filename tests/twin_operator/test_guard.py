"""The sim container guard: L3 semantics unchanged, OPERATOR_SIM added as a disjoint, non-qualifying capsule."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('check_authorization', ROOT / 'digital_twin/docker/sim/check_authorization.py')
G = importlib.util.module_from_spec(spec)
spec.loader.exec_module(G)
ENV = {'TWIN_MC': 'official', 'TWIN_MAP': 'd1_qazbot_20261029'}
NOW = 1_000_000.0


def l3(**kw):
    d = dict(mode='L3_QUALIFICATION', scope='stationary_official_mc', source_sha='a' * 40, campaign_sha='b' * 64, expires=NOW + 600,
             l3_workers=1, map='d1_qazbot_20261029', mc='official')
    d.update(kw)
    return d


def op(**kw):
    d = dict(mode='OPERATOR_SIM', scope='interactive_operator_mc', source_sha='a' * 40, session_id='s1', issued_by='operator_sim',
             qualifies=False, issued=NOW, expires=NOW + 3600, l3_workers=1, map='d1_qazbot_20261029', mc='official')
    d.update(kw)
    return d


def test_l3_capsule_still_accepted_exactly_as_before():
    assert G.validate(l3(), ENV, NOW) == 'L3_QUALIFICATION'
    assert G.validate(l3(mode='FULL_NIGHTLY'), ENV, NOW) == 'FULL_NIGHTLY'


@pytest.mark.parametrize('mut', [dict(scope='interactive_operator_mc'), dict(expires=NOW - 1), dict(l3_workers=2), dict(mc='r9'),
                                 dict(map='other'), dict(source_sha=''), dict(campaign_sha=''), dict(mode='ENGINEERING')])
def test_l3_capsule_variants_rejected(mut):
    with pytest.raises(ValueError):
        G.validate(l3(**mut), ENV, NOW)


def test_operator_capsule_accepted():
    assert G.validate(op(), ENV, NOW) == 'OPERATOR_SIM'


@pytest.mark.parametrize('mut', [dict(scope='stationary_official_mc'), dict(qualifies=True), dict(qualifies=None), dict(issued_by='nightly'),
                                 dict(session_id=''), dict(expires=NOW - 1), dict(expires=NOW + 9 * 3600), dict(issued=NOW + 5000),
                                 dict(l3_workers=0), dict(mc='r9'), dict(map='main_office'), dict(source_sha=None), dict(issued='x')])
def test_operator_capsule_variants_rejected(mut):
    with pytest.raises(ValueError):
        G.validate(op(**mut), ENV, NOW)


def test_operator_capsule_cannot_carry_a_nightly_scope_and_env_must_match():
    with pytest.raises(ValueError):
        G.validate(op(), {'TWIN_MC': 'r9', 'TWIN_MAP': 'd1_qazbot_20261029'}, NOW)
    with pytest.raises(ValueError):
        G.validate(op(), {'TWIN_MC': 'official', 'TWIN_MAP': 'first_map_nav2_v3'}, NOW)
    with pytest.raises(ValueError):
        G.validate(None, ENV, NOW)


def test_script_refuses_without_capsule(tmp_path):
    r = subprocess.run([sys.executable, str(ROOT / 'digital_twin/docker/sim/check_authorization.py')], capture_output=True, text=True,
                       env={'NIGHTLY_AUTHORIZATION_FILE': str(tmp_path / 'none.json'), 'TWIN_MAP': 'd1_qazbot_20261029', 'PATH': '/usr/bin'})
    assert r.returncode != 0 and 'official MC refused' in r.stderr
