"""Real-bag integration (recorded X2 clouds, SYNTHETIC re-ranged edge object).

Uses the recorded udp_20261028 cloud_registered bag; skipped if it is absent.
"""
import os

import numpy as np
import pytest

import replay_d04 as R

BAG = 'udp_20261028'
pytestmark = pytest.mark.skipif(not os.path.exists(R.BAGS[BAG]['db']), reason='bag not on disk')


@pytest.fixture(scope='module')
def ctx():
    frames, lsrc, _ = R.prepare(BAG)
    fz = R.floor_z(frames)
    A = R.load_bridge(R.RUNNING, 'lb_running_it')
    C = R.load_bridge(R.CANDIDATE, 'lb_candidate_it')
    return frames, fz, A, C


def _after(res, v):
    reg = np.array(res['out'][v]['region']).sum(axis=1)
    last_off = max(e[1] for e in res['events'])
    return reg, last_off


@pytest.mark.parametrize('name', ['S9_D04_right_-52deg_0.5m_transient', 'S10_D04_left_+48deg_0.55m_transient',
                                  'S2_left_+49deg_0.8m', 'S3_right_-55deg_0.65m'])
def test_edge_transient_kept_by_A_cleared_by_C(ctx, name):
    frames, fz, A, C = ctx
    spec = R.scenarios(len(frames))[name]
    res = R.run_scenario(frames, fz, name, spec, A, C, variants=('A', 'C'), collide=False)
    regA, off = _after(res, 'A')
    regC, _ = _after(res, 'C')
    assert regA[off - 1] > 0 and regC[off - 1] > 0          # both mark the object while present
    assert regA[-1] > 0                                     # running: edge marks never cleared (D-04)
    assert regC[-1] == 0                                    # candidate: cleared by new evidence
    assert regC[off] == 0                                   # ... on the first update after it left


def test_blind_sector_seeds_are_never_cleared(ctx):
    frames, fz, A, C = ctx
    name = 'S8_blind_seed_absence_not_free'
    res = R.run_scenario(frames, fz, name, R.scenarios(len(frames))[name], A, C, variants=('A', 'C'),
                         collide=False)
    assert all(res['out']['A']['seed_kept'])
    assert all(res['out']['C']['seed_kept'])


def test_recorded_scene_real_obstacles_identical_in_A_and_C(ctx):
    frames, fz, A, C = ctx
    name = 'recorded_only'
    res = R.run_scenario(frames, fz, name, R.scenarios(len(frames))[name], A, C, variants=('A', 'C'),
                         collide=False)
    fa = {tuple(c) for c in res['out']['A']['final_cells']}
    fc = {tuple(c) for c in res['out']['C']['final_cells']}
    assert fa == fc and len(fa) > 0
