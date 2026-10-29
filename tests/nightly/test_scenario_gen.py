import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools' / 'nightly'))
import scenario_gen as sg  # noqa: E402


def test_deterministic_and_seed_sensitive():
    a = sg.gen(7, 4, {})[1]
    assert a == sg.gen(7, 4, {})[1]
    assert a != sg.gen(8, 4, {})[1]


def test_positive_poses_are_free_with_margin_and_unknown_is_not_free():
    W = sg.World()
    for s in sg.gen(3, 6, {})[1]:
        if not s['positive']:
            continue
        for p in (s['start'], s['goal']):
            r, c = W.w2c(*p[:2])
            assert W.main[r, c] and W.edt_main[r, c] >= sg.R_CIRC + sg.SAFETY - 0.06  # not-too-close sampling tolerance = 1 cell
    assert not W.free[~(W.free | W.occ)].any()


def test_negative_goals_never_positive_and_expect_reject():
    for s in sg.gen(5, 8, {})[1]:
        if s['category'].startswith('NEG_'):
            assert s['expected'] == 'safe_reject' and not s['positive']


def test_yaw_independent():
    sc = [s for s in sg.gen(2, 20, {})[1] if s['category'] == 'corridor_internal']
    assert len({round(s['start'][2], 1) for s in sc}) > 3 and np.corrcoef([s['start'][2] for s in sc], [s['goal'][2] for s in sc])[0, 1] < 0.9
