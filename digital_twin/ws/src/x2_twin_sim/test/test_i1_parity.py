"""The twin's copy of the I1 scan semantics must stay identical to the candidate level_bridge.py (skipped where the candidate is absent)."""
import inspect
import os

import pytest

from x2_twin_sim import level_scan

CANDIDATE = os.environ.get('TWIN_I1_CANDIDATE', '/work/candidates/i1_rpp_collision/level_bridge.py')


def _block(text, start, end):
    a = text.index(start)
    return text[a:text.index(end, a)].strip()


@pytest.mark.skipif(not os.path.exists(CANDIDATE), reason='candidate not mounted')
def test_build_ranges_is_identical_to_the_candidate():
    cand = open(CANDIDATE).read()
    mine = inspect.getsource(level_scan)
    assert _block(cand, '#: Observed field of view', 'class LevelBridge') == mine[mine.index('#: Observed field of view'):].strip()
