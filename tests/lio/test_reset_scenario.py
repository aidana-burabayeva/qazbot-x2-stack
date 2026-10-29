"""Measured-size reset and data-gap regression of the current fail-closed guard."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ws/src/lio'))
import native_odom as no


@pytest.mark.parametrize('gap', [.1, 40.])
@pytest.mark.parametrize('displacement', [.467, 1.563])
def test_origin_jump_remains_latched_after_gap(gap, displacement):
    guard = no.native_guard()
    q = (0., -math.sqrt(.5), 0., math.sqrt(.5))
    add = lambda t, x: guard.add(t, no.NATIVE_FRAME, no.NATIVE_CHILD, (x, 0., 1., *q), t)
    assert add(100., 0.) == 'ok'
    old_pose = guard.pose
    assert guard.fault(100. + gap) == ('stale' if gap > no.NATIVE_MAX_AGE else '')
    assert add(100. + gap, displacement) == 'origin_reset'
    assert guard.pose == old_pose
    assert guard.fault(100. + gap).startswith('origin_reset:')
    assert add(100. + gap + .01, displacement) == 'latched'
