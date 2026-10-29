"""Offline regression of P20 against the exact navigation-branch source snapshot."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ws/src/navigation'))
from x2_motion_adapter import (AdapterCore, Config, McState, YawRateEstimator,
                               YawRateLoop, BODY_STAND, FSM_STABLE, LATCHED)


@pytest.mark.parametrize('start,end,expected', [(math.pi - .05, -math.pi + .05, 1.),
                                              (-math.pi + .05, math.pi - .05, -1.)])
def test_estimator_wraps_pi(start, end, expected):
    est = YawRateEstimator()
    est.update(10., start, 10.)
    est.update(10.1, end, 10.1)
    assert est.get(10.1, .3) == pytest.approx(expected)
    assert est.get(10.41, .3) is None


@pytest.mark.parametrize('next_stamp', [9., 11.])
def test_estimator_discards_rate_after_clock_jump_or_gap(next_stamp):
    est = YawRateEstimator()
    est.update(10., 0., 10.)
    est.update(10.1, .1, 10.1)
    est.update(next_stamp, .2, 11.)
    assert est.get(11., .3) is None
    est.update(next_stamp + .1, .3, 11.1)
    assert est.get(11.1, .3) == pytest.approx(1.)


def test_loop_integrates_and_backs_off_on_overspeed_and_direction_change():
    loop = YawRateLoop(Config(yaw_settle_s=0.))
    for i in range(30):
        before = loop.update(i * .1, .5, 0.)
    assert loop.integ > 0
    after = loop.update(3., .5, 1.5)
    assert after < before
    loop.update(3.1, -.5, 0.)
    assert loop.sign == -1 and loop.integ == 0


def mc(now, bounds=(-1., 1.)):
    return McState(stamp=now, source='rc', body=BODY_STAND, fsm=FSM_STABLE, ang=bounds)


def drive(core, now, cmd=(0., 0., .5), rate=0., **kwargs):
    return core.step(now, cmd, now, mc(now, (-.4, .4)), yaw_rate=rate, **kwargs)


def test_mc_limit_and_stale_feedback_passthrough():
    core = AdapterCore(Config())
    result = drive(core, 1.)
    assert result.publish[2] == pytest.approx(.4)
    result = drive(core, 1.1, rate=None)
    assert result.publish[2] == pytest.approx(.4)
    assert 'yaw_meas_stale' in result.notes and not core.yaw.active


def test_translation_bypasses_loop_and_resets_integral():
    core = AdapterCore(Config(yaw_settle_s=0.))
    for i in range(10):
        drive(core, 1. + i * .1)
    assert core.yaw.integ > 0
    result = drive(core, 2., cmd=(.2, 0., .3))
    assert result.publish == pytest.approx((.2, 0., .3))
    assert core.yaw.integ == 0 and not core.yaw.active


@pytest.mark.parametrize('pause', [False, True])
def test_stop_or_t1_pause_resets_integral(pause):
    core = AdapterCore(Config(yaw_settle_s=0.))
    for i in range(10):
        drive(core, 1. + i * .1)
    assert core.yaw.integ > 0
    result = (drive(core, 2., localized=False, loc_reason='T1_scan_stale', t1_ok=True)
              if pause else drive(core, 2., cmd=(0., 0., 0.)))
    assert result.publish == (0., 0., 0.)
    assert core.yaw.integ == 0 and not core.yaw.active


def test_localization_fault_latches_and_rejects_motion():
    core = AdapterCore(Config())
    drive(core, 1.)
    result = drive(core, 1.1, localized=False, loc_reason='untrusted')
    assert result.state == LATCHED and result.publish == (0., 0., 0.)
    assert drive(core, 1.2).state == LATCHED
