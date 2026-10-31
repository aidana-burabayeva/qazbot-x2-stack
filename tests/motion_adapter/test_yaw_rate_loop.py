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


def _wound_up():
    core = AdapterCore(Config(yaw_settle_s=0.))
    for i in range(10):
        drive(core, 1. + i * .1)
    assert core.yaw.integ > 0
    return core


def test_translation_bypasses_loop_and_pauses_integral():
    """P23 contract (deployed on X2 2026-10-30): a translation tick hands the output back and PAUSES the loop; I is kept, frozen."""
    core = _wound_up()
    kept = core.yaw.integ
    result = drive(core, 2., cmd=(.2, 0., .3))
    assert result.publish == pytest.approx((.2, 0., .3))
    assert core.yaw.integ == kept and not core.yaw.active


@pytest.mark.parametrize('pause', [False, True])
def test_stop_or_t1_pause_keeps_integral_frozen_for_hold_window(pause):
    core = _wound_up()
    kept = core.yaw.integ
    result = (drive(core, 2., localized=False, loc_reason='T1_scan_stale', t1_ok=True)
              if pause else drive(core, 2., cmd=(0., 0., 0.)))
    assert result.publish == (0., 0., 0.)
    assert core.yaw.integ == kept and not core.yaw.active


def test_resume_within_hold_keeps_integral_and_after_hold_resets_it():
    core = _wound_up()
    kept = core.yaw.integ
    drive(core, 2., cmd=(0., 0., 0.))
    drive(core, 2.5)                                   # same sign, 0.5 s <= yaw_hold_s: I survives the interruption
    assert core.yaw.integ >= kept * 0.99
    core = _wound_up()
    drive(core, 2., cmd=(0., 0., 0.))
    drive(core, 2. + core.cfg.yaw_hold_s + .5)         # paused longer than yaw_hold_s: restart from zero
    assert core.yaw.integ == 0


def test_target_sign_change_resets_integral():
    core = _wound_up()
    drive(core, 2., cmd=(0., 0., -.5))
    assert core.yaw.integ == 0


def test_localization_fault_latches_and_rejects_motion():
    core = AdapterCore(Config())
    drive(core, 1.)
    result = drive(core, 1.1, localized=False, loc_reason='untrusted')
    assert result.state == LATCHED and result.publish == (0., 0., 0.)
    assert drive(core, 1.2).state == LATCHED


@pytest.mark.parametrize('rate', [float('nan'), float('inf'), -float('inf')])
def test_invalid_measurement_never_reaches_integrator(rate):
    core = AdapterCore(Config())
    result = drive(core, 1., rate=rate)
    assert result.publish[2] == pytest.approx(.4)
    assert not core.yaw.active and core.yaw.integ == 0


@pytest.mark.parametrize('cmd', [(.02, 0., .5), (0., 0., .05), (0., 0., -.05)])
def test_pure_rotation_thresholds_do_not_activate_loop(cmd):
    core = AdapterCore(Config())
    result = core.step(1., cmd, 1., mc(1.), yaw_rate=0.)
    assert result.publish == cmd
    assert not core.yaw.active


def test_dead_time_then_rate_limit_and_antiwindup():
    cfg = Config()
    loop = YawRateLoop(cfg)
    before = loop.update(0., .5, 0.)
    for i in range(1, 30):
        after = loop.update(i*.02, .5, 0.)
        assert loop.integ == 0  # no integration during MC startup dead time
        assert after == before
    for i in range(30, 1000):
        after = loop.update(i*.02, .5, 0.)
        assert abs(after-before) <= cfg.yaw_du_max*.02 + 1e-12
        assert cfg.yaw_u_min_ccw <= after <= cfg.yaw_u_max
        before = after
    saturated_integral = loop.integ
    for i in range(1000, 1100):
        assert loop.update(i*.02, .5, 0.) == pytest.approx(cfg.yaw_u_max)
    assert loop.integ == pytest.approx(saturated_integral)


def test_stale_command_and_takeover_reset_active_yaw_loop():
    core = AdapterCore(Config())
    drive(core, 1.)
    result = core.step(2., (0., 0., .5), 1., mc(2.), yaw_rate=0.)
    assert result.publish == (0., 0., 0.) and not core.yaw.active
    drive(core, 2.1)
    other = mc(2.2)
    other.source = 'vr'  # accepted built-in source; a change while active is takeover
    result = core.step(2.2, (0.,0.,.5), 2.2, other, yaw_rate=0.)
    assert result.state == LATCHED and result.publish == (0.,0.,0.)
    assert not core.yaw.active and core.yaw.integ == 0
