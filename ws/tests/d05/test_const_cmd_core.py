"""Unit tests of the D-05 fixed-command timing/limits logic. No ROS, no sleeps, no wall clock."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'd05'))
import const_cmd_core as c  # noqa: E402

GOOD = {'live': True, 'mode': 'live', 'registration': 'registered', 'motion_armed': True, 'state': 'idle',
        'latch': '', 'ready': True, 'foreign_publishers': [], 'mc_source': 'rc', 'reason': 'no_command',
        't1_pause': False}


def st(**kw):
    d = dict(GOOD)
    d.update(kw)
    return d


# ---- limits
@pytest.mark.parametrize('vx', [0.10, 0.30, 0.60, -0.30, -0.60, 0.02])
def test_vx_accepted(vx):
    assert c.validate_params(vx, 10.0).vx == vx


@pytest.mark.parametrize('vx', [0.6000001, 0.61, 0.8, -0.61, -1.0, 0.0, 0.019, float('nan'), float('inf'), -float('inf')])
def test_vx_rejected(vx):
    with pytest.raises(c.LimitError):
        c.validate_params(vx, 10.0)


@pytest.mark.parametrize('d', [3.0, 7.0, 12.0])
def test_duration_accepted(d):
    assert c.validate_params(0.3, d).duration_s == d


@pytest.mark.parametrize('d', [12.0001, 13, 100, 0, -1, 2.9, float('nan'), float('inf')])
def test_duration_rejected(d):
    with pytest.raises(c.LimitError):
        c.validate_params(0.3, d)


def test_settle_must_leave_steady_segment():
    assert c.validate_params(0.3, 3.0, settle_s=2.0)
    with pytest.raises(c.LimitError):
        c.validate_params(0.3, 2.9, settle_s=2.0)
    with pytest.raises(c.LimitError):
        c.validate_params(0.3, 10.0, settle_s=-0.1)


@pytest.mark.parametrize('rate', [19.9, 0, 50.1, 100, float('nan')])
def test_rate_rejected(rate):
    with pytest.raises(c.LimitError):
        c.validate_params(0.3, 10.0, rate_hz=rate)


def test_rate_default_is_nav2_rate():
    assert c.validate_params(0.3, 10.0).rate_hz == 20.0 == c.NAV2_RATE_HZ


def test_non_numeric_rejected():
    with pytest.raises(c.LimitError):
        c.validate_params('0.3', 10.0)
    with pytest.raises(c.LimitError):
        c.validate_params(0.3, None)


def test_all_protocol_runs_are_valid():
    # P-D05 table: (vx, duration)
    for vx, d in [(0.30, 10), (0.10, 12), (0.15, 12), (0.20, 10), (0.25, 10), (0.40, 8), (0.50, 8), (0.60, 7)]:
        c.validate_params(vx, d)


# ---- schedule
def test_schedule_phases_and_commands():
    s = c.Schedule(c.validate_params(0.3, 10.0))
    assert s.phase(0.0) == 'settle'
    assert s.phase(1.999) == 'settle'
    assert s.phase(2.0) == 'steady'
    assert s.phase(9.999) == 'steady'
    assert s.phase(10.0) == 'done'
    assert s.phase(50.0) == 'done'
    assert s.command(0.0) == (0.3, 0.0, 0.0)
    assert s.command(9.99) == (0.3, 0.0, 0.0)
    assert s.command(10.0) == (0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        s.phase(-0.1)


def test_schedule_wz_and_vy_always_zero():
    s = c.Schedule(c.validate_params(-0.4, 8.0))
    for k in range(0, 200):
        cmd = s.command(k * 0.05)
        assert cmd[1] == 0.0 and cmd[2] == 0.0


def test_duration_includes_settle_total_message_count():
    s = c.Schedule(c.validate_params(0.3, 10.0))
    assert s.nominal_messages() == 200            # 10 s * 20 Hz, settle included
    nonzero = [k for k in range(400) if s.command(k / 20.0)[0] != 0.0]
    assert len(nonzero) == 200 and max(nonzero) / 20.0 < 10.0
    s50 = c.Schedule(c.validate_params(0.3, 7.0, rate_hz=50.0))
    assert s50.nominal_messages() == 350


def test_commands_never_exceed_limit_on_grid():
    for vx in (0.6, -0.6):
        s = c.Schedule(c.validate_params(vx, 12.0))
        assert all(abs(s.command(k / 20.0)[0]) <= c.MAX_ABS_VX for k in range(300))


# ---- start checks
def pre(status=GOOD, age=0.2, gc=None, others=(), rc_age=None, subs=1):
    return c.preflight_refusals(status, age, gc, others, rc_age, subs)


def test_preflight_ok():
    assert pre() == []
    assert pre(gc={'busy': False, 'latched': False}) == []
    assert pre(rc_age=5.0) == []              # old RC activity is not "active"


@pytest.mark.parametrize('kw', [
    dict(live=False, mode='dry_run'), dict(live=True, mode='dry_run'), dict(registration='not_requested'),
    dict(registration='pending'), dict(motion_armed=False), dict(state='active'), dict(state='latched'),
    dict(latch='operator_takeover'), dict(ready=False), dict(foreign_publishers=['x']), dict(mc_source='pnc'),
    dict(t1_pause=True)])
def test_preflight_refuses_bad_adapter_status(kw):
    assert pre(status=st(**kw))


def test_preflight_refuses_missing_or_stale_status():
    assert pre(status=None, age=None)
    assert pre(age=1.01)
    assert pre(status={}, age=0.1)            # empty dict: every field missing -> refuse


def test_preflight_refuses_gc_busy_or_latched():
    assert pre(gc={'busy': True})
    assert pre(gc={'latched': True, 'fault': 'x'})


def test_preflight_refuses_other_publisher_and_no_subscriber():
    assert pre(others=['nav2_controller'])
    assert pre(subs=0)


def test_preflight_refuses_rc_active():
    assert pre(rc_age=0.0)
    assert pre(rc_age=c.RC_QUIET_S)
    assert not c.rc_active(None)
    assert not c.rc_active(c.RC_QUIET_S + 0.01)


def test_parse_status():
    assert c.parse_status('{"live": true}') == {'live': True}
    assert c.parse_status('not json') is None
    assert c.parse_status('[1]') is None
    assert c.parse_status(None) is None


# ---- abort conditions during the run
def test_abort_none_when_healthy():
    assert c.abort_reason(3.0, st(state='active', mc_source='sdk_x', reason='driving'), 0.3, None) == ''


@pytest.mark.parametrize('kw', [dict(state='latched'), dict(latch='operator_takeover'), dict(reason='operator_takeover'),
                                dict(reason='localization_untrusted'), dict(reason='driver_not_armed')])
def test_abort_on_adapter_fault(kw):
    assert c.abort_reason(3.0, st(**{'state': 'active', 'mc_source': 'sdk_x', **kw}), 0.3, None)


def test_abort_on_status_loss_and_rc():
    assert c.abort_reason(3.0, None, None, None)
    assert c.abort_reason(3.0, st(state='active', mc_source='sdk_x'), c.RUN_STATUS_MAX_AGE_S + 0.01, None)
    assert c.abort_reason(3.0, st(state='active', mc_source='sdk_x'), 0.2, 0.1) == 'RC activity'


def test_mc_source_rc_allowed_only_during_acquisition_grace():
    s = st(state='active', mc_source='rc', reason='driving')
    assert c.abort_reason(0.5, s, 0.2, None) == ''
    assert c.abort_reason(2.0, s, 0.2, None)


def test_limits_constants_match_protocol():
    assert c.MAX_ABS_VX == 0.60 and c.MAX_DURATION_S == 12.0 and c.DEFAULT_SETTLE_S == 2.0
    assert math.isclose(c.NAV2_RATE_HZ, 20.0)
