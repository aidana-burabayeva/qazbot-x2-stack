"""Unit tests of the D-05 Nav2-goal calibration logic. No ROS, no sleeps, no wall clock."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools', 'd05', 'nav2_calib'))
import calib_core as c  # noqa: E402

# live values 2026-11-02/03 (~/.ros/log/qazbot_ctl_t9htg14z.yaml)
LIVE = c.LiveParams(desired_linear_vel=0.3, smoother_max_vx=0.3, smoother_accel_x=0.4, smoother_decel_x=-0.6,
                    approach_dist=0.6, min_approach_vel=0.12, xy_goal_tolerance=0.35)
GC_OK = {'busy': False, 'latched': False, 'fault': '', 'ready': True}
AD_OK = {'live': True, 'registration': 'registered', 'latch': '', 'foreign_publishers': []}


def live(**kw):
    d = LIVE.__dict__.copy()
    d.update(kw)
    return c.LiveParams(**d)


# ---- speed limit plan
@pytest.mark.parametrize('v,pct', [(0.30, 100.0), (0.15, 50.0), (0.10, 100 / 3), (0.05, 100 / 6), (0.25, 250 / 3)])
def test_plan_percentage(v, pct):
    p = c.plan_speed_limit(v, LIVE)
    assert p.percentage == pytest.approx(pct)
    assert p.expected_cmd_vx == pytest.approx(v)
    assert p.percentage <= 100.0


@pytest.mark.parametrize('v', [0.3000001, 0.31, 0.40, 0.60, 0.049, 0.0, -0.1, float('nan'), float('inf'), None, '0.2'])
def test_plan_refuses(v):
    with pytest.raises(c.CalibError):
        c.plan_speed_limit(v, LIVE)


def test_envelope_is_min_of_rpp_and_smoother():
    p = live(desired_linear_vel=0.5, smoother_max_vx=0.3)
    assert c.envelope(p) == 0.3
    with pytest.raises(c.CalibError):
        c.plan_speed_limit(0.35, p)
    assert c.plan_speed_limit(0.3, p).percentage == pytest.approx(60.0)


def test_plan_exact_envelope_rounding_capped_at_100():
    assert c.plan_speed_limit(0.3 + 5e-10, LIVE).percentage == 100.0


@pytest.mark.parametrize('kw', [dict(desired_linear_vel=0.0), dict(smoother_max_vx=-1.0), dict(smoother_accel_x=0.0),
                                dict(smoother_decel_x=0.6), dict(approach_dist=float('nan')),
                                dict(xy_goal_tolerance=None)])
def test_live_invalid(kw):
    with pytest.raises(c.CalibError):
        c.plan_speed_limit(0.2, live(**kw))


# ---- estimates
def test_approach_affects():
    assert c.approach_affects(0.30, LIVE)
    assert not c.approach_affects(0.12, LIVE)
    assert not c.approach_affects(0.10, LIVE)


def test_segment_estimate_anchor():
    e = c.segment_estimate(0.30, 4.0, LIVE, gain=1.61)
    assert e['ramp_up_s'] == pytest.approx(0.75)
    assert e['ramp_down_s'] == pytest.approx(0.5)
    assert e['ramp_up_m'] == pytest.approx(0.5 * 0.483 * 0.75)
    assert e['tail_m'] == 0.6
    assert e['steady_s'] == pytest.approx((4.0 - 0.5 * 0.483 * 0.75 - 0.6) / 0.483)
    assert e['steady_s'] > 6.0


def test_segment_estimate_low_speed_uses_goal_tolerance_tail_and_clamps():
    e = c.segment_estimate(0.10, 4.0, LIVE)
    assert e['tail_m'] == 0.35
    assert c.segment_estimate(0.30, 0.5, LIVE)['steady_s'] == 0.0


@pytest.mark.parametrize('args', [(0.0, 4.0, 1.0), (0.3, 0.0, 1.0), (0.3, 4.0, 0.0), (float('nan'), 4.0, 1.0)])
def test_segment_estimate_invalid(args):
    with pytest.raises(c.CalibError):
        c.segment_estimate(args[0], args[1], LIVE, args[2])


def test_min_path_roundtrip():
    d = c.min_path_for_steady(0.30, 6.0, LIVE, 1.61)
    assert c.segment_estimate(0.30, d, LIVE, 1.61)['steady_s'] == pytest.approx(6.0)


# ---- preflight
def test_preflight_ok():
    assert c.preflight(GC_OK, 0.1, AD_OK, 0.3, 1, 0) == []


@pytest.mark.parametrize('gc,gc_age,ad,ad_age,subs,foreign,key', [
    (None, None, AD_OK, 0.1, 1, 0, 'goal_coordinator status'),
    (GC_OK, 1.5, AD_OK, 0.1, 1, 0, 'goal_coordinator status'),
    (dict(GC_OK, busy=True), 0.1, AD_OK, 0.1, 1, 0, 'busy'),
    (dict(GC_OK, latched=True, fault='operator_stop'), 0.1, AD_OK, 0.1, 1, 0, 'latched'),
    (dict(GC_OK, ready=False), 0.1, AD_OK, 0.1, 1, 0, 'not ready'),
    (GC_OK, 0.1, None, None, 1, 0, 'motion_adapter status'),
    (GC_OK, 0.1, AD_OK, 2.0, 1, 0, 'motion_adapter status'),
    (GC_OK, 0.1, dict(AD_OK, live=False), 0.1, 1, 0, 'live+registered'),
    (GC_OK, 0.1, dict(AD_OK, registration='dry_run'), 0.1, 1, 0, 'live+registered'),
    (GC_OK, 0.1, dict(AD_OK, latch='operator_takeover'), 0.1, 1, 0, 'adapter latched'),
    (GC_OK, 0.1, dict(AD_OK, foreign_publishers=['x']), 0.1, 1, 0, 'foreign MC'),
    (GC_OK, 0.1, AD_OK, 0.1, 0, 0, 'no subscriber'),
    (GC_OK, 0.1, AD_OK, 0.1, 1, 1, 'another publisher'),
])
def test_preflight_refusals(gc, gc_age, ad, ad_age, subs, foreign, key):
    why = c.preflight(gc, gc_age, ad, ad_age, subs, foreign)
    assert len(why) == 1 and key in why[0]


# ---- goal tracker
def test_tracker_full_cycle():
    t = c.GoalTracker(post_roll_s=4.0)
    assert t.update(0.0, GC_OK) == 'WAIT'
    assert t.update(1.0, dict(GC_OK, busy=True)) == 'ACTIVE' and t.t_start == 1.0
    assert t.update(5.0, dict(GC_OK, busy=True)) == 'ACTIVE'
    assert t.update(9.0, GC_OK) == 'POST' and t.t_end == 9.0
    assert t.update(12.9, dict(GC_OK, busy=True)) == 'POST'   # a second goal does not restart the run
    assert t.update(13.0, GC_OK) == 'DONE'
    assert t.fault_end == '' and not t.latched_end


def test_tracker_records_fault():
    t = c.GoalTracker(post_roll_s=0.0)
    t.update(0.0, dict(GC_OK, busy=True))
    assert t.update(1.0, dict(GC_OK, latched=True, fault='operator_takeover')) == 'DONE'
    assert t.fault_end == 'operator_takeover' and t.latched_end


# ---- command analysis
def ramp(v, acc=0.4, dec=-0.6, hold_s=5.0, dt=0.05):
    s, t, x = [], 0.0, 0.0
    while x < v:
        s.append((t, x))
        t += dt
        x = min(v, x + acc * dt)
    end = t + hold_s
    while t < end:
        s.append((t, v))
        t += dt
    while x > 0:
        x = max(0.0, x + dec * dt)
        s.append((t, x))
        t += dt
    return s


def test_honored_and_steady():
    s = ramp(0.15)
    assert c.honored(s, 0.15)
    assert not c.honored(s + [(99.0, 0.153)], 0.15)
    d, t0, t1 = c.longest_steady(s, 0.15)
    assert d == pytest.approx(5.0, abs=0.06) and t1 > t0


def test_longest_steady_picks_longest_and_empty():
    s = [(0, .3), (1, .3), (2, .1), (3, .3), (4, .3), (5, .3)]
    assert c.longest_steady(s, 0.3) == (2, 3, 5)
    assert c.longest_steady([], 0.3) == (0.0, None, None)


def test_ramp_stats_and_within():
    r = c.ramp_stats(ramp(0.30))
    assert r['max_accel'] == pytest.approx(0.4, rel=1e-6)
    assert r['max_decel'] == pytest.approx(-0.6, rel=1e-6)
    assert c.ramp_within(r, LIVE)
    assert not c.ramp_within({'max_accel': 0.9, 'max_decel': -0.6}, LIVE)
    assert not c.ramp_within({'max_accel': 0.4, 'max_decel': -1.2}, LIVE)
    assert c.ramp_stats([(0, 0), (0, 1)]) == {'max_accel': 0.0, 'max_decel': 0.0}   # dt 0 ignored


def test_summary_dict():
    plan = c.plan_speed_limit(0.15, LIVE)
    s = c.RunSummary(0.15, plan, nav_samples=[(0, .15), (1, .15)], smooth_samples=ramp(0.15))
    d = s.to_dict(LIVE)
    assert d['limit_honored'] and d['ramp_within_limits']
    assert d['nav_cmd_max_vx'] == .15 and math.isclose(d['smoothed_cmd_max_vx'], .15)
    assert d['steady_cmd_s'] > 4.9
    empty = c.RunSummary(0.15, plan).to_dict(LIVE)
    assert not empty['limit_honored'] and not empty['ramp_within_limits'] and empty['nav_cmd_max_vx'] is None
