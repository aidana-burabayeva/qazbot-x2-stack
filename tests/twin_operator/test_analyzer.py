"""analyze_session on synthetic evidence: only objective signatures become Nightly feedback."""
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/twin/operator'))
import analyze_session as A  # noqa: E402

MAP = ROOT / 'ws/src/maps/d1_qazbot_20261029/map.yaml'
T0 = 1_000_000.0


def write(tmp, events, trace):
    (tmp / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    (tmp / 'trace.jsonl').write_text('\n'.join(json.dumps(r) for r in trace))
    return tmp


def row(t, truth, v=0.0, cmd=(0, 0, 0), mc=(0, 0, 0), busy=True, ready=True):
    return dict(t=T0 + t, truth=list(truth), v=v, cmd_nav=list(cmd), cmd_auth=list(cmd), mc_cmd=list(mc), busy=busy, ready=ready, amcl=None)


def test_clean_success_has_no_feedback_and_measures_final_pose_after_settling(tmp_path):
    ev = [dict(t=T0, kind='GOAL', goal=[-8.4, -3.3, 0], truth=[-8.4, -1.2, 0], trust_ready=True),
          dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING'), dict(t=T0 + 5, kind='NAV_STATUS', status='SUCCEEDED')]
    tr = [row(i * 0.5, (-8.4, -1.2 - 0.2 * i, 0), v=0.4, cmd=(0.3, 0, 0)) for i in range(12)] + [row(6 + k, (-8.4, -3.3, 0), busy=False) for k in range(8)]
    rep = A.analyze(write(tmp_path, ev, tr), MAP)
    g = rep['goals'][0]
    assert g['nav2_outcome'] == 'SUCCEEDED' and g['reached_within_tolerance'] and rep['nightly_feedback'] == {}


def test_linear_and_angular_stalls_are_detected_and_become_feedback(tmp_path):
    ev = [dict(t=T0, kind='GOAL', goal=[-4.0, -3.0, 0], truth=[-8.4, -3.0, 0.0], trust_ready=True), dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING')]
    tr = [row(i, (-8.4, -3.0, 0.0), v=0.0, cmd=(0.0, 0, 0.5), mc=(0, 0, 0.5)) for i in range(2, 12)]      # asked to turn, does not
    rep = A.analyze(write(tmp_path, ev, tr), MAP)
    st = rep['goals'][0]['stalls']
    assert st and st[0]['kind'] == 'ANGULAR_NO_YAW' and st[0]['duration_s'] >= A.STALL_MIN_S
    assert rep['nightly_feedback']['component'] == 'navigation_stall'
    ev = [dict(t=T0, kind='GOAL', goal=[-4.0, -3.0, 0], truth=[-8.4, -3.0, 0.0], trust_ready=True), dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING')]
    tr = [row(i, (-8.4, -3.0, 0.0), v=0.0, cmd=(0.3, 0, 0.0), mc=(0.3, 0, 0)) for i in range(2, 12)]
    st = A.analyze(write(tmp_path, ev, tr), MAP)['goals'][0]['stalls']
    assert st and st[0]['kind'] == 'LINEAR_NO_MOTION'


def test_operator_cancel_and_idle_robot_are_not_defects(tmp_path):
    ev = [dict(t=T0, kind='GOAL', goal=[-4.0, -3.0, 0], truth=[-8.4, -3.0, 0.0], trust_ready=True), dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING'),
          dict(t=T0 + 3, kind='NAV_STATUS', status='CANCELED')]
    tr = [row(i, (-8.4, -3.0, 0.0), v=0.0) for i in range(0, 12)]            # no command: nothing to stall against
    rep = A.analyze(write(tmp_path, ev, tr), MAP)
    assert rep['nightly_feedback'] == {} and rep['goals'][0]['nav2_outcome'] == 'CANCELED'


def test_abort_for_goal_behind_a_door_narrower_than_the_robot_is_a_safe_reject_not_a_defect(tmp_path):
    # start west of the 0.805 m doorway, goal east of it: unreachable for a disc of radius 0.454 (needs 0.908 m)
    ev = [dict(t=T0, kind='GOAL', goal=[-4.7, -6.93, 0], truth=[-8.3, -6.93, 0.0], trust_ready=True), dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING'),
          dict(t=T0 + 60, kind='NAV_STATUS', status='ABORTED')]
    tr = [row(i, (-8.3, -6.93, 0.0), busy=True) for i in range(0, 60, 5)]
    rep = A.analyze(write(tmp_path, ev, tr), MAP)
    g = rep['goals'][0]
    assert g['nav2_outcome'] == 'ABORTED' and g['expected_reject_by_inflation'] is True and g['nav2_outcome_class'] == 'SAFE_REJECT'
    assert rep['nightly_feedback'] == {}


def test_abort_for_a_reachable_goal_is_a_defect(tmp_path):
    ev = [dict(t=T0, kind='GOAL', goal=[-8.4, -3.3, 0], truth=[-8.4, -1.2, 0.0], trust_ready=True), dict(t=T0 + 1, kind='NAV_STATUS', status='EXECUTING'),
          dict(t=T0 + 20, kind='NAV_STATUS', status='ABORTED')]
    tr = [row(i, (-8.4, -1.2, 0.0), busy=True) for i in range(0, 20, 5)]
    rep = A.analyze(write(tmp_path, ev, tr), MAP)
    assert rep['goals'][0]['expected_reject_by_inflation'] is False
    assert rep['nightly_feedback']['nav2_action_outcome'] == 'ABORTED'


def test_misfitting_seed_admitted_as_ready_is_reported_but_a_fitting_seed_is_not(tmp_path):
    bad = [dict(t=T0, kind='SEED', pose=[-9.5, -6.5, 0.0], truth=[-12.8, -4.4, -2.7]),
           dict(t=T0 + 1, kind='SEED_DIAG', verdict='DOES_NOT_FIT', seed_score=0.07, trust_hit_min_abs=0.35, text='x')]
    rep = A.analyze(write(tmp_path, bad, [row(0.5, (0, 0, 0), ready=True)]), MAP)
    assert rep['trust_admitted_misfitting_seed'] and rep['nightly_feedback']['component'] == 'localization_trust_admission'
    ok = [dict(t=T0, kind='SEED', pose=[-8.3, -1.2, 0.0], truth=[-8.3, -1.2, 0.0]),
          dict(t=T0 + 1, kind='SEED_DIAG', verdict='FITS', seed_score=0.5, trust_hit_min_abs=0.35, text='x')]
    assert A.analyze(write(tmp_path, ok, [row(0.5, (0, 0, 0), ready=True)]), MAP)['nightly_feedback'] == {}


def test_empty_evidence_does_not_crash(tmp_path):
    rep = A.analyze(write(tmp_path, [], []), MAP)
    assert rep['goals'] == [] and rep['nightly_feedback'] == {}
