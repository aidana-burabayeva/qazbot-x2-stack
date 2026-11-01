"""T5 in AMCL replay summaries must follow the Trust node, on a REAL X2 recording.

Fixture: tests/localization/data/goal2_1216_t5_fixture.json, extracted from the real bag RViz_goal2_1216
(2026-10-27, stationary robot, two operator seeds 7 s apart, recorded Trust status with zero T5 reasons).
The old summary counted raw map->odom translation steps, including the two intentional reseeds, as T5 trips.
"""
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'digital_twin/ws/src/x2_twin_scenarios'))
sys.path.insert(0, str(ROOT / 'ws/src/navigation'))
from x2_twin_scenarios.analysis.replay_inputs import t5_steps  # noqa: E402
from loc_trust import TrustConfig, TrustEvaluator  # noqa: E402

FX = json.loads((ROOT / 'tests/localization/data/goal2_1216_t5_fixture.json').read_text())
CFG = TrustConfig()


def seq():
    return [(m[0], tuple(m[1:])) for m in FX['map_odom']]


def odom_at(t):
    return min(FX['odom_base_at_map_odom'], key=lambda r: abs(r[0] - t))[1:]


def trips(steps):
    return [s for s in steps if s[1] > CFG.jump_xy_max or s[2] > CFG.jump_yaw_max]


def old_raw_steps(s):
    wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
    return [(tb, math.hypot(b[0] - a[0], b[1] - a[1]), abs(wrap(b[2] - a[2]))) for (ta, a), (tb, b) in zip(s, s[1:])]


def test_real_recording_legacy_metric_reports_false_trips_trust_reported_none():
    assert FX['recorded_trust_T5_reason_messages'] == 0
    assert len(trips(old_raw_steps(seq()))) == 2          # the 3.5 m reseed steps


def test_real_recording_new_metric_matches_trust_node():
    steps = t5_steps(seq(), odom_at, FX['initialpose_t'], TrustEvaluator.correction_at)
    assert len(trips(steps)) == FX['recorded_trust_T5_reason_messages'] == 0
    assert max(s[1] for s in steps) < CFG.jump_xy_max


def test_unseeded_real_correction_is_still_a_trip():
    s = seq()
    t, p = s[-1]
    s.append((t + 5, (p[0] + 0.5, p[1], p[2])))           # 0.5 m AMCL correction, no operator seed
    assert len(trips(t5_steps(s, odom_at, FX['initialpose_t'], TrustEvaluator.correction_at))) == 1


def test_only_first_update_after_a_seed_starts_the_epoch():
    s = seq()
    i = [k for k, (t, _) in enumerate(s) if t > 114.924][0]
    t, p = s[i + 1]
    s[i + 1] = (t, (p[0] + 0.6, p[1], p[2]))              # second update after the seed jumps
    assert len(trips(t5_steps(s, odom_at, FX['initialpose_t'], TrustEvaluator.correction_at))) >= 1


def test_metric_is_robot_displacement_not_transform_translation():
    # rotate map->odom about the robot's current odom position: transform translation moves a lot, robot does not
    odom = (3.0, 0.0, 0.0)
    prev = (0.0, 0.0, 0.0)
    th = 0.05
    c, s_ = math.cos(th), math.sin(th)
    new = (3.0 - (c * 3.0), -(s_ * 3.0), th)
    x, y, _ = TrustEvaluator.correction_at(prev, new, odom), 0, 0
    assert x[0] < 1e-9
    steps = t5_steps([(0, prev), (1, new)], lambda t: odom, [], TrustEvaluator.correction_at)
    assert steps[0][1] < 1e-9
