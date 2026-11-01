"""Real X2 episode 2026-10-30 16:05 (initialpose_episodes.json #0): +0.65 s after the operator seed AMCL moved 0.445 m / -14.8 deg
with the robot stationary. Fixture: seed, recorded AMCL pose and ONE real /qazbot_lio/scan from the recording
(twin_core_190402_0.mcap, sha256 in the fixture). Scores use the Trust node's own soft_score on the real map.
No simulator truth and no AMCL output used as truth: the question is only which pose the real scan fits.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'digital_twin/ws/src/x2_twin_scenarios'))
sys.path.insert(0, str(ROOT / 'ws/src/navigation'))
import loc_trust as L  # noqa: E402
from x2_twin_scenarios.analysis.replay_inputs import seed_refresh_events  # noqa: E402

FX = json.loads((ROOT / 'tests/localization/data/ep0_20261030_seed_scan_fixture.json').read_text())
CFG = L.TrustConfig()


def pts_and_grid():
    s = FX['scan']
    ranges = [float('nan') if r is None else r for r in s['ranges']]
    return (L.scan_endpoints(ranges, s['angle_min'], s['angle_increment'], s['range_min'], s['range_max']),
            L.OccGrid.from_pgm(str(ROOT / FX['source']['map'])))


def test_fixture_is_episode_zero_of_archive():
    # initialpose_episodes.json #0: seed (-8.3444, -1.2126, -2.1449) -> AMCL +0.65 s (-8.7296, -1.4361, -2.4028): 0.445 m / -14.8 deg
    d = math.hypot(FX['recorded_amcl_after'][0] - FX['seed'][0], FX['recorded_amcl_after'][1] - FX['seed'][1])
    assert abs(d - 0.445) < 0.005 and abs(FX['recorded_amcl_after_dt_s'] - 0.65) < 0.02
    assert abs(FX['seed'][0] + 8.344439506530762) < 1e-9


def test_operator_seed_does_not_fit_the_real_scan_and_amcl_correction_does():
    pts, grid = pts_and_grid()
    seed = L.soft_score(pts, grid, tuple(FX['seed']), CFG.score_sigma)
    after = L.soft_score(pts, grid, tuple(FX['recorded_amcl_after']), CFG.score_sigma)
    assert seed < 0.05 and after > 0.8, (seed, after)


def test_best_local_fit_around_seed_is_the_recorded_correction():
    pts, grid = pts_and_grid()
    amb = L.ambiguity(pts, grid, tuple(FX['seed']), CFG)
    dx, dy, dth = amb['best_offset']
    rec = [FX['recorded_amcl_after'][i] - FX['seed'][i] for i in range(3)]
    assert amb['best'] > 0.9 and amb['nominal'] < 0.05
    assert math.hypot(dx - rec[0], dy - rec[1]) < 0.2 and abs(dth - rec[2]) < 0.1


def events(seeds, scans):
    rows = [(t, 'scan', None) for t in scans] + [(t, 'initialpose', None) for t in seeds]
    return [t for t, k, _ in seed_refresh_events(sorted(rows, key=lambda r: r[0])) if k == 'nomotion']


def test_refresh_schedule_follows_trust_helper_on_real_scan_timeline():
    ev = events([FX['seed_t']], FX['scan_receipts'])
    assert 1 <= len(ev) <= 3 and ev[0] > FX['seed_t']
    assert all(b - a >= 0.6 for a, b in zip(ev, ev[1:])) and ev[-1] - FX['seed_t'] <= 6.0
    assert ev[0] - FX['seed_t'] < 0.5            # first fresh scan after the seed
    assert events([FX['seed_t']], [t for t in FX['scan_receipts'] if t < FX['seed_t']]) == []
    ev2 = events([FX['seed_t'], FX['seed_t'] + 1.0], FX['scan_receipts'])
    assert [t for t in ev2 if FX['seed_t'] < t <= FX['seed_t'] + 1.0] and len([t for t in ev2 if t > FX['seed_t'] + 1.0]) <= 3
