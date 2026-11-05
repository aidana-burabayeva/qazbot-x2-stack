#!/usr/bin/env python3
"""Consolidate measured evidence without promoting a partial twin to physical qualification."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]

def read(path):
    return json.loads(path.read_text()) if path.exists() else None


def report(folder):
    comparisons=[read(p) for p in sorted(folder.glob('*comparison.json'))]
    checks=read(folder/'observation_final_check.json') or read(folder/'observation_check.json')
    replays={p.parent.name:read(p) for p in sorted((ROOT/'tests/integration_results').glob('replay_raw_lio_twin*/results.json'))}
    profile=ROOT/'tools/twin/profiles/robot66_observation_v1.json'
    return dict(status='PARTIAL: NOT QUALIFIED AS A PHYSICAL .66 TWIN',
                predictive_motion_tuning_supported=False,
                observation=dict(contract_passed=bool(checks and checks.get('passed')),profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
                                 remaining=['2D scene instead of real 3D projection','no interactive FAST-LIO or calibrated LIO drift','no measured sensor range noise']),
                body_comparisons=[{k:v for k,v in c.items() if k!='trace'} for c in comparisons],
                real_sensor_replays=replays,
                versions=dict(sim_mc='v1.0.0-20260522',robot_mc_snapshot='v1.1.0-r9-rc9',match=False),
                active_default_model='official; nominal-inertia candidate not promoted',
                next_required_evidence=['current per-unit calibration and model from .66','matched initial stance/history and repeated physical trials','external displacement/heading reference','matching x86 MC build, if vendor can provide it'],
                scope='Native body reference is odometry proxy; raw replay compares estimator reproducibility, not physical truth. No per-unit physics fitting was accepted.')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('folder',type=Path);a=ap.parse_args()
    r=report(a.folder);(a.folder/'fidelity_report.json').write_text(json.dumps(r,indent=2)+'\n')
    print(r['status'])
