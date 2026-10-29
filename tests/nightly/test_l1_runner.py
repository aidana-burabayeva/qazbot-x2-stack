import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_l1_negatives_rejected_and_yaw_only_reached(tmp_path):
    exe = tmp_path / 'l1'
    subprocess.run(['g++', '-O2', '-std=c++17', '-pthread', '-o', str(exe), str(ROOT / 'tools/nightly/l1/l1_runner.cpp')], check=True)
    sc = tmp_path / 's.jsonl'
    subprocess.run(['python3', str(ROOT / 'tools/nightly/scenario_gen.py'), '--seed', '4', '--per-category', '8', '--out', str(sc)], check=True, capture_output=True)
    out = tmp_path / 'r.tsv'
    subprocess.run([str(exe), str(ROOT / 'ws/src/maps/main_office'), str(sc.with_suffix('.tsv')), str(out), '2'], check=True)
    rows = [ln.split('\t') for ln in out.read_text().splitlines()]
    neg = [r for r in rows if r[1].startswith('NEG_')]
    assert neg and all(r[3] == 'REJECT' for r in neg)           # unknown/obstacle/island goals are never reached
    assert all(r[3] == 'REACHED' for r in rows if r[1] == 'yaw_only_far')
