"""The AMCL used by tools/twin/replay_amcl.py must carry the PRODUCTION parameters (ws/src/navigation/localization.launch.py).

Found on real X2 episode 2026-10-30 16:05: the replay launch only set max_beams=1440 and used Nav2 defaults for
alpha/recovery/update thresholds, so 'real AMCL' replays were not the production filter.
"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def amcl_params(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node'
                and any(k.arg == 'executable' and getattr(k.value, 'value', None) == 'amcl' for k in node.keywords)):
            params = next(k.value for k in node.keywords if k.arg == 'parameters')
            merged = {}
            for d in params.elts:
                for k, v in zip(d.keys, d.values):
                    try:
                        merged[k.value] = ast.literal_eval(v)
                    except ValueError:
                        pass                      # LaunchConfiguration(...) placeholders
            return merged
    raise AssertionError('no amcl node in ' + str(path))


def test_replay_amcl_launch_has_production_parameters():
    prod = amcl_params(ROOT / 'ws/src/navigation/localization.launch.py')
    sim = amcl_params(ROOT / 'tools/twin/amcl_sim.launch.py')
    for key in ('max_beams', 'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5', 'recovery_alpha_slow',
                'recovery_alpha_fast', 'update_min_d', 'update_min_a', 'robot_model_type'):
        assert key in prod, key
        assert sim.get(key) == prod[key], (key, sim.get(key), prod[key])
