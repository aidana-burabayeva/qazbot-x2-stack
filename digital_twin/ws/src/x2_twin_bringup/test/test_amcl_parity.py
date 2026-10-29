"""The twin's map_server/AMCL parameters must equal production's (ws/src/navigation/localization.launch.py).

Only `scan_topic` (a LaunchConfiguration in the twin) and the map path may differ.
"""
import ast
import os
import pathlib

import pytest

HERE = pathlib.Path(__file__).resolve()
STACK = pathlib.Path(os.environ.get('TWIN_STACK_DIR', HERE.parents[5] / 'control_backend' / 'ws' / 'src'))
PROD = STACK / 'navigation' / 'localization.launch.py'
TWIN = HERE.parents[1] / 'launch' / 'amcl.launch.py'


def node_params(path, package):
    for call in ast.walk(ast.parse(pathlib.Path(path).read_text())):
        if isinstance(call, ast.Call) and getattr(call.func, 'id', '') == 'Node':
            kw = {k.arg: k.value for k in call.keywords}
            if isinstance(kw.get('package'), ast.Constant) and kw['package'].value == package:
                d = kw['parameters'].elts[0]
                return {k.value: (ast.literal_eval(v) if not isinstance(v, ast.Call) else '<launch-configuration>')
                        for k, v in zip(d.keys, d.values)}
    raise AssertionError(f'{package} not found in {path}')


@pytest.mark.skipif(not PROD.exists(), reason='stack source not available (set TWIN_STACK_DIR)')
@pytest.mark.parametrize('package,may_differ', [('nav2_amcl', {'scan_topic'}), ('nav2_map_server', {'yaml_filename'})])
def test_parameters_match_production(package, may_differ):
    prod, twin = node_params(PROD, package), node_params(TWIN, package)
    assert prod.keys() == twin.keys(), prod.keys() ^ twin.keys()
    diff = {k for k in prod if prod[k] != twin[k]}
    assert diff <= may_differ, {k: (prod[k], twin[k]) for k in diff}
