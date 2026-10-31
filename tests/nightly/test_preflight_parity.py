"""P0 parity contract: exact image edits and fail-closed per-message TF ownership."""
import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/nightly'))
import preflight

spec = importlib.util.spec_from_file_location('nav_patch', ROOT / 'digital_twin/docker/nav_control_patch.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


def test_patch_changes_only_experimental_turn_with_two_localhost_guards():
    other = "    untouched = {'ROS_LOCALHOST_ONLY':'1'}\n"
    source = other + patch.TURN_GUARD + '\n' + patch.INF_LITERAL + '\n'
    assert source.count("'ROS_LOCALHOST_ONLY':'1'") == 2
    changed = patch.transform(source)
    assert changed.startswith(other)
    assert changed.count("'ROS_LOCALHOST_ONLY':'1'") == 1
    assert changed == other + patch.TURN_GUARD.replace("'ROS_LOCALHOST_ONLY':'1'", "'TWIN_SANDBOX':'1'") + '\n' + "'obstacle_layer.scan.inf_is_valid': os.environ.get('QAZBOT_I1_INF_VALID') == '1',\n"


@pytest.mark.parametrize('source', ['', patch.TURN_GUARD, patch.TURN_GUARD * 2 + patch.INF_LITERAL])
def test_patch_refuses_changed_or_duplicate_contract(source):
    with pytest.raises(ValueError):
        patch.transform(source)


def test_actual_source_transform_and_docker_use_same_module():
    source = (ROOT / 'ws/src/navigation/nav_control.launch.py').read_text()
    result = patch.transform(source)
    assert result != source
    docker = (ROOT / 'digital_twin/docker/Dockerfile').read_text()
    assert 'RUN python3 /opt/x2twin/nav_control_patch.py /opt/x2twin/stack/navigation/nav_control.launch.py' in docker
    assert '"\'ROS_LOCALHOST_ONLY\':\'1\'"' not in docker


def observation():
    return {'pass': True, 'owners': {
        'map->odom': [{'gid': 'a' * 48, 'samples': 8, 'nodes': ['/qazbot_d1_map/amcl']}],
        'odom->base_footprint': [{'gid': 'b' * 48, 'samples': 8, 'nodes': ['/kinematic_plant']}],
    }}


def test_nominal_and_official_odom_owner():
    doc = observation()
    assert preflight.ownership_report_ok(doc)
    doc['owners']['odom->base_footprint'][0]['nodes'] = ['/hal_plant']
    assert not preflight.ownership_report_ok(doc)
    assert preflight.ownership_report_ok(doc, '/hal_plant')


@pytest.mark.parametrize('fault', ['duplicate', 'same-name', 'unknown', 'missing', 'zero-gid', 'insufficient', 'false-pass'])
def test_ownership_never_passes_without_single_measured_resolved_gid(fault):
    d = observation()
    rows = d['owners']['map->odom']
    if fault in ('duplicate', 'same-name'):
        rows.append({'gid': 'c' * 48, 'samples': 8, 'nodes': ['/qazbot_d1_map/amcl' if fault == 'same-name' else '/intruder']})
    elif fault == 'unknown': rows[0]['nodes'] = []
    elif fault == 'missing': rows.clear()
    elif fault == 'zero-gid': rows[0]['gid'] = '0' * 48
    elif fault == 'insufficient': rows[0]['samples'] = 2
    else: d['pass'] = False
    assert not preflight.ownership_report_ok(d)


def test_native_core_counts_gids_instead_of_node_names(tmp_path):
    source = tmp_path / 'ownership_test.cpp'
    source.write_text(r'''
#include "ownership.hpp"
#include <iostream>
int main() {
  using namespace tf_probe;
  const std::map<std::string,std::string> expected{{"map->odom","/amcl"},{"odom->base_footprint","/plant"}};
  Samples s{{"map->odom",{{"aaa",4}}},{"odom->base_footprint",{{"bbb",4}}}};
  std::map<std::string,std::set<std::string>> names{{"aaa",{"/amcl"}},{"bbb",{"/plant"}}};
  if (!valid(s,names,expected)) return 1;
  s["map->odom"]["ccc"]=4; names["ccc"]={"/amcl"};
  if (valid(s,names,expected)) return 2;
  s["map->odom"].erase("ccc"); s["map->odom"]["aaa"]=2;
  if (valid(s,names,expected)) return 3;
  s["map->odom"]["aaa"]=4; names.erase("aaa");
  if (valid(s,names,expected)) return 4;
  if (valid({},names,expected)) return 5;
  std::cout << "native ownership core PASS\n";
}
''')
    exe = tmp_path / 'ownership_test'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'digital_twin/ws/src/qazbot_tf_probe/src'), str(source), '-o', str(exe)], check=True)
    assert subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.strip() == 'native ownership core PASS'
