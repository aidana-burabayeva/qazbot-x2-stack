"""NAV-OBS-01: only the GLOBAL inflation radius changes (0.60 -> 0.80). Everything else in the planner/costmap parameters stays."""
import importlib.util
import os
import sys
import tempfile

import pytest
import yaml

launch = pytest.importorskip('launch')
from launch.actions import DeclareLaunchArgument  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.abspath(os.path.join(HERE, '..', 'launch'))
MAP = os.path.abspath(os.path.join(HERE, '..', '..', 'qazbot_maps', 'maps', 'd1_qazbot_20261029', 'map.yaml'))

def _require_smac_overlay():
    """The patched Smac overlay is a REQUIRED dependency: absent -> hard failure (never a skip)."""
    from ament_index_python.packages import get_package_prefix
    prefix = os.environ.get('QAZBOT_SMAC_PREFIX') or get_package_prefix('nav2_smac_planner')
    assert os.path.isfile(os.path.join(prefix, 'qazbot_bounds_fix.json')), (
        f'REQUIRED qualified Smac overlay missing in {prefix!r}: run docker/tools/build_workspace.sh --prefix DIR '
        'and source DIR/install/setup.bash')


def planner_params():
    _require_smac_overlay()
    sys.path.insert(0, NAV)
    spec = importlib.util.spec_from_file_location('nf', os.path.join(NAV, 'nav_full.launch.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = launch.LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument) and e.default_value is not None:
            ctx.launch_configurations[e.name] = ''.join(s.perform(ctx) for s in e.default_value)
    ctx.launch_configurations.update(robot_radius='0.454', scan_topic='/qazbot_lio/scan',
                                     odom_topic='/qazbot_lio/odom_level', map_yaml=MAP)
    with tempfile.TemporaryDirectory() as d:
        os.environ['ROS_LOG_DIR'] = d
        m.setup(ctx)
        out = {}
        for f in os.listdir(d):
            if f.startswith(('qazbot_plan_', 'qazbot_ctl_')):
                with open(os.path.join(d, f)) as fh:
                    out.update(yaml.safe_load(fh))
    return out


def test_global_inflation_radius_changed_only():
    p = planner_params()
    g = p['/qazbot_d1_map/nav2/global_costmap/global_costmap']['ros__parameters']
    assert g['inflation_layer.inflation_radius'] == 0.80
    assert g['inflation_layer.cost_scaling_factor'] == 1.5
    assert g['inflation_layer.inflate_around_unknown'] is True
    s = p['/qazbot_d1_map/nav2/planner_server']['ros__parameters']
    assert s['GridBased.cost_travel_multiplier'] == 4.0 and s['GridBased.tolerance'] == 0.3 and s['GridBased.allow_unknown'] is False


def test_local_costmap_and_controller_untouched():
    p = planner_params()
    c = p['/qazbot_d1_map/nav2/controller_server']['ros__parameters']
    assert c['goal_checker.xy_goal_tolerance'] == 0.30 and c['goal_checker.yaw_goal_tolerance'] == 0.35 and c['goal_checker.stateful'] is True
    assert c['FollowPath.lookahead_dist'] == 0.5 and c['FollowPath.desired_linear_vel'] == 0.3
    loc = p['/qazbot_d1_map/nav2/local_costmap/local_costmap']['ros__parameters']
    assert loc['inflation_layer.inflation_radius'] == 0.50
