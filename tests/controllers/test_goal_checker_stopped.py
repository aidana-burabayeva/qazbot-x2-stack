"""Launch-parameter test: goal_checker_stopped switches the live goal checker; default behaviour is unchanged."""
import importlib.util
import os
import sys
import tempfile

import pytest
import yaml

launch = pytest.importorskip('launch')
HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.join(HERE, '..', '..', 'ws', 'src', 'navigation')


def controller_params(extra):
    sys.path.insert(0, NAV)
    spec = importlib.util.spec_from_file_location('qazbot_control_launch', os.path.join(NAV, 'nav_control.launch.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    ctx = launch.LaunchContext()
    cfg = {'ns': 'qazbot_d1_map', 'robot_radius': '0.454', 'max_linear': '0.3', 'max_angular': '0.6', 'xy_tol': '0.10',
           'yaw_tol': '0.10', 'goal_checker_stateful': 'false', 'local_frame': 'odom', 'scan_topic': '',
           'output_topic': 'cmd_vel', 'odom_topic': '/qazbot_lio/odom_level',
           'trans_stopped_velocity': '0.08', 'rot_stopped_velocity': '0.05'}
    cfg.update(extra)
    ctx.launch_configurations.update(cfg)
    with tempfile.TemporaryDirectory() as d:
        os.environ['ROS_LOG_DIR'] = d
        m.setup(ctx, raw=True)
        files = [f for f in os.listdir(d) if f.startswith('qazbot_ctl_')]
        assert files
        with open(os.path.join(d, files[0])) as f:
            return yaml.safe_load(f)['/qazbot_d1_map/controller_server']['ros__parameters']


def test_default_is_simple_goal_checker_unchanged():
    p = controller_params({})
    assert p['goal_checker.plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert 'goal_checker.trans_stopped_velocity' not in p
    assert p['goal_checker.xy_goal_tolerance'] == 0.10 and p['goal_checker.yaw_goal_tolerance'] == 0.10


def test_stopped_goal_checker_when_requested():
    p = controller_params({'goal_checker_stopped': 'true'})
    assert p['goal_checker.plugin'] == 'nav2_controller::StoppedGoalChecker'
    assert p['goal_checker.trans_stopped_velocity'] == 0.08
    assert p['goal_checker.rot_stopped_velocity'] == 0.05
    assert p['goal_checker.xy_goal_tolerance'] == 0.10  # tolerances are untouched by this switch


def test_invalid_value_rejected():
    with pytest.raises(ValueError):
        controller_params({'goal_checker_stopped': 'maybe'})
