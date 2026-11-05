"""Production goal-checker profile (operator decision 2026-11-05): SimpleGoalChecker, xy 0.30 m, yaw 0.35 rad, stateful.
Offline check of every launch path that builds the controller_server parameters: declared defaults, fallback when the
argument is missing (so a reboot/relaunch cannot restore 0.10/0.10/false) and the generated parameter YAML."""
import importlib.util
import os
import sys
import tempfile

import pytest
import yaml

launch = pytest.importorskip('launch')
from launch.actions import DeclareLaunchArgument  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.abspath(os.path.join(HERE, '..', '..', 'ws', 'src', 'navigation'))
MAP = os.path.abspath(os.path.join(HERE, '..', '..', 'ws', 'src', 'maps', 'd1_qazbot_20261029', 'map.yaml'))


def load(name):
    sys.path.insert(0, NAV)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), os.path.join(NAV, name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def declared_defaults(module):
    ctx = launch.LaunchContext()
    out = {}
    for e in module.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument) and e.default_value is not None:
            out[e.name] = ''.join(s.perform(ctx) for s in e.default_value)
    return out


@pytest.mark.parametrize('name', ['nav_full.launch.py', 'nav_control.launch.py'])
def test_declared_defaults(name):
    d = declared_defaults(load(name))
    assert d['xy_tol'] == '0.30' and d['yaw_tol'] == '0.35' and d['goal_checker_stateful'] == 'true'
    assert 'goal_checker_stopped' not in d  # the StoppedGoalChecker switch is not part of the production profile


def generated_params(module, entry, extra):
    ctx = launch.LaunchContext()
    cfg = declared_defaults(module)
    cfg.update({'robot_radius': '0.454', 'scan_topic': '/qazbot_lio/scan', 'odom_topic': '/qazbot_lio/odom_level',
                'map_yaml': MAP})
    cfg.update(extra)
    ctx.launch_configurations.update(cfg)
    with tempfile.TemporaryDirectory() as d:
        os.environ['ROS_LOG_DIR'] = d
        for k in [k for k in os.environ if k.startswith('QAZBOT_SIM')]:
            del os.environ[k]
        entry(ctx)
        files = sorted(f for f in os.listdir(d) if f.startswith('qazbot_ctl_'))
        assert files, 'controller parameter YAML was not generated'
        with open(os.path.join(d, files[-1])) as f:
            data = yaml.safe_load(f)
    key = [k for k in data if k.endswith('controller_server')][0]
    return data[key]['ros__parameters']


def check_profile(p):
    assert p['goal_checker.plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert p['goal_checker.xy_goal_tolerance'] == 0.30
    assert p['goal_checker.yaw_goal_tolerance'] == 0.35
    assert p['goal_checker.stateful'] is True
    assert 'goal_checker.trans_stopped_velocity' not in p


def test_nav_full_generated_parameters():
    if not os.environ.get('QAZBOT_SMAC_PREFIX'):
        pytest.skip('needs the robot environment (source ws/src/navigation/env.sh: QAZBOT_SMAC_PREFIX)')
    m = load('nav_full.launch.py')
    check_profile(generated_params(m, m.setup, {}))


def test_nav_control_generated_parameters_with_declared_defaults():
    m = load('nav_control.launch.py')
    check_profile(generated_params(m, lambda c: m.setup(c, raw=True), {}))


def test_missing_stateful_argument_falls_back_to_true():
    m = load('nav_control.launch.py')
    ctx = launch.LaunchContext()
    cfg = declared_defaults(m)
    cfg.update({'robot_radius': '0.454', 'odom_topic': '/qazbot_lio/odom_level'})
    del cfg['goal_checker_stateful']
    ctx.launch_configurations.update(cfg)
    with tempfile.TemporaryDirectory() as d:
        os.environ['ROS_LOG_DIR'] = d
        m.setup(ctx, raw=True)
        f = [x for x in os.listdir(d) if x.startswith('qazbot_ctl_')][0]
        with open(os.path.join(d, f)) as fh:
            p = yaml.safe_load(fh)['/qazbot_d1_map/controller_server']['ros__parameters']
    assert p['goal_checker.stateful'] is True


def test_rpp_smoother_and_speed_parameters_unchanged_vs_baseline():
    """Only the goal checker may differ from the previous profile: compare with the base commit's generator output."""
    m = load('nav_control.launch.py')
    p = generated_params(m, lambda c: m.setup(c, raw=True), {})
    expect = {'FollowPath.desired_linear_vel': 0.3, 'FollowPath.rotate_to_heading_min_angle': 0.6,
              'FollowPath.rotate_to_heading_angular_vel': 0.6, 'FollowPath.lookahead_dist': 0.5,
              'FollowPath.approach_velocity_scaling_dist': 0.6, 'FollowPath.min_approach_linear_velocity': 0.12,
              'FollowPath.allow_reversing': False, 'FollowPath.use_rotate_to_heading': True,
              'progress_checker.required_movement_radius': 0.1, 'progress_checker.movement_time_allowance': 15.0}
    for k, v in expect.items():
        assert p[k] == v, k
