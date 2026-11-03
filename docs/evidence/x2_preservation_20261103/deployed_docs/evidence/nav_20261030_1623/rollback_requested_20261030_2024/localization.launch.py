"""X2 independent LIO + map_server + AMCL. No command publisher or MC calls.

odom_source (reports/20261027-native-odom-nav2-design.md 3.6) selects the ONE owner of TF odom->base_footprint:
  lio (default)  level_bridge (FAST-LIO pose), exactly as before; the native bridge is not started.
  native_imu     native_odom_bridge (MC /aima/mc/leg_odometry, same base_footprint = LiDAR-IMU footprint);
                 level_bridge runs with publish_tf:=false and still publishes /qazbot_lio/scan and
                 /qazbot_lio/odom_level unchanged. Consumers of the odometry topic must then use /qazbot_native/odom
                 (run_nav_full.sh QAZBOT_ODOM_TOPIC, trust -p odom_topic; tools/stack/stack_up.sh does both).
  Anything else (native_base: step 2, scan not yet in a base_link frame) is refused.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch_stagger import staggered  # noqa: E402
import pose_store  # noqa: E402
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch import Substitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


class TasksetPrefix(Substitution):
    """Opt-in CPU pinning prefix for FAST-LIO: '' -> no prefix, '5,6' / '5-6' -> 'taskset -c 5,6'.
    Unprivileged (sched_setaffinity on our own process). Measured on X2 .66 2026-10-22 (liolat report):
    CPUs 5-6 are the least loaded cores (no vendor pinning, not the IRQ core 0)."""

    def __init__(self, cpus):
        super().__init__()
        self.cpus = cpus

    def perform(self, context):
        cpus = perform_substitutions(context, [self.cpus]).strip()
        if not cpus:
            return ''
        if not re.fullmatch(r'[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*', cpus):
            raise ValueError('lio_cpus must be a CPU list like "5,6" or "5-6", got ' + repr(cpus))
        return 'taskset -c ' + cpus


ODOM_SOURCES = ('lio', 'native_imu')


class LevelBridgePublishTf(Substitution):
    """'true' when level_bridge owns odom->base_footprint (odom_source lio), 'false' for native_imu; refuses any
    other source so a typo can never start zero or two TF owners."""

    def __init__(self, source):
        super().__init__()
        self.source = source

    def perform(self, context):
        source = perform_substitutions(context, [self.source]).strip()
        if source not in ODOM_SOURCES:
            raise ValueError(f'odom_source must be one of {ODOM_SOURCES}, got {source!r}')
        return 'true' if source == 'lio' else 'false'


def generate_launch_description():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # navigation/ lives in control_backend/ or in control_backend/ws/src/; runtime/ stays in control_backend/
    backend = next((p for p in (root, os.path.dirname(os.path.dirname(root)))
                    if os.path.isdir(os.path.join(p, 'runtime'))), root)
    ns = 'qazbot_d1_map'
    tf_remaps = [('/tf', '/qazbot_d1_map/tf'), ('/tf_static', '/qazbot_d1_map/tf_static')]
    lio_remaps = [('/Odometry', '/qazbot_lio/odometry'),
                  ('/cloud_registered', '/qazbot_lio/cloud_registered'),
                  ('/cloud_registered_body', '/qazbot_lio/cloud_registered_body'),
                  ('/cloud_effected', '/qazbot_lio/cloud_effected'),
                  ('/Laser_map', '/qazbot_lio/map'), ('/path', '/qazbot_lio/path'),
                  ('/tf', '/qazbot_lio/tf'), ('map_save', '/qazbot_lio/map_save')]
    lio_args = ['--ros-args', '-r', '__node:=qazbot_lio_mapping', '--params-file',
                os.path.join(root, 'lio', 'x2.yaml')]
    for source, target in lio_remaps:
        lio_args += ['-r', source + ':=' + target]
    actions = [
        DeclareLaunchArgument('lio_binary', default_value=os.path.join(backend,'runtime','fastlio_mapping')),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(root, 'maps', 'first_map_nav2_v2', 'map.yaml')),
        # Empty lio_cpus (default) leaves scheduling unchanged.
        DeclareLaunchArgument('lio_cpus', default_value=''),
        DeclareLaunchArgument('seed_pose_file', default_value=pose_store.DEFAULT_PATH),
        DeclareLaunchArgument('odom_source', default_value='lio', choices=list(ODOM_SOURCES)),
        DeclareLaunchArgument('native_rate_hz', default_value='30.0'),
        # tests only (isolated domain): feed recorded leg odometry without publishing on /aima/*
        DeclareLaunchArgument('native_leg_topic', default_value='/aima/mc/leg_odometry'),
        ExecuteProcess(cmd=[LaunchConfiguration('lio_binary')] + lio_args, output='screen',
                       prefix=TasksetPrefix(LaunchConfiguration('lio_cpus'))),
        ExecuteProcess(cmd=['python3', os.path.join(root, 'lio', 'level_bridge.py'), '--ros-args',
                            '-r', '__node:=qazbot_level_bridge',
                            '-r', '/Odometry:=/qazbot_lio/odometry',
                            '-r', '/cloud_registered:=/qazbot_lio/cloud_registered',
                            '-r', '/scan:=/qazbot_lio/scan',
                            '-r', '/odom_level:=/qazbot_lio/odom_level',
                            '-r', '/tf:=/qazbot_d1_map/tf',
                            '-r', '/tf_static:=/qazbot_d1_map/tf_static',
                            '-p', 'range_max:=8.0', '-p', 'scan_every:=2',
                            '-p', ['publish_tf:=', LevelBridgePublishTf(LaunchConfiguration('odom_source'))]],
                       output='screen'),
        Node(package='nav2_map_server', executable='map_server', namespace=ns,
             name='map_server', output='screen', remappings=tf_remaps,
             parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'), 'topic_name': 'map',
                          'frame_id': 'map', 'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', namespace=ns, name='amcl',
             output='screen', remappings=tf_remaps,
             parameters=[{'use_sim_time': False, 'global_frame_id': 'map', 'odom_frame_id': 'odom',
                          'base_frame_id': 'base_footprint', 'map_topic': 'map',
                          # odometry trusted more than laser per operator 2026-10-28; tuning later
                          'scan_topic': '/qazbot_lio/scan', 'max_beams': 180,
                          'alpha1': 0.3, 'alpha2': 0.2, 'alpha3': 0.05, 'alpha4': 0.15, 'alpha5': 0.15,  # 2026-10-30: map now good; walking gait yaw noise
                          'recovery_alpha_slow': 0.0, 'recovery_alpha_fast': 0.0,
                          # P12 operator 2026-10-28: smaller, more frequent corrections (default 0.25 m / 0.2 rad)
                          # P26 2026-10-30 (independent review Q1): 0.10/0.05 fed correlated scans (overconfidence)
                          'update_min_d': 0.20, 'update_min_a': 0.15,
                          # P26: beam-skip (only effective with likelihood_field_prob) drops beams most particles disagree
                          # with (unmapped chairs, people), capped at 90 % of beams; resample every 2nd update (depletion)
                          'laser_model_type': 'likelihood_field_prob', 'do_beamskip': True,
                          'beam_skip_distance': 0.5, 'beam_skip_threshold': 0.3, 'beam_skip_error_threshold': 0.9,
                          'resample_interval': 2,
                          'robot_model_type': 'nav2_amcl::OmniMotionModel'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ]
    # Re-seed the last CONFIRMED pose (written by the trust node, param pose_store) ONCE as initialpose with its
    # saved covariance. Missing / corrupt / other-map file: nothing is published. Never grants READY: the trust
    # node needs confirm_anchor + a clean gate on it. seed_pose_file:='' disables.
    seed = ExecuteProcess(cmd=['python3', os.path.join(root, 'navigation', 'seed_pose.py'),
                               '--from-file', LaunchConfiguration('seed_pose_file'),
                               '--map-yaml', LaunchConfiguration('map_yaml')], output='screen',
                          condition=LaunchConfigurationNotEquals('seed_pose_file', ''))
    # native_imu only: the odom->base_footprint owner. Started last so the lio-mode start times are unchanged.
    native = ExecuteProcess(cmd=['python3', os.path.join(root, 'lio', 'native_odom_bridge.py'), '--ros-args',
                                 '-r', '__node:=qazbot_native_odom_bridge',
                                 '-r', '/tf:=/qazbot_d1_map/tf',
                                 '-r', '/tf_static:=/qazbot_d1_map/tf_static',
                                 '-p', 'mode:=native_imu', '-p', 'publish_tf:=true',
                                 '-p', ['publish_rate_hz:=', LaunchConfiguration('native_rate_hz')],
                                 '-p', ['leg_topic:=', LaunchConfiguration('native_leg_topic')]],
                            output='screen', condition=LaunchConfigurationEquals('odom_source', 'native_imu'))
    actions.append(native)
    guards=[RegisterEventHandler(OnProcessExit(target_action=a,on_exit=[EmitEvent(event=Shutdown(reason='Localization dependency exited'))])) for a in actions if isinstance(a,ExecuteProcess)]
    actions.insert(len(actions) - 1, seed)   # one-shot: exits after publishing, so it has no shutdown guard
    return LaunchDescription(guards+staggered(actions))
