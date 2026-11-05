"""X2 independent LIO + map_server + AMCL. No command publisher or MC calls.

odom_source (reports/20261027-native-odom-nav2-design.md 3.6) selects the ONE owner of TF odom->base_footprint:
  lio (default, only)  qazbot_level_bridge (C++ projector, FAST-LIO pose) owns the TF, exactly as before.
  native_imu was removed in the 2026-11-05 consolidation together with the python native_odom_bridge
  (recover from git tag pre-consolidation-fb18dc1); anything other than lio is refused.

projector_prefix (T1 2026-11-04) is the colcon install prefix of qazbot_lidar_projector: the C++ projector, started
with node name qazbot_level_bridge, the remaps below, publish_tf (LevelBridgePublishTf) and --params-file
projector_params. Consolidation 2026-11-05: the default is the installed qazbot_lidar_projector of this workspace and
lidar_projector_switch.yaml (the live production selection); the python lio/level_bridge.py fallback ('' prefix) was
removed (tag pre-consolidation-fb18dc1). lio_binary defaults to fastlio_mapping of the fast_lio package built from
ws/third_party (pinned upstream + patches).
"""
import os
import re
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory

PKG = 'qazbot_localization'
# Python nodes and launch helpers (launch_stagger, pose_store, seed_pose) are installed to lib/<pkg>.
LIB = os.path.join(get_package_prefix(PKG), 'lib', PKG)
SHARE = get_package_share_directory(PKG)
sys.path.insert(0, LIB)
from launch_stagger import staggered  # noqa: E402
import pose_store  # noqa: E402
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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


ODOM_SOURCES = ('lio',)   # native_imu removed 2026-11-05 (python native_odom_bridge), tag pre-consolidation-fb18dc1


class LevelBridgePublishTf(Substitution):
    """'true' when level_bridge owns odom->base_footprint (odom_source lio); refuses any other source so a typo can
    never start zero or two TF owners."""

    def __init__(self, source):
        super().__init__()
        self.source = source

    def perform(self, context):
        source = perform_substitutions(context, [self.source]).strip()
        if source not in ODOM_SOURCES:
            raise ValueError(f'odom_source must be one of {ODOM_SOURCES}, got {source!r}')
        return 'true' if source == 'lio' else 'false'


class ProjectorSelected(Substitution):
    """'true' when projector_prefix holds the C++ qazbot_lidar_projector and projector_params exists; refuses ''
    (the python level_bridge fallback was removed in the consolidation)."""

    def __init__(self, prefix, params):
        super().__init__()
        self.prefix, self.params = prefix, params

    def perform(self, context):
        prefix = perform_substitutions(context, [self.prefix]).strip()
        if not prefix:
            raise ValueError("projector_prefix '' selected the python level_bridge, removed 2026-11-05 "
                             '(tag pre-consolidation-fb18dc1); omit it for the installed qazbot_lidar_projector')
        if not os.access(os.path.join(prefix, 'lib', 'qazbot_lidar_projector', 'lidar_projector'), os.X_OK):
            raise ValueError(f'projector_prefix {prefix!r} has no lib/qazbot_lidar_projector/lidar_projector')
        if not os.path.isfile(perform_substitutions(context, [self.params]).strip()):
            raise ValueError('projector_prefix needs projector_params:=<copied YAML>')
        return 'true'


class PackageExecutable(Substitution):
    """Absolute path of lib/<package>/<executable>, resolved at launch time so --show-args works without it."""

    def __init__(self, package, executable, hint):
        super().__init__()
        self.package, self.executable, self.hint = package, executable, hint

    def describe(self):
        return f'$(find-pkg-prefix {self.package})/lib/{self.package}/{self.executable}'

    def perform(self, context):
        try:
            prefix = get_package_prefix(self.package)
        except PackageNotFoundError:
            raise RuntimeError(f'package {self.package!r} not found: {self.hint}') from None
        path = os.path.join(prefix, 'lib', self.package, self.executable)
        if not os.access(path, os.X_OK):
            raise RuntimeError(f'{path} is not executable: {self.hint}')
        return path


class PackagePrefix(Substitution):
    """Install prefix of a package (colcon layout <install>/<package>), resolved at launch time."""

    def __init__(self, package):
        super().__init__()
        self.package = package

    def describe(self):
        return f'$(find-pkg-prefix {self.package})'

    def perform(self, context):
        return get_package_prefix(self.package)


class PackageShareFile(Substitution):
    """share/<package>/<relative path>, resolved at launch time."""

    def __init__(self, package, *parts):
        super().__init__()
        self.package, self.parts = package, parts

    def describe(self):
        return f'$(find-pkg-share {self.package})/' + '/'.join(self.parts)

    def perform(self, context):
        return os.path.join(get_package_share_directory(self.package), *self.parts)


def generate_launch_description():
    ns = 'qazbot_d1_map'
    tf_remaps = [('/tf', '/qazbot_d1_map/tf'), ('/tf_static', '/qazbot_d1_map/tf_static')]
    lio_remaps = [('/Odometry', '/qazbot_lio/odometry'),
                  ('/cloud_registered', '/qazbot_lio/cloud_registered'),
                  ('/cloud_registered_body', '/qazbot_lio/cloud_registered_body'),
                  ('/cloud_effected', '/qazbot_lio/cloud_effected'),
                  ('/Laser_map', '/qazbot_lio/map'), ('/path', '/qazbot_lio/path'),
                  ('/tf', '/qazbot_lio/tf'), ('map_save', '/qazbot_lio/map_save')]
    lio_args = ['--ros-args', '-r', '__node:=qazbot_lio_mapping', '--params-file',
                os.path.join(SHARE, 'config', 'x2.yaml')]
    for source, target in lio_remaps:
        lio_args += ['-r', source + ':=' + target]
    actions = [
        DeclareLaunchArgument('lio_binary', default_value=[PackageExecutable(
            'fast_lio', 'fastlio_mapping', 'build ws/third_party (FAST-LIO pinned + patches) or pass lio_binary:=')],
            description='FAST-LIO fastlio_mapping executable; default: lib/fast_lio/fastlio_mapping of package fast_lio'),
        DeclareLaunchArgument('map_yaml', default_value=[PackageShareFile('qazbot_maps', 'maps', 'first_map_nav2_v2',
                                                                         'map.yaml')]),
        # Empty lio_cpus (default) leaves scheduling unchanged.
        DeclareLaunchArgument('lio_cpus', default_value=''),
        DeclareLaunchArgument('seed_pose_file', default_value=pose_store.DEFAULT_PATH),
        DeclareLaunchArgument('odom_source', default_value='lio', choices=list(ODOM_SOURCES)),
        # Install prefix of the C++ projector (default: this workspace's qazbot_lidar_projector) and its params.
        DeclareLaunchArgument('projector_prefix', default_value=[PackagePrefix('qazbot_lidar_projector')]),
        DeclareLaunchArgument('projector_params', default_value=[PackageShareFile(
            'qazbot_lidar_projector', 'config', 'lidar_projector_switch.yaml')]),
        ExecuteProcess(cmd=[LaunchConfiguration('lio_binary')] + lio_args, output='screen',
                       prefix=TasksetPrefix(LaunchConfiguration('lio_cpus'))),
        # FAST-LIO -> 2D bridge: C++ projector (node name qazbot_level_bridge, the only odom->base_footprint owner).
        ExecuteProcess(cmd=[PathJoinSubstitution([LaunchConfiguration('projector_prefix'), 'lib',
                                                  'qazbot_lidar_projector', 'lidar_projector']), '--ros-args',
                            '-r', '__node:=qazbot_level_bridge',
                            '-r', '/Odometry:=/qazbot_lio/odometry',
                            '-r', '/cloud_registered:=/qazbot_lio/cloud_registered',
                            '-r', '/scan:=/qazbot_lio/scan',
                            '-r', '/odom_level:=/qazbot_lio/odom_level',
                            '-r', '/tf:=/qazbot_d1_map/tf',
                            '-r', '/tf_static:=/qazbot_d1_map/tf_static',
                            '--params-file', LaunchConfiguration('projector_params'),
                            '-p', ['publish_tf:=', LevelBridgePublishTf(LaunchConfiguration('odom_source'))]],
                       additional_env={'LD_LIBRARY_PATH': [LaunchConfiguration('projector_prefix'), '/lib:',
                                                           EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]},
                       condition=IfCondition(ProjectorSelected(LaunchConfiguration('projector_prefix'),
                                                               LaunchConfiguration('projector_params'))),
                       output='screen'),
        Node(package='nav2_map_server', executable='map_server', namespace=ns,
             name='map_server', output='screen', remappings=tf_remaps,
             parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'), 'topic_name': 'map',
                          'frame_id': 'map', 'use_sim_time': False}]),
        # AMCL parameters: config/amcl.yaml (values unchanged from the inline dict of fb18dc1).
        Node(package='nav2_amcl', executable='amcl', namespace=ns, name='amcl',
             output='screen', remappings=tf_remaps,
             parameters=[os.path.join(SHARE, 'config', 'amcl.yaml')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
    ]
    # Re-seed the last CONFIRMED pose (written by the trust node, param pose_store) ONCE as initialpose with its
    # saved covariance. Missing / corrupt / other-map file: nothing is published. Never grants READY: the trust
    # node needs confirm_anchor + a clean gate on it. seed_pose_file:='' disables.
    seed = ExecuteProcess(cmd=['python3', os.path.join(LIB, 'seed_pose.py'),
                               '--from-file', LaunchConfiguration('seed_pose_file'),
                               '--map-yaml', LaunchConfiguration('map_yaml')], output='screen',
                          condition=LaunchConfigurationNotEquals('seed_pose_file', ''))
    guards=[RegisterEventHandler(OnProcessExit(target_action=a,on_exit=[EmitEvent(event=Shutdown(reason='Localization dependency exited'))])) for a in actions if isinstance(a,ExecuteProcess)]
    actions.append(seed)   # one-shot: exits after publishing, so it has no shutdown guard
    return LaunchDescription(guards+staggered(actions))
