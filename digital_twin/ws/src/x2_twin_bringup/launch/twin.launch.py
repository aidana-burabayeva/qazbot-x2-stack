"""Run the unmodified X2 navigation stack against the digital twin.

  ros2 launch x2_twin_bringup twin.launch.py plant:=official|lite [map_yaml:=...] [max_angular:=0.5]

plant:=official  expects the official mc + MuJoCo (separate container) publishing /aima/hal/odom/state and serving the MC services.
plant:=lite      starts the protocol-level virtual MC and a kinematic plant here (no MuJoCo; idealised, not calibrated).
The stack is started the way tools/stack/stack_up.sh and the run_*.sh scripts start it: same scripts, same arguments, no remaps.
The adapter runs with live:=true and registers in the (virtual or official) MC only when the operator calls ~/register.
"""
import hashlib
import json
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

NS = 'qazbot_d1_map'


def _stack_dir() -> str:
    return os.environ.get('TWIN_STACK_DIR', '/opt/x2twin/stack')


def _localization(L, nav, map_yaml, pgm_sha):
    if L('localization') == 'truth':      # twin-only isolation of the controller and the MC from localisation (roadmap A2)
        return [ExecuteProcess(cmd=['ros2', 'launch', 'x2_twin_bringup', 'truth_loc.launch.py', f'map_yaml:={map_yaml}'], output='screen')]
    if L('localization') != 'amcl':
        raise ValueError("localization must be 'amcl' or 'truth'")
    return [
        ExecuteProcess(cmd=['ros2', 'launch', 'x2_twin_bringup', 'amcl.launch.py', f'map_yaml:={map_yaml}'], output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(nav, 'localization_trust_node.py'), '--ros-args',
                            '-p', f'map_yaml:={map_yaml}', '-p', f'expected_map_sha256:={pgm_sha}',
                            *(['-p', f'admission_policy:={L("trust_policy")}'] if L('trust_policy') else [])], output='screen'),
    ]


def _setup(context):
    def L(name):
        return LaunchConfiguration(name).perform(context)

    nav = os.path.join(_stack_dir(), 'navigation')
    sys.path.insert(0, nav)
    from launch_stagger import staggered
    plant, map_yaml = L('plant'), L('map_yaml')
    if plant not in ('official', 'lite'):
        raise ValueError("plant must be 'official' or 'lite'")
    manifest = os.path.join(os.path.dirname(map_yaml), 'manifest.json')
    with open(manifest) as f:
        pgm_sha = json.load(f)['pgm_sha256']
    with open(os.path.join(os.path.dirname(map_yaml), 'map.pgm'), 'rb') as f:
        if hashlib.sha256(f.read()).hexdigest() != pgm_sha:
            raise RuntimeError('map.pgm does not match its manifest')
    params = os.path.join(get_package_share_directory('x2_twin_bringup'), 'config', 'twin.yaml')
    start = [float(v) for v in L('start_pose').split(',')]
    extra = {'map_yaml': map_yaml, 'start_pose': start, 'scan_format': L('scan_format')}
    if float(L('scan_fov_deg')) > 0:                    # scenario override of the plant's scan sector (default: twin.yaml)
        extra['scan_fov_deg'] = float(L('scan_fov_deg'))
    plant_params = [params, extra]

    actions = []
    if plant == 'official':
        actions.append(Node(package='x2_twin_sim', executable='hal_plant', name='hal_plant', output='screen',
                            parameters=plant_params))
    else:
        actions += [Node(package='x2_twin_sim', executable='virtual_mc', name='virtual_mc', output='screen', parameters=[params]),
                    Node(package='x2_twin_sim', executable='kinematic_plant', name='kinematic_plant', output='screen',
                         parameters=plant_params)]
    actions += [
        *_localization(L, nav, map_yaml, pgm_sha),
        ExecuteProcess(cmd=['python3', os.path.join(nav, 'x2_motion_adapter.py'), '--ros-args', '-r', f'__ns:=/{NS}',
                            '-p', 'live:=true', '-p', 'lock_path:=/tmp/x2_motion_owner.lock'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'launch', os.path.join(nav, 'nav_full.launch.py'),
                            f'robot_radius:={L("robot_radius")}', f'max_angular:={L("max_angular")}',
                            f'xy_tol:={L("xy_tol")}', f'yaw_tol:={L("yaw_tol")}',
                            'scan_topic:=/qazbot_lio/scan', 'odom_topic:=/qazbot_lio/odom_level',
                            'allow_dry_run_driver:=false', f'map_yaml:={map_yaml}'], output='screen'),
    ]
    return staggered(actions)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('plant', default_value='official', description='official | lite'),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(_stack_dir(), 'maps/first_map_nav2_v3/map.yaml')),
        DeclareLaunchArgument('start_pose', default_value='-1.275,-4.825,0.0', description='x,y,yaw of the robot at start (map frame)'),
        DeclareLaunchArgument('max_angular', default_value='0.5', description='rad/s; held at 0.5 until the operator approves a change'),
        DeclareLaunchArgument('robot_radius', default_value='0.454'),
        DeclareLaunchArgument('xy_tol', default_value='0.10', description='goal checker XY, m; robot runs ROBOT_XY_TOL (scripts/contract/robot.env)'),
        DeclareLaunchArgument('yaw_tol', default_value='0.10', description='goal checker yaw, rad; robot runs ROBOT_YAW_TOL'),
        DeclareLaunchArgument('trust_policy', default_value='',
                              description="strict | go2; empty = the stack's own default (currently go2)"),
        DeclareLaunchArgument('scan_format', default_value='legacy',
                              description='legacy = level_bridge today; i1 = candidate scan (finite / +inf in FOV / NaN)'),
        DeclareLaunchArgument('scan_fov_deg', default_value='0', description='0 = twin.yaml; else the plant scan sector in degrees'),
        DeclareLaunchArgument('localization', default_value='amcl',
                              description="amcl = real AMCL + Localization Trust; truth = ground truth map->odom (twin-only isolation)"),
        OpaqueFunction(function=_setup),
    ])
