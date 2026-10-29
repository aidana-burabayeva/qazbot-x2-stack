"""Truth-only localisation (A2): map_server + truth_localizer instead of AMCL + Localization Trust. Twin only.

The TF edge map -> odom and the trust verdict come from ground truth (see x2_twin_sim.loc.truth_localizer_node).
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'qazbot_d1_map'


def generate_launch_description():
    stack = os.environ.get('TWIN_STACK_DIR', '/opt/x2twin/stack')
    sys.path.insert(0, os.path.join(stack, 'navigation'))
    from launch_stagger import staggered          # AimDK: <= 2 new ROS nodes per second

    tf = [('/tf', f'/{NS}/tf'), ('/tf_static', f'/{NS}/tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(stack, 'maps/first_map_nav2_v3/map.yaml')),
        *staggered([
            Node(package='nav2_map_server', executable='map_server', namespace=NS, name='map_server', output='screen',
                 remappings=tf,
                 parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'), 'topic_name': 'map',
                              'frame_id': 'map', 'use_sim_time': False}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=NS,
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}]),
            Node(package='x2_twin_sim', executable='truth_localizer', output='screen'),
        ]),
    ])
