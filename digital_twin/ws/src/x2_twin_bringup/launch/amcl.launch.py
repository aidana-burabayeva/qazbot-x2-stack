"""REAL map_server + REAL nav2_amcl with the PRODUCTION parameters of ws/src/navigation/localization.launch.py.

Only the scan source differs from the robot (the twin's plant instead of FAST-LIO + level_bridge); FAST-LIO itself is not started.
Parity with the production launch is enforced by test/test_amcl_parity.py.
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
    from launch_stagger import staggered          # AimDK: <= 2 new ROS nodes per second while the robot stands or walks

    tf = [('/tf', f'/{NS}/tf'), ('/tf_static', f'/{NS}/tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(stack, 'maps/first_map_nav2_v3/map.yaml')),
        DeclareLaunchArgument('scan_topic', default_value='/qazbot_lio/scan'),
        *staggered([
            Node(package='nav2_map_server', executable='map_server', namespace=NS, name='map_server', output='screen',
                 remappings=tf,
                 parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'), 'topic_name': 'map',
                              'frame_id': 'map', 'use_sim_time': False}]),
            Node(package='nav2_amcl', executable='amcl', namespace=NS, name='amcl', output='screen', remappings=tf,
                 parameters=[{'use_sim_time': False, 'global_frame_id': 'map', 'odom_frame_id': 'odom',
                              'base_frame_id': 'base_footprint', 'map_topic': 'map',
                              'scan_topic': LaunchConfiguration('scan_topic'), 'max_beams': 1440,
                              'robot_model_type': 'nav2_amcl::OmniMotionModel'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=NS,
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        ]),
    ])
