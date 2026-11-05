"""REAL map_server + REAL nav2_amcl for the twin: the AMCL part of navigation/localization.launch.py.

The parameters are the production ones, unchanged (parity is enforced by twin/test_amcl_parity.py). The only
difference is where the scan comes from: `scan_topic` (production: /qazbot_lio/scan, FAST-LIO via level_bridge;
twin: the plant's ray-cast scan). FAST-LIO and level_bridge are not started here: the plant publishes odom/TF.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))  # repo root (ws/tools/twin -> root)
sys.path.insert(0, os.path.join(ROOT, 'ws/src/navigation'))
from launch_stagger import staggered  # noqa: E402
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    ns = LaunchConfiguration('ns')
    remaps = [('/tf', ['/', ns, '/tf']), ('/tf_static', ['/', ns, '/tf_static'])]
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='qazbot_d1_map'),
        DeclareLaunchArgument('scan_topic'),
        DeclareLaunchArgument('map_yaml', default_value=os.path.join(ROOT, 'ws/src/maps/first_map_nav2_v2/map.yaml')),
        *staggered([
            Node(package='nav2_map_server', executable='map_server', namespace=ns, name='map_server', output='screen',
                 remappings=remaps,
                 parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'), 'topic_name': 'map',
                              'frame_id': 'map', 'use_sim_time': False}]),
            Node(package='nav2_amcl', executable='amcl', namespace=ns, name='amcl', output='screen', remappings=remaps,
                 parameters=[{'use_sim_time': False, 'global_frame_id': 'map', 'odom_frame_id': 'odom',
                              'base_frame_id': 'base_footprint', 'map_topic': 'map',
                              'scan_topic': LaunchConfiguration('scan_topic'), 'max_beams': 1440,
                              'robot_model_type': 'nav2_amcl::OmniMotionModel'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=ns,
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        ])])
