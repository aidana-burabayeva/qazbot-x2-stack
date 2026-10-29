"""Static checks of the launch files (no ROS runtime): they parse and expose the documented arguments."""
import ast
import pathlib

LAUNCH = pathlib.Path(__file__).resolve().parents[1] / 'launch'


def test_launch_files_parse():
    for f in LAUNCH.glob('*.launch.py'):
        ast.parse(f.read_text())


def test_twin_launch_exposes_documented_arguments():
    text = (LAUNCH / 'twin.launch.py').read_text()
    for arg in ('plant', 'map_yaml', 'start_pose', 'max_angular', 'robot_radius', 'trust_policy'):
        assert f"DeclareLaunchArgument('{arg}'" in text


def test_max_angular_default_is_held_at_half_a_radian_per_second():
    assert "DeclareLaunchArgument('max_angular', default_value='0.5'" in (LAUNCH / 'twin.launch.py').read_text()
