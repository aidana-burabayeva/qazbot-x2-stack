"""Process-start pacing of the qazbot_navigation launch files (AimDK: at most 2 ROS nodes per second while X2 stands).

Parses the launch files without starting anything. LAUNCH_DIR overrides the launch/ directory of this package.
Needs ROS Humble (launch, launch_ros) and the sibling qazbot_localization scripts (launch_stagger).
"""
import importlib.util
import os
import sys
import unittest

from launch import LaunchContext
from launch.actions import ExecuteProcess, TimerAction

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.environ.get('LAUNCH_DIR', os.path.join(HERE, '..', 'launch'))
MAPS = os.path.join(HERE, '..', '..', 'qazbot_maps', 'maps')
sys.path.insert(0, os.path.join(HERE, '..', '..', 'qazbot_localization', 'scripts'))


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), os.path.join(NAV, name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def start_times(entities):
    t = []
    for e in entities:
        if isinstance(e, TimerAction):
            t += [float(e.period)] * sum(isinstance(a, ExecuteProcess) for a in e.actions)
        elif isinstance(e, ExecuteProcess):
            t.append(0.0)
    return sorted(t)


def max_starts_in_one_second(times):
    return max((sum(1 for u in times if t <= u < t + 1.0) for t in times), default=0)


class Pacing(unittest.TestCase):
    def check(self, times, minimum):
        self.assertGreaterEqual(len(times), minimum)
        self.assertLessEqual(max_starts_in_one_second(times), 2, times)
        self.assertEqual(times, sorted(times))

    def test_nav_plan_launch_is_paced(self):
        mod = load('nav_plan.launch.py')
        ctx = LaunchContext()
        near = os.path.join(MAPS, 'first_map_nav2_v2', 'map.yaml')
        repo = os.path.join(MAPS, 'first_map_nav2_v2', 'map.yaml')
        ctx.launch_configurations.update(robot_radius='0.65', cell_margin='true',
                                         map_yaml=near if os.path.exists(near) else repo)
        times = start_times(mod.setup(ctx))
        self.check(times, 3)

    def test_full_nav_launch_is_paced(self):
        overlay = _smac_overlay_prefix()
        if not overlay:
            self.skipTest('qualified Smac overlay absent (no qazbot_bounds_fix.json in QAZBOT_SMAC_PREFIX / ament index): '
                          'build with docker/tools/build_workspace.sh --prefix DIR and source DIR/install/setup.bash')
        mod=load('nav_full.launch.py');ctx=LaunchContext()
        ctx.launch_configurations.update(ns='qazbot_test',robot_radius='0.454',cell_margin='true',
            map_yaml=os.path.join(MAPS,'first_map_nav2_v2','map.yaml'),
            scan_topic='/qazbot_test/scan',odom_topic='/qazbot_test/odom',max_linear='0.3',max_angular='0.5',xy_tol='0.1',yaw_tol='0.1')
        self.check(start_times(mod.setup(ctx)),6)

    def test_full_nav_refuses_unqualified_old_smac(self):
        from unittest.mock import patch
        mod = load('nav_full.launch.py')
        # Consolidation: no QAZBOT_SMAC_PREFIX and only prefixes WITHOUT the qualified overlay manifest in the index.
        unq = [p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
               if p and not os.path.isfile(os.path.join(p, 'qazbot_bounds_fix.json'))]
        with patch.dict(os.environ,{'AMENT_PREFIX_PATH': os.pathsep.join(unq)},clear=True):
            with self.assertRaisesRegex(RuntimeError,'qualified Smac'):
                mod.setup(LaunchContext())


def _smac_overlay_prefix():
    """Prefix of the qualified (patched, manifest-carrying) nav2_smac_planner, or '' when it is not built/sourced."""
    prefix = os.environ.get('QAZBOT_SMAC_PREFIX')
    if not prefix:
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = get_package_prefix('nav2_smac_planner')
        except Exception:
            return ''
    return prefix if os.path.isfile(os.path.join(prefix, 'qazbot_bounds_fix.json')) else ''


if __name__ == '__main__':
    unittest.main()
