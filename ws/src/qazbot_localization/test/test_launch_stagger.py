"""Process-start pacing of the stack launch files (AimDK: at most 2 ROS nodes per second while X2 stands).

Parses the launch files without starting anything. LAUNCH_DIR selects the navigation directory
(default: the launch/ dirs of qazbot_localization and qazbot_navigation in this source tree). Needs ROS Humble (launch, launch_ros).
Run: python3 -m unittest -v tests/launch_stagger/test_launch_stagger.py
"""
import importlib.util
import os
import sys
import unittest

from launch import LaunchContext
from launch.actions import ExecuteProcess, TimerAction

HERE = os.path.dirname(os.path.abspath(__file__))
LOC_LAUNCH = os.environ.get('LAUNCH_DIR', os.path.join(HERE, '..', 'launch'))
NAV = os.environ.get('LAUNCH_DIR', os.path.join(HERE, '..', '..', 'qazbot_navigation', 'launch'))
MAPS = os.path.join(HERE, '..', '..', 'qazbot_maps', 'maps')
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), os.path.join(LOC_LAUNCH if name.startswith('localization') else NAV, name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def start_times(entities):
    """Seconds after launch at which each process starts."""
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

    def test_localization_launch_is_paced(self):
        mod = load('localization.launch.py')
        times = start_times(mod.generate_launch_description().entities)
        self.check(times, 5)                                     # fastlio, bridge, map_server, amcl, lifecycle (+ adapter)
        self.assertEqual(times[0], 0.0)

    def test_localization_lio_cpus_prefix_is_opt_in(self):
        """lio_cpus '' (default) adds no prefix; a CPU list adds 'taskset -c <list>'; junk is refused."""
        mod = load('localization.launch.py')
        procs = [a for a in mod.generate_launch_description().entities if isinstance(a, ExecuteProcess)]
        lio = [p for p in procs if p.process_description.prefix
               and isinstance(p.process_description.prefix[0], mod.TasksetPrefix)]
        self.assertEqual(len(lio), 1)
        from launch.utilities import perform_substitutions
        for value, expected in (('', ''), ('5,6', 'taskset -c 5,6'), ('5-6', 'taskset -c 5-6')):
            ctx = LaunchContext()
            ctx.launch_configurations['lio_cpus'] = value
            self.assertEqual(perform_substitutions(ctx, lio[0].process_description.prefix), expected)
        for bad in ('5;rm -rf /', 'all', '5,'):
            ctx = LaunchContext()
            ctx.launch_configurations['lio_cpus'] = bad
            with self.assertRaises(ValueError):
                perform_substitutions(ctx, lio[0].process_description.prefix)

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

    def test_helper_keeps_order_and_leaves_non_processes_alone(self):
        from launch.actions import DeclareLaunchArgument
        from launch_stagger import STEP_S, staggered
        acts = [DeclareLaunchArgument('x'), ExecuteProcess(cmd=['true']), ExecuteProcess(cmd=['true']),
                ExecuteProcess(cmd=['true'])]
        out = staggered(acts)
        self.assertIsInstance(out[0], DeclareLaunchArgument)
        self.assertIsInstance(out[1], ExecuteProcess)
        self.assertEqual([round(a.period, 3) for a in out[2:]], [STEP_S, 2 * STEP_S])
        self.assertLess(1.0 / STEP_S, 2.0)


if __name__ == '__main__':
    unittest.main()
