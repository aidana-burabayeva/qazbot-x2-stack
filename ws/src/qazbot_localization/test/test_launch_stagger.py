"""Process-start pacing of the stack launch files (AimDK: at most 2 ROS nodes per second while X2 stands).

Parses the launch files without starting anything. LAUNCH_DIR selects the navigation directory
(default: launch/ of qazbot_localization). Pacing of the qazbot_navigation launch files is tested in
qazbot_navigation/test/test_launch_stagger_nav.py (qazbot_navigation depends on qazbot_localization, not vice versa).
Needs ROS Humble (launch, launch_ros).
"""
import importlib.util
import os
import sys
import unittest

from launch import LaunchContext
from launch.actions import ExecuteProcess, TimerAction

HERE = os.path.dirname(os.path.abspath(__file__))
LOC_LAUNCH = os.environ.get('LAUNCH_DIR', os.path.join(HERE, '..', 'launch'))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), os.path.join(LOC_LAUNCH, name))
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
