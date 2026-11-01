#!/usr/bin/env python3
"""Records the operator session to a rosbag2 (sqlite3). Read-only subscriber; SIGTERM/SIGINT closes the bag cleanly."""
import argparse
import os
import signal
import subprocess
import sys

NS = '/qazbot_d1_map'
REGEX = '|'.join([
    NS + r'/(tf|tf_static|initialpose|goal_pose|amcl_pose|particle_cloud|map|cmd_vel|cmd_vel_authorized)',
    NS + r'/localization_trust/.*', NS + r'/goal_coordinator/.*', NS + r'/motion_adapter/.*', NS + r'/operator/.*',
    NS + r'/nav2/(plan|received_global_plan|cmd_vel_nav|lookahead_point|local_costmap/costmap|global_costmap/costmap'
         r'|navigate_to_pose/_action/(status|feedback)|follow_path/_action/status)',
    r'/qazbot_lio/(scan|odom_level)', r'/x2_twin/.*',
    r'/aima/(mc/locomotion/velocity|hal/odom/state|mc/common/state)',
])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    cmd = ['ros2', 'bag', 'record', '-o', a.out, '--storage', 'sqlite3', '--max-cache-size', str(200 * 2**20), '-e', '^(' + REGEX + ')$']
    p = subprocess.Popen(cmd)
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda s, f: p.send_signal(signal.SIGINT))
    sys.exit(p.wait())


if __name__ == '__main__':
    main()
