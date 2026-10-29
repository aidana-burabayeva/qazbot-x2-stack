#!/usr/bin/env python3
"""Health of the official sim: the simulated robot stands (pelvis z > 0.6 m) and mc reports body=STAND, fsm in (STABLE, MOVING).

  twin-sim-health            one check (up to 6 s)           exit 0 healthy / 1 not
  twin-sim-health --wait S   poll until healthy, at most S s
Wire values (observed on the robot): body_status 1 = STAND; fsm 4 = STABLE, 5 = MOVING (AimDK 'mc_status', zh-cn page).
"""
import sys
import time

import rclpy
from aimdk_msgs.msg import McCommonState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def healthy(node, st, secs):
    st.clear()
    end = time.monotonic() + secs
    while time.monotonic() < end and not ('z' in st and 'body' in st):
        rclpy.spin_once(node, timeout_sec=0.1)
    return st.get('z', 0.0) > 0.6 and st.get('body') == 1 and st.get('fsm') in (4, 5)


def main():
    wait = float(sys.argv[sys.argv.index('--wait') + 1]) if '--wait' in sys.argv else 0.0
    rclpy.init()
    node, st = rclpy.create_node('twin_sim_health'), {}
    node.create_subscription(Odometry, '/aima/hal/odom/state', lambda m: st.__setitem__('z', m.pose.pose.position.z), qos_profile_sensor_data)
    node.create_subscription(McCommonState, '/aima/mc/common/state',
                             lambda m: st.update(body=m.body_status.value, fsm=m.fsm_state.current_state), qos_profile_sensor_data)
    end = time.monotonic() + wait
    ok = healthy(node, st, 6.0)
    while not ok and time.monotonic() < end:
        ok = healthy(node, st, 3.0)
    print(dict(st), 'healthy' if ok else 'NOT healthy')
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
