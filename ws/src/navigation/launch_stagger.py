"""Space out process starts.

AimDK X2 docs (5.1.3): while the robot stands or walks, do not start more than 2 ROS nodes per
second; many nodes joining DDS discovery at once can congest communication and destabilise MC.
Every ExecuteProcess (launch_ros Node subclasses it) after the first is delayed by STEP_S more.
"""
from launch.actions import ExecuteProcess, TimerAction

STEP_S = 0.6     # 1 / 0.6 = 1.7 starts per second, below the vendor limit of 2


def staggered(actions, step=STEP_S):
    out, n = [], 0
    for a in actions:
        if isinstance(a, ExecuteProcess):
            out.append(a if n == 0 else TimerAction(period=n * step, actions=[a]))
            n += 1
        else:
            out.append(a)
    return out
