#!/usr/bin/env bash
# Process-local, additive overlay that makes aimdk_msgs visible to ROS tools and Python.
# Source it AFTER /opt/ros/humble/setup.bash, in the shell of the process that needs it; never from
# a service, a profile or .bashrc. It only prepends the vendor prefix; nothing is rebuilt or replaced.
#
# Why it is needed: the vendor prefix /agibot/software/common is not in AMENT_PREFIX_PATH of a normal
# shell (only /opt/ros/humble is), so `ros2 pkg prefix aimdk_msgs` says "Package not found" although the
# running vendor processes (and DDS discovery) use the same types.
AIMDK_PREFIX="${AIMDK_PREFIX:-/agibot/software/common}"
export AMENT_PREFIX_PATH="$AIMDK_PREFIX:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$AIMDK_PREFIX/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"   # keep the ROS entries (rosidl_parser)
export LD_LIBRARY_PATH="$AIMDK_PREFIX/lib:${LD_LIBRARY_PATH:-}"
