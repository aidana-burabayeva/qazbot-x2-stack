#!/usr/bin/env bash
# SIM environment (x86). Replaces navigation/env.sh, which sources the robot's vendor env and ARM64 runtime.
# Isolation: domain 87 + localhost only + loopback-only DDS profile. Refuses to run next to the real robot.
TWIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TWIN_ROOT/runtime_x86/build_ws/install/setup.bash"
export QAZBOT_SMAC_PREFIX="$TWIN_ROOT/runtime_x86/build_ws/install/nav2_smac_planner"
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 QAZBOT_SIM=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$TWIN_ROOT/tools/twin/dds_sim_loopback.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
