#!/bin/bash
# Stand-in for the robot's aarch64 aima-mc-app-main (renamed to aima-mc-app-main.real): runs it under qemu-user inside this container.
# The host needs no binfmt entry. Guest libraries come only from the robot's own trees (never the x86 ROS of this container).
set -e
D=/vendor/pc1; MC=$(cd "$(dirname "$0")/.." && pwd)
export QEMU_LD_PREFIX=$D/sysroot_ubuntu2204_arm64
R=$D/ros_humble_aarch64/humble
export QEMU_SET_ENV="LD_LIBRARY_PATH=$MC/lib:$MC/mc3th/lib:$D/common_aarch64/common/lib:$D/common_aarch64/common/rtlib:$D/syslibs_aarch64/lib/aarch64-linux-gnu:$D/syslibs_aarch64/lib:$R/lib:$R/lib/aarch64-linux-gnu,AMENT_PREFIX_PATH=$R,RMW_IMPLEMENTATION=rmw_fastrtps_cpp,FASTRTPS_DEFAULT_PROFILES_FILE=/opt/twin-sim/fastdds_udp_only.xml"
exec qemu-aarch64-static "$(dirname "$0")/aima-mc-app-main.real" "$@"
