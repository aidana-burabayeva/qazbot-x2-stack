#!/bin/bash
# Environment INSIDE the twin container for running the QazBot stack next to the official mc + MuJoCo.
# Usage: exec through ex.sh-like wrapper:  docker exec -i qazbot-twin-sim tools/.../env_container.sh CMD...
CB=/workspace/control_backend
source /opt/ros/humble/setup.bash
source /workspace/aimdk/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 QAZBOT_SIM=1
SMAC=$CB/runtime_x86/build_ws/install/nav2_smac_planner          # host-built, bounds.patch (baked prefixes differ: set explicitly)
if [ "${QAZBOT_SIM_EXACT_GOAL:-0}" = 1 ]; then
  SMAC=$CB/runtime_x86/exact_goal_ws/install/nav2_smac_planner
  [ -f "$SMAC/qazbot_bounds_fix.json" ] || { echo 'Build the exact-goal simulation candidate first' >&2;exit 2; }
fi
export AMENT_PREFIX_PATH="$SMAC:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$SMAC/lib:$LD_LIBRARY_PATH" QAZBOT_SMAC_PREFIX="$SMAC"
export FASTRTPS_DEFAULT_PROFILES_FILE=$CB/tools/twin/dds_sim_loopback.xml
if [ "${QAZBOT_SIM_DDS_UDP_ONLY:-0}" = 1 ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=$CB/tools/twin/dds_sim_udp_loopback.xml
fi
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [ "${QAZBOT_SIM_MC_TURN:-0}" = 1 ]; then
  source "$CB/runtime_x86/turn_ws/install/local_setup.bash"
fi
if [ "${QAZBOT_SIM_POSE_REFINE:-0}" = 1 ]; then
  source "$CB/runtime_x86/pose_refine_ws/install/local_setup.bash"
fi
exec "$@"
