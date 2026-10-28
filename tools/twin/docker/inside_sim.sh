#!/bin/bash
# Runs INSIDE the isolated container. Official order (docs "X2 MuJoCo Motion-Control Simulation"): 1 MuJoCo, 2 MC.
source /opt/ros/humble/setup.bash
# Container has no GPU device. Software Mesa avoids nouveau/DRI3 MIT-SHM crashes.
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-true}
export LIBGL_DRI3_DISABLE=${LIBGL_DRI3_DISABLE:-1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Vendor -s clears the configuration AFTER parsing -c. Never combine them for overlays.
case "${QAZBOT_TWIN_MODEL:-official}" in
  official) sim_args=(-s) ;;
  map_world) sim_args=(-c "${QAZBOT_SIM_CONFIG:?generated map configuration required}") ;;
  robot66_nominal) sim_args=(-c /workspace/control_backend/runtime_x86/twin_robot66/configuration) ;;
  *) echo "Unknown twin model" >&2; exit 2 ;;
esac
if [ "${QAZBOT_HEADLESS:-0}" = 1 ]; then
  command -v xvfb-run >/dev/null || { echo "QAZBOT_HEADLESS requires xvfb-run" >&2; exit 2; }
  # Wrap this script: vendor pgrep rejects any parent whose argv contains start_sim.sh.
  exec env QAZBOT_HEADLESS=0 xvfb-run -a -s '-screen 0 1280x720x24' "$0"
fi
cd /workspace/sim_mujoco/bin && echo 0 | ./start_sim.sh "${sim_args[@]}"
