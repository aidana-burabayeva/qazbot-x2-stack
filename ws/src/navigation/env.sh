#!/usr/bin/env bash
# Process-local overlays: never source from a robot service or shell profile.
set -e
source /opt/ros/humble/setup.bash
QAZBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# navigation/ lives in control_backend/ or in control_backend/ws/src/; runtime/ stays in control_backend/.
if [ -d "$QAZBOT_ROOT/runtime" ]; then QAZBOT_BACKEND="$QAZBOT_ROOT"; else QAZBOT_BACKEND="$(cd "$QAZBOT_ROOT/../.." && pwd)"; fi
export QAZBOT_BACKEND
QAZBOT_RUNTIME="$QAZBOT_BACKEND/runtime"
export AMENT_PREFIX_PATH="$QAZBOT_RUNTIME/nav2_bt_deps/root/opt/ros/humble:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble:$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble:$QAZBOT_RUNTIME/fast_lio:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$QAZBOT_RUNTIME/nav2_bt_deps/root/opt/ros/humble/lib:$QAZBOT_RUNTIME/fast_lio/lib:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble/lib:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble/lib/aarch64-linux-gnu:$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble/lib:$QAZBOT_RUNTIME/user_pkgs/usr/lib:$QAZBOT_RUNTIME/user_pkgs/usr/lib/aarch64-linux-gnu:$QAZBOT_RUNTIME/user_pkgs/usr/lib/aarch64-linux-gnu/lapack:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
source "$HOME/.aima/env/bashrc"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE
# Stack-only DDS profile: vendor profile + wifi address so a PC on 192.168.1.0/24 (RViz2) can discover this stack.
export FASTRTPS_DEFAULT_PROFILES_FILE="$QAZBOT_ROOT/navigation/dds_qazbot.xml"

# Explicit candidate overlay; never replaces system/vendor packages.
if [ -n "${QAZBOT_SMAC_PREFIX:-}" ]; then
  test -f "$QAZBOT_SMAC_PREFIX/share/ament_index/resource_index/packages/nav2_smac_planner"
  export AMENT_PREFIX_PATH="$QAZBOT_SMAC_PREFIX:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$QAZBOT_SMAC_PREFIX/lib:$LD_LIBRARY_PATH"
fi

# Explicit lifecycle shutdown candidate, isolated from system/vendor packages.
if [ -n "${QAZBOT_LIFECYCLE_PREFIX:-}" ]; then
  test -f "$QAZBOT_LIFECYCLE_PREFIX/share/ament_index/resource_index/packages/nav2_lifecycle_manager"
  export AMENT_PREFIX_PATH="$QAZBOT_LIFECYCLE_PREFIX:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$QAZBOT_LIFECYCLE_PREFIX/lib:$LD_LIBRARY_PATH"
fi
