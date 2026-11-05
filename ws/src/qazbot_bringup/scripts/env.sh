#!/usr/bin/env bash
# Process-local environment of the X2 navigation stack (installed layout). Never source it from a robot service,
# a shell profile or .bashrc. Usage:  source <install>/qazbot_bringup/lib/qazbot_bringup/env.sh
#   QAZBOT_RUNTIME  pinned Nav2 runtime debs (ws/third_party/fetch_runtime_debs.sh); default <release>/deps,
#                   layout {qazbot_nav_deps/root,nav2_bt_deps/root,user_pkgs} as the legacy control_backend/runtime
#   QAZBOT_DDS_PROFILE  FastDDS profile; default share/qazbot_bringup/config/dds_qazbot.xml (robot, 10.0.1.41 + Wi-Fi)
#   QAZBOT_SMAC_PREFIX / QAZBOT_LIFECYCLE_PREFIX  explicit candidate overlays (unchanged semantics)
source /opt/ros/humble/setup.bash
QAZBOT_INSTALL="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
export QAZBOT_INSTALL
QAZBOT_RUNTIME="${QAZBOT_RUNTIME:-$(cd -- "$QAZBOT_INSTALL/.." && pwd)/deps}"
export QAZBOT_RUNTIME
if [ -d "$QAZBOT_RUNTIME/qazbot_nav_deps" ]; then
  export AMENT_PREFIX_PATH="$QAZBOT_RUNTIME/nav2_bt_deps/root/opt/ros/humble:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble:$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$QAZBOT_RUNTIME/nav2_bt_deps/root/opt/ros/humble/lib:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble/lib:$QAZBOT_RUNTIME/qazbot_nav_deps/root/opt/ros/humble/lib/aarch64-linux-gnu:$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble/lib:$QAZBOT_RUNTIME/user_pkgs/usr/lib:$QAZBOT_RUNTIME/user_pkgs/usr/lib/aarch64-linux-gnu:$QAZBOT_RUNTIME/user_pkgs/usr/lib/aarch64-linux-gnu/lapack:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$QAZBOT_RUNTIME/user_pkgs/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
else
  echo "qazbot env.sh: WARN no runtime debs at $QAZBOT_RUNTIME (Nav2 must then come from dpkg)" >&2
fi
# Workspace last: its packages (patched nav2_smac_planner, fast_lio, qazbot_*) shadow the runtime debs.
source "$QAZBOT_INSTALL/setup.bash"
if [ -f "$HOME/.aima/env/bashrc" ]; then source "$HOME/.aima/env/bashrc"; fi   # vendor env (robot only)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE
# Stack-only DDS profile: vendor profile + wifi address so a PC on 192.168.1.0/24 (RViz2) can discover this stack.
export FASTRTPS_DEFAULT_PROFILES_FILE="${QAZBOT_DDS_PROFILE:-$QAZBOT_INSTALL/qazbot_bringup/share/qazbot_bringup/config/dds_qazbot.xml}"

# Explicit candidate overlay; never replaces system/vendor packages.
if [ -n "${QAZBOT_SMAC_PREFIX:-}" ]; then
  test -f "$QAZBOT_SMAC_PREFIX/share/ament_index/resource_index/packages/nav2_smac_planner" || { echo "bad QAZBOT_SMAC_PREFIX" >&2; return 1 2>/dev/null || exit 1; }
  export AMENT_PREFIX_PATH="$QAZBOT_SMAC_PREFIX:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$QAZBOT_SMAC_PREFIX/lib:$LD_LIBRARY_PATH"
fi

# Explicit lifecycle shutdown candidate, isolated from system/vendor packages.
if [ -n "${QAZBOT_LIFECYCLE_PREFIX:-}" ]; then
  test -f "$QAZBOT_LIFECYCLE_PREFIX/share/ament_index/resource_index/packages/nav2_lifecycle_manager" || { echo "bad QAZBOT_LIFECYCLE_PREFIX" >&2; return 1 2>/dev/null || exit 1; }
  export AMENT_PREFIX_PATH="$QAZBOT_LIFECYCLE_PREFIX:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$QAZBOT_LIFECYCLE_PREFIX/lib:$LD_LIBRARY_PATH"
fi
