#!/usr/bin/env bash
# Build only the X86 simulation candidate, never a robot installation.
set -euo pipefail
CB="$(cd -P "$(dirname "$0")/../../.." && pwd)"
C=${QAZBOT_TWIN_CONTAINER:-qazbot-twin-sim}
[[ "$C" =~ ^[a-zA-Z0-9_.-]+$ ]] || exit 2
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$C")" = none ] || { echo 'network=none required' >&2;exit 2; }
mkdir -p "$CB/runtime_x86"
exec 9>"$CB/runtime_x86/pose_refine_build.lock";flock 9
if python3 "$CB/ws/tools/twin/pose_refine_manifest.py" >/dev/null 2>&1; then exit 0;fi
library="$CB/runtime_x86/pose_refine_ws/install/qazbot_pose_refine/lib/libqazbot_pose_refine.so"
if [ -f "$library" ]; then
  command -v fuser >/dev/null || { echo 'fuser required before rebuilding an existing plugin' >&2;exit 2; }
  if fuser -s "$library"; then echo 'Finish active refinement tests before rebuilding their library' >&2;exit 2;fi
fi
docker exec -i -e QAZBOT_SIM_POSE_REFINE=0 -e QAZBOT_SIM_MC_TURN=0 "$C" \
  /workspace/control_backend/docker/twin/env_container.sh bash <<'BUILD'
set -e
cd /workspace/control_backend
mkdir -p runtime_x86/pose_refine_ws/src
target=runtime_x86/pose_refine_ws/src/qazbot_pose_refine
if [ ! -e "$target" ]; then ln -s /workspace/control_backend/candidates/nav2_pose_refine "$target";fi
[ "$(readlink -f "$target")" = /workspace/control_backend/candidates/nav2_pose_refine ] || exit 2
cd runtime_x86/pose_refine_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select qazbot_pose_refine \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
python3 /workspace/control_backend/ws/tools/twin/pose_refine_manifest.py --write
BUILD
