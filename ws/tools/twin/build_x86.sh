#!/usr/bin/env bash
# Reproducible x86 build of what the twin needs from outside the robot's ARM64 runtime:
#   aimdk_msgs (official SDK source) and nav2_smac_planner 1.1.20 + QazBot bounds.patch.
# Output: runtime_x86/build_ws/install (+ Smac manifest checked by nav_full.launch.py). Needs network once (git clone).
set -eo pipefail   # no -u: ROS setup.bash uses unset variables
CB="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
W="$CB/runtime_x86/build_ws"; mkdir -p "$W/src"
source /opt/ros/humble/setup.bash
if [ ! -d "$W/src/aimdk_msgs" ]; then
  rsync -a --exclude='prebuilt_*' "$CB/vendor/official/sdk/aimdk-aarch64-faf4c945-artifacts/src/aimdk_msgs/" "$W/src/aimdk_msgs/"
fi
if [ ! -d "$W/src/smac_only/nav2_smac_planner" ]; then
  tmp="$(mktemp -d)"; git clone -q --depth 1 --branch 1.1.20 https://github.com/ros-navigation/navigation2 "$tmp/nav2"
  mkdir -p "$W/src/smac_only"; cp -r "$tmp/nav2/nav2_smac_planner" "$W/src/smac_only/"; rm -rf "$tmp"
  (cd "$W/src/smac_only" && patch -p1 < "$CB/candidates/nav2_smac_bounds/bounds.patch")
fi
cd "$W" && colcon build --packages-select aimdk_msgs nav2_smac_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
"$CB/ws/tools/twin/make_smac_manifest.sh"
