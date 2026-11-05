#!/usr/bin/env bash
# Localization trust node on the hardware graph (read-only consumer; no command or MC output).
# Pins the served map to the image hash from the map manifest (MAP_DIR, default first_map_nav2_v2 as before);
# extra args are passed as --ros-args (-p "pose_store:=''" disables saving the confirmed pose).
set -e
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.sh"
LOC_LIB="$(ros2 pkg prefix qazbot_localization)/lib/qazbot_localization"
MAPS="$(ros2 pkg prefix --share qazbot_maps)/maps"
MANIFEST="$MAPS/${MAP_DIR:-first_map_nav2_v2}/manifest.json"
SHA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["pgm_sha256"])' "$MANIFEST")"
# Confirmed-pose store (READY rising edge / clean stop), re-seeded by localization.launch.py (seed_pose_file, same default).
STORE="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import pose_store; print(pose_store.DEFAULT_PATH)' "$LOC_LIB")"
exec python3 "$LOC_LIB/localization_trust_node.py" --ros-args -p "map_yaml:=$MAPS/${MAP_DIR:-first_map_nav2_v2}/map.yaml" \
  -p "expected_map_sha256:=$SHA" -p "pose_store:=$STORE" "$@"
