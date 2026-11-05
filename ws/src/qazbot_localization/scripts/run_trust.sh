#!/usr/bin/env bash
# Localization trust node on the hardware graph (read-only consumer; no command or MC output).
# Pins the served map to the v2 image hash from the map manifest; extra args are passed as --ros-args
# (-p "pose_store:=''" disables saving the confirmed pose).
set -e
QAZBOT_NAV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$QAZBOT_NAV_DIR/env.sh"
MANIFEST="$QAZBOT_NAV_DIR/../maps/first_map_nav2_v2/manifest.json"
SHA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["pgm_sha256"])' "$MANIFEST")"
# Confirmed-pose store (READY rising edge / clean stop), re-seeded by localization.launch.py (seed_pose_file, same default).
STORE="$(python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); import pose_store; print(pose_store.DEFAULT_PATH)' "$QAZBOT_NAV_DIR")"
exec python3 "$QAZBOT_NAV_DIR/localization_trust_node.py" --ros-args -p "expected_map_sha256:=$SHA" -p "pose_store:=$STORE" "$@"
