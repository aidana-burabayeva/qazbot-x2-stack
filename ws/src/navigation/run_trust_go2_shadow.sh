#!/usr/bin/env bash
# Compare against the active trust without granting motion or replacing its publisher.
# Usage: bash run_trust_go2_shadow.sh MAP_YAML [extra ROS arguments]
set -euo pipefail
QAZBOT_NAV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
QAZBOT_COMPARE_MAP="${1:?Usage: run_trust_go2_shadow.sh MAP_YAML [extra ROS arguments]}"
shift
source "$QAZBOT_NAV_DIR/env.sh"
QAZBOT_COMPARE_SHA="$(python3 -c 'import sys; sys.path.insert(0,sys.argv[1]); import pose_store; print(pose_store.map_image_sha256(sys.argv[2]))' "$QAZBOT_NAV_DIR" "$QAZBOT_COMPARE_MAP")"
exec python3 "$QAZBOT_NAV_DIR/localization_trust_node.py" --ros-args \
  --params-file "$QAZBOT_NAV_DIR/profiles/go2_admission.yaml" \
  -p go2_nomotion_on_initialpose:=false \
  -r __node:=localization_trust_go2_shadow \
  -p "map_yaml:=$QAZBOT_COMPARE_MAP" -p "expected_map_sha256:=$QAZBOT_COMPARE_SHA" "$@"
