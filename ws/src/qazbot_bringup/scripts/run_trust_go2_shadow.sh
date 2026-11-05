#!/usr/bin/env bash
# Compare against the active trust without granting motion or replacing its publisher.
# Usage: bash run_trust_go2_shadow.sh MAP_YAML [extra ROS arguments]
set -euo pipefail
QAZBOT_COMPARE_MAP="${1:?Usage: run_trust_go2_shadow.sh MAP_YAML [extra ROS arguments]}"
shift
set +u; source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.sh"; set -u
LOC_LIB="$(ros2 pkg prefix qazbot_localization)/lib/qazbot_localization"
LOC_SHARE="$(ros2 pkg prefix --share qazbot_localization)"
QAZBOT_COMPARE_SHA="$(python3 -c 'import sys; sys.path.insert(0,sys.argv[1]); import pose_store; print(pose_store.map_image_sha256(sys.argv[2]))' "$LOC_LIB" "$QAZBOT_COMPARE_MAP")"
exec python3 "$LOC_LIB/localization_trust_node.py" --ros-args \
  --params-file "$LOC_SHARE/config/go2_admission.yaml" \
  -p go2_nomotion_on_initialpose:=false \
  -r __node:=localization_trust_go2_shadow \
  -p "map_yaml:=$QAZBOT_COMPARE_MAP" -p "expected_map_sha256:=$QAZBOT_COMPARE_SHA" "$@"
