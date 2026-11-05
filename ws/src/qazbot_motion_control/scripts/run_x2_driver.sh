#!/usr/bin/env bash
# X2 driver adapter entry. Default dry-run; never launches Nav2 or changes posture.
set -e
QAZBOT_DRIVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$QAZBOT_DRIVER_DIR/env.sh"
source "$QAZBOT_DRIVER_DIR/aimdk_env.sh"
exec python3 "$QAZBOT_DRIVER_DIR/x2_motion_adapter.py" --ros-args -r __ns:=/qazbot_d1_map "$@"
