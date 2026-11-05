#!/usr/bin/env bash
# X2 driver adapter entry. Default dry-run; never launches Nav2 or changes posture.
set -e
D="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$D/env.sh"
source "$D/aimdk_env.sh"
exec python3 "$(ros2 pkg prefix qazbot_motion_control)/lib/qazbot_motion_control/x2_motion_adapter.py" --ros-args -r __ns:=/qazbot_d1_map "$@"
