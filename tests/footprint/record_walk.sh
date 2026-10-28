#!/bin/bash
# Passive walking-envelope recording (one ROS node, subscribes only). Robot must already stand; operator walks it with the PS5.
# Usage: record_walk.sh NAME SECONDS      -> data/NAME.npz, then prints per-phase envelope
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="$HERE/../../ws/src/navigation"
source /opt/ros/humble/setup.bash
source "$NAV/aimdk_env.sh"
export X2_URDF_DIR=${X2_URDF_DIR:-/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0}
OUT="$HERE/data/$1.npz"
python3 "$HERE/joint_record.py" "$OUT" "${2:-30}" 50
python3 "$HERE/envelope_phases.py" "$OUT"
