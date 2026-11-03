#!/usr/bin/env bash
# Starts native_odom_bridge.py with the aimdk_msgs overlay ONLY for this process (not for FAST-LIO/AMCL).
# Usage: run_native_odom_bridge.sh <bridge.py> [--ros-args ...]
set -e
NAV_DIR="${QAZBOT_NAV_DIR:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"   # deployed: its own dir (next to env.sh)
source "$NAV_DIR/env.sh"
source "$NAV_DIR/aimdk_env.sh"
exec python3 "$@"
