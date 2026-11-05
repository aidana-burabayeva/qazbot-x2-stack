#!/usr/bin/env bash
set -e
QAZBOT_NAV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$QAZBOT_NAV_DIR/env.sh"
exec ros2 launch "$QAZBOT_NAV_DIR/localization.launch.py" "$@"
