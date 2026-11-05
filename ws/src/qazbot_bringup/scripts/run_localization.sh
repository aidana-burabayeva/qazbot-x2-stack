#!/usr/bin/env bash
# Localization only (FAST-LIO + C++ projector + map_server + AMCL). Extra args go to ros2 launch.
set -e
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.sh"
exec ros2 launch qazbot_localization localization.launch.py "$@"
