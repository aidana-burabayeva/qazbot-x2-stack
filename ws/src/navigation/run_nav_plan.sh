#!/usr/bin/env bash
# Plan-only Nav2 (planner + bt_navigator). No controller, no velocity output.
# The global costmap waits for map->base_footprint, so it comes up as soon as
# AMCL has a pose (set it in RViz); starting this earlier is fine.
set -e
QAZBOT_NAV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$QAZBOT_NAV_DIR/env.sh"
exec ros2 launch "$QAZBOT_NAV_DIR/nav_plan.launch.py" robot_radius:=${ROBOT_RADIUS:?set ROBOT_RADIUS (m): body envelope, no default} "$@"
