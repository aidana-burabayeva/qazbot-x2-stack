#!/usr/bin/env bash
# Source the ROS underlay and the twin overlay, then exec the command. Fail on unset variables only AFTER sourcing
# (ROS setup scripts use unset variables).
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/x2twin/overlay/setup.bash
set -u
exec "$@"
