#!/bin/bash
# Run a command inside the twin container with the official AimDK environment. Usage: ex.sh CMD...
source /opt/ros/humble/setup.bash; source /workspace/aimdk/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec "$@"
