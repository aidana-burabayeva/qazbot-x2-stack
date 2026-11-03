#!/bin/bash
# Isolated environment for the offline controller comparison (20261103-controller-compare). Nothing system-wide.
#   DWB_PREFIX=<dir from fetch_dwb.sh>/opt/ros/humble ./run.sh matrix [...]   |   ./run.sh single --controller ...
# Domain 78 + localhost unicast profile (never the hardware domain 0), nice 19 / ionice idle.
HERE="$(cd "$(dirname "$0")" && pwd)"
R="$HERE/../../runtime"
DWB_PREFIX="${DWB_PREFIX:-/tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/dwb_prefix/opt/ros/humble}"
source /opt/ros/humble/setup.bash
for P in "$R/qazbot_nav_deps/root/opt/ros/humble" "$R/nav_ext_ws/root/opt/ros/humble" "$DWB_PREFIX"; do
  [ -d "$P" ] || { echo "missing overlay $P" >&2; exit 2; }
  export AMENT_PREFIX_PATH="$P:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$P/lib:$LD_LIBRARY_PATH"
done
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
# dwb_msgs/nav_2d_msgs fastrtps typesupport need libfastcdr.so.1 (host Fast-CDR 2.x only) -> introspection-based RMW.
export RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_domain78_unicast.xml"
export RCUTILS_COLORIZED_OUTPUT=0
mkdir -p "$HERE/results"; cd "$HERE/results"   # fastdds.log lands here
exec nice -n 19 ionice -c3 python3 "$HERE/run_compare.py" "$@"
