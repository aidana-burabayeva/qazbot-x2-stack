#!/usr/bin/env bash
# D-05 const_cmd dry-run integration against a STUB adapter status publisher in isolated ROS_DOMAIN_ID=79.
# No real adapter, no robot topics, never domain 0 (localhost-only, vendor DDS profile unset).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; TOOL="$HERE/../../tools/d05/const_cmd.py"
set +u; source /opt/ros/humble/setup.bash; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh; set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_CONFIG_FILE
export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1
[ "$ROS_DOMAIN_ID" != 0 ] || exit 9
exec python3 "$HERE/run_dry_integration.py" "$TOOL" "$HERE/stub_adapter.py"
