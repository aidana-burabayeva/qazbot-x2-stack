#!/usr/bin/env bash
# Isolated model test of tests/s3/step_cmd.py (domain 87, localhost only). Never touches hardware DDS or MC velocity.
set -e
D="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"; R="$(cd -- "$D/../.." && pwd)"
source "$R/ws/src/navigation/env.sh"
source "$R/ws/src/navigation/aimdk_env.sh"      # aimdk_msgs for the adapter and the FAKE McCommonState in domain 87
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$R/tests/nav_e2e/dds_domain87_unicast.xml"
exec python3 "$D/model_test_step_cmd.py" "${1:?Provide a new result directory name}"
