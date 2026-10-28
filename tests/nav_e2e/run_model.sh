#!/usr/bin/env bash
# Isolated model only (domain 87, localhost). Never touches hardware DDS, MC velocity or registration.
# Default: full chain with the REAL x2_motion_adapter (live=false, dry-run shadow) and REAL localization_trust.
# Legacy fixture suite (ideal localization, faked adapter status): run_model.sh --legacy NAME
set -e
QAZBOT_TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
QAZBOT_TEST_ROOT="$(cd -- "$QAZBOT_TEST_DIR/../.." && pwd)"
SUITE=run_suite_real.py
if [ "${1:-}" = "--legacy" ]; then SUITE=run_suite.py; shift; fi
export QAZBOT_SMAC_PREFIX="$QAZBOT_TEST_ROOT/candidates/nav2_smac_bounds/install/nav2_smac_planner"
source "$QAZBOT_TEST_ROOT/ws/src/navigation/env.sh"
# aimdk_msgs only for the adapter's subscription and the FAKE McCommonState inside domain 87
source "$QAZBOT_TEST_ROOT/ws/src/navigation/aimdk_env.sh"
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$QAZBOT_TEST_ROOT/tests/nav_e2e/dds_domain87_unicast.xml"
exec python3 "$QAZBOT_TEST_DIR/$SUITE" "${1:?Provide a new result directory name}"
