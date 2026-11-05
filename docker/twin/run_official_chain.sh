#!/bin/bash
# Host-side: official mc + MuJoCo (stand) -> QazBot stack (real Nav2/Trust/Coordinator/AMCL/adapter live) inside the container.
# Usage: run_official_chain.sh RESULT_NAME     Result: tests/integration_results/RESULT_NAME/results.json
set -e
CB="$(cd "$(dirname "$0")/../.." && pwd)"
"$CB/docker/twin/sim_restart.sh" "pre_${1:?RESULT_NAME}" | tee /dev/stderr | grep -q '^READY' || { echo "sim not ready"; exit 1; }
docker exec -i -e QAZBOT_TWIN_MODEL="${QAZBOT_TWIN_MODEL:-official}" -e QAZBOT_SENSOR_PROFILE="${QAZBOT_SENSOR_PROFILE:-ideal}" -e QAZBOT_TRUST_POLICY="${QAZBOT_TRUST_POLICY:-go2}" -e QAZBOT_PLANT=official -e QAZBOT_MAX_ANGULAR="${QAZBOT_MAX_ANGULAR:-0.5}" qazbot-twin-sim \
  /workspace/control_backend/docker/twin/env_container.sh python3 /workspace/control_backend/ws/tests/twin/test_live_chain.py "$1"
