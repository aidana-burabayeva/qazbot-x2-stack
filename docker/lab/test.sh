#!/usr/bin/env bash
# (container, no network, domain 88, localhost only) colcon test of ws/src.
set -euo pipefail
WS=/workspace/control_backend/ws
set +u; source /opt/ros/humble/setup.bash; source /lab/install/setup.bash; set -u
export QAZBOT_RUNTIME=/lab/deps
nice -n 19 colcon --log-base /lab/log test --build-base /lab/build --install-base /lab/install --base-paths "$WS/src" \
  --test-result-base /lab/test_results --parallel-workers 1 --event-handlers console_direct-
colcon test-result --test-result-base /lab/test_results --verbose
