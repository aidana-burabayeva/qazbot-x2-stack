#!/usr/bin/env bash
# Experimental controller, x86 SIM only. Does not start or move a plant.
set -euo pipefail
C=${QAZBOT_TWIN_CONTAINER:-qazbot-twin-sim}
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$C")" = none ]
docker exec -e QAZBOT_SIM_MC_TURN=0 "$C" /workspace/control_backend/docker/twin/env_container.sh bash -c '
colcon --log-base /workspace/control_backend/runtime_x86/turn_ws/log build \
 --base-paths /workspace/control_backend/ws/tools/twin/mc_turn_controller \
 --build-base /workspace/control_backend/runtime_x86/turn_ws/build \
 --install-base /workspace/control_backend/runtime_x86/turn_ws/install \
 --cmake-args -DCMAKE_BUILD_TYPE=Release'
