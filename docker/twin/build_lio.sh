#!/usr/bin/env bash
# Build portable FAST-LIO for x86, using the single-thread search path selected on ARM .66.
# The upstream CMake enables MP_EN on x86; undefine it in release flags without editing vendor sources.
set -euo pipefail
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' qazbot-twin-sim)" = none ]
docker exec qazbot-twin-sim /workspace/control_backend/tools/twin/docker/env_container.sh bash -c '
colcon --log-base /workspace/control_backend/runtime_x86/lio_ws/log build \
  --base-paths /workspace/control_backend/candidates/fast_lio_portable/src \
  --build-base /workspace/control_backend/runtime_x86/lio_ws/build \
  --install-base /workspace/control_backend/runtime_x86/lio_ws/install \
  --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_LIVOX=OFF -DBUILD_TESTING=OFF \
  "-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -UMP_EN -UMP_PROC_NUM -DMP_PROC_NUM=1"'
