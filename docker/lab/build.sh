#!/usr/bin/env bash
# (container, no network) build third-party (patched FAST-LIO, Smac; sources from fetch.sh) + ws/src into /lab/install.
set -euo pipefail
WS=/workspace/control_backend/ws
J="${QAZBOT_LAB_JOBS:-2}"
nice -n 19 "$WS/third_party/build_third_party.sh" --no-fetch --src /lab/tp_src --build-base /lab/build_tp \
  --install-base /lab/install --deps /lab/deps --jobs "$J"
set +u; source /opt/ros/humble/setup.bash; set -u
nice -n 19 colcon --log-base /lab/log build --build-base /lab/build --install-base /lab/install --base-paths "$WS/src" \
  --parallel-workers "$J" --cmake-args -DCMAKE_BUILD_TYPE=Release
