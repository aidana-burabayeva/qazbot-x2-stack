#!/usr/bin/env bash
# Create ONE isolated research container (official mc + MuJoCo runtime; image qazbot-twin-sim:humble). Network: none.
# Usage: run_container.sh [NAME] [-e KEY=VAL ...]        (default NAME: qazbot-twin-sim)
#
# Layout contract (after the 2026-10-27 relocation): the research root (digital_twin/research) is mounted as
# /workspace/control_backend, because every script here resolves its ROOT as parents[2] and uses that container path;
# the real navigation stack (control_backend) is mounted read-write at /control_backend, which is where the research root's
# relative mirror links ("../../../control_backend/...") land inside the container.
set -euo pipefail
NAME="${1:-qazbot-twin-sim}"; [ $# -gt 0 ] && shift
IMAGE=${QAZBOT_TWIN_IMAGE:-qazbot-twin-sim:humble}
R="$(cd -P "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"       # also resolve the old compatibility symlink
BACKEND="$(cd "$R/../../control_backend" && pwd)"
mkdir -p "$R/runtime_x86/run/agibot_data/var" "$R/runtime_x86/run/aimdk"
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --init --name "$NAME" --network none --ipc private --shm-size "${QAZBOT_TWIN_SHM_SIZE:-64m}" --user "$(id -u):$(id -g)" -e HOME=/tmp/home \
  -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/.Xauthority -v "$HOME/.Xauthority:/tmp/.Xauthority:ro" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=87 -e ROS_LOCALHOST_ONLY=1 -e QAZBOT_SIM=1 "$@" \
  -v "$R/runtime_x86/run/agibot_data:/agibot/data" -v "$R/runtime_x86/run/aimdk:/workspace/aimdk" \
  -v "$R/vendor/official/mc/mc-x86-v1.0.0-20260522:/workspace/mc" -v "$R/vendor/official/sim/sim_mujoco-x86-v1.0.0-20260522:/workspace/sim_mujoco" \
  -v "$R:/workspace/control_backend" -v "$BACKEND:/control_backend" \
  -w /workspace "$IMAGE" sleep infinity
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$NAME")" = none ] && echo "$NAME up, network=none"
