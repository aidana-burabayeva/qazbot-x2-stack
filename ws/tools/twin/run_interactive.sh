#!/usr/bin/env bash
# All ROS nodes, including RViz, stay inside the network-none twin container.
set -euo pipefail
CB="$(cd -P "$(dirname "$0")/../../.." && pwd)"
C=${QAZBOT_TWIN_CONTAINER:-qazbot-twin-sim}
[[ "$C" =~ ^[a-zA-Z0-9_.-]+$ ]] || { echo 'invalid container name' >&2;exit 2; }
plant=kinematic
args=("$@")
for ((i=0;i<${#args[@]};i++)); do
  if [ "${args[i]}" = --plant ]; then plant="${args[i+1]:?--plant needs a value}"; fi
done
if [ "${QAZBOT_SIM_MC_TURN:-0}" = 1 ] && [ "$plant" != official ]; then echo "MC turn candidate requires --plant official" >&2; exit 2; fi
case "$plant" in official|kinematic) ;; *) echo 'plant must be official or kinematic' >&2;exit 2;; esac
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$C")" = none ] || { echo 'twin must have network=none' >&2;exit 1; }
[ "$(docker inspect -f '{{.HostConfig.IpcMode}}' "$C")" = private ] || { echo 'twin requires private IPC' >&2;exit 1; }
mkdir -p "$CB/runtime_x86"
lock=interactive.lock
[ "$C" = qazbot-twin-sim ] || lock="${C}.interactive.lock"
exec 9>"$CB/runtime_x86/$lock"
flock -n 9 || { echo 'interactive session already running' >&2;exit 1; }
# Never replace an ongoing measurement or an interactive stack.
if docker exec "$C" pgrep -f 'test_live_chain.py|interactive_navigation.py|sim_step_run.py|mc_turn_after_walk.py' >/dev/null; then
  echo 'twin is busy: finish its current session first' >&2;exit 1
fi
if [ "$plant" = official ]; then
  if [ ! -f "$CB/runtime_x86/rviz_map_v3/configuration/manifest.json" ]; then
    python3 "$CB/ws/tools/twin/make_map_world.py" "$CB/ws/src/maps/first_map_nav2_v3/map.yaml" "$CB/runtime_x86/rviz_map_v3/configuration"
  fi
  QAZBOT_TWIN_CONTAINER="$C" QAZBOT_TWIN_MODEL=map_world QAZBOT_SIM_CONFIG=/workspace/control_backend/runtime_x86/rviz_map_v3/configuration \
    "$CB/docker/twin/sim_restart.sh" "rviz_$(date +%Y%m%d_%H%M%S)"
else
  # Virtual MC has the same service/topic names; only one MC may own this isolated graph.
  for pattern in inside_sim.sh start_sim.sh aima-sim-app inside_mc.sh em_run.sh aima-mc-app-main; do
    docker exec "$C" pkill -f "$pattern" || true
  done
  sleep 2
  for pattern in aima-sim-app aima-mc-app-main iox-roudi; do docker exec "$C" pkill -9 -f "$pattern" || true;done
fi
# docker exec without a TTY does not reliably forward Ctrl+C to its Python child.
cleanup() { docker exec "$C" pkill -INT -f '^python3 /workspace/control_backend/ws/tools/twin/interactive_navigation.py( |$)' >/dev/null 2>&1 || true; }
trap cleanup EXIT
extra_env=()
if [ -n "${QAZBOT_SIM_MC_LINEAR_DECEL:-}" ]; then extra_env=(-e "QAZBOT_SIM_MC_LINEAR_DECEL=$QAZBOT_SIM_MC_LINEAR_DECEL"); fi
docker exec -i "${extra_env[@]}" -e QAZBOT_SIM_DDS_UDP_ONLY="${QAZBOT_SIM_DDS_UDP_ONLY:-0}" -e QAZBOT_SIM_MC_WALK="${QAZBOT_SIM_MC_WALK:-0}" -e QAZBOT_SIM_MC_WALK_LEAD="${QAZBOT_SIM_MC_WALK_LEAD:-0.45}" -e QAZBOT_SIM_POSE_REFINE="${QAZBOT_SIM_POSE_REFINE:-0}" -e QAZBOT_SIM_MC_TURN="${QAZBOT_SIM_MC_TURN:-0}" -e QAZBOT_SIM_EXACT_GOAL="${QAZBOT_SIM_EXACT_GOAL:-0}" -e LIBGL_ALWAYS_SOFTWARE=true -e LIBGL_DRI3_DISABLE=1 -e QT_X11_NO_MITSHM=1 "$C" \
  /workspace/control_backend/docker/twin/env_container.sh python3 \
  /workspace/control_backend/ws/tools/twin/interactive_navigation.py "${args[@]}"
