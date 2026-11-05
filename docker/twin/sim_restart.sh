#!/bin/bash
# Host-side. Full restart of the OFFICIAL MC + MuJoCo sim in the isolated twin container and bring the robot to a stable stand.
# Verified order (see docs/digital_twin/official_sim_results.md + sim_steps_20261023.md):
#   MC -> sim (robot falls: MC has no torque yet) -> SetMcAction SD (MC only accepts it once HAL topics exist) -> restart sim only
#   -> robot spawns standing and MC holds it -> LD (locomotion mode).
# Usage: sim_restart.sh TAG   (logs: logs/twin/sim_steps_20261023/logs/TAG_{mc,sim}.log). Prints "READY ..." or "FAIL ..." (exit 1).
C=${QAZBOT_TWIN_CONTAINER:-qazbot-twin-sim}; TAG=${1:-run}
CB="$(cd "$(dirname "$0")/../.." && pwd)"
X="docker exec $C /workspace/control_backend/docker/twin/ex.sh"
D=${QAZBOT_TWIN_LOG_DIR:-/workspace/control_backend/logs/twin/sim_steps_20261023/logs}
mkdir -p "${QAZBOT_TWIN_HOST_LOG_DIR:-$CB/logs/twin/sim_steps_20261023/logs}"
[ "$(docker inspect -f '{{.HostConfig.NetworkMode}}' $C)" = none ] || { echo "FAIL network not none"; exit 1; }
# NB: pkill must be a direct docker exec argument (inside bash -c it would kill its own wrapper)
for p in inside_sim.sh start_sim.sh aima-sim-app inside_mc.sh em_run.sh aima-mc-app-main sim_step_run.py mc_locomotion_v; do docker exec $C pkill -f $p; done
sleep 3
for p in inside_sim.sh start_sim.sh em_run.sh inside_mc.sh aima-sim-app aima-mc-app-main iox-roudi; do docker exec $C pkill -9 -f $p; done
sleep 2
left=$(docker exec $C ps -eo stat,args | awk '$1 !~ /^Z/' | grep -E 'start_sim|em_run|aima-sim|aima-mc|iox-roudi' | grep -v grep | wc -l)
[ "$left" = 0 ] || { echo "FAIL $left stale processes remain"; exit 1; }
start_mc()  { docker exec -d $C bash -c "/workspace/control_backend/docker/twin/inside_mc.sh  > $D/${TAG}_mc.log  2>&1"; }
start_sim() { docker exec -d -e QAZBOT_HEADLESS="${QAZBOT_HEADLESS:-0}" -e QAZBOT_SIM_CONFIG="${QAZBOT_SIM_CONFIG:-}" -e QAZBOT_TWIN_MODEL="${QAZBOT_TWIN_MODEL:-official}" $C bash -c "/workspace/control_backend/docker/twin/inside_sim.sh > $D/${TAG}_sim.log 2>&1"; }
start_mc; sleep 8; start_sim
for i in $(seq 1 30); do sleep 2; $X timeout 6 ros2 topic list 2>/dev/null | grep -q "/aima/hal/joint/leg/state" && break; done
ok=""; for i in $(seq 1 12); do
  $X timeout 30 ros2 run py_examples set_mc_action SD 2>&1 | grep -q 'set successfully' && { ok=1; break; }; sleep 3; done
[ -n "$ok" ] || { echo "FAIL SD never accepted"; exit 1; }
sleep 3
docker exec $C pkill -f aima-sim-app; sleep 3; docker exec $C pkill -9 -f aima-sim-app; sleep 1
start_sim
ok=0; for i in $(seq 1 20); do
  sleep 2; r=$($X python3 /workspace/control_backend/ws/tools/twin/sim_odom_peek.py 1)
  z=$(echo "$r" | sed -n 's/.*z_min=\([-0-9.]*\).*/\1/p')
  if [ -n "$z" ] && awk "BEGIN{exit !($z>0.6)}"; then ok=$((ok+1)); [ $ok -ge 2 ] && break; else ok=0; fi
done
[ $ok -ge 2 ] || { echo "FAIL not standing: $r"; exit 1; }
$X timeout 30 ros2 run py_examples set_mc_action LD 2>&1 | grep -q 'set successfully' || { echo "FAIL LD"; exit 1; }
sleep 4
ok=0
for i in 1 2 3; do
  r=$($X python3 /workspace/control_backend/ws/tools/twin/sim_odom_peek.py 2)
  n=$(echo "$r" | sed -n 's/^n=\([0-9]*\).*/\1/p')
  z=$(echo "$r" | sed -n 's/.*z_min=\([-0-9.]*\).*/\1/p')
  if [ "${n:-0}" -ge 5 ] && [ -n "$z" ] && awk "BEGIN{exit !($z>0.6)}"; then ok=1; break; fi
done
[ "$ok" = 1 ] || { echo "FAIL no standing HAL samples after LD: $r"; exit 1; }
echo "READY $r | $(docker exec $C /workspace/control_backend/docker/twin/state_once.sh)"
