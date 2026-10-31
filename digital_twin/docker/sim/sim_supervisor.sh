#!/usr/bin/env bash
# PID 1 payload of the sim container: bring the OFFICIAL mc + MuJoCo up to a stable stand (verified order, see
# docs/official_sim.md), then supervise both. Exits non-zero if either dies, so the container restarts/fails loudly.
#   mc -> sim (robot falls: mc has no torque yet) -> SetMcAction SD (mc accepts it only once HAL topics exist)
#      -> restart sim only -> robot spawns standing and mc holds it -> SetMcAction LD
set -Eeuo pipefail
python3 /opt/twin-sim/check_authorization.py
exec 9>/run/qazbot/locks/official-mc.lock
flock -n 9 || { echo 'another official MC worker owns the LAB lease' >&2; exit 2; }
LOG=/var/log/twin; mkdir -p "$LOG"
[ "${TWIN_SANDBOX:-}" = 1 ] || { echo "TWIN_SANDBOX!=1: refusing to start" >&2; exit 2; }
# TWIN_MC=r9: the robot's MC v1.1.0-r9-rc9 (aarch64) under qemu-user, seeded into /opt/mc-r9 by sim-init; default: official x86 mc
MCDIR=/opt/mc
if [ "${TWIN_MC:-official}" = r9 ]; then
  MCDIR=/opt/mc-r9/mc
  [ -e "$MCDIR/bin/aima-mc-app-main.real" ] || { mv "$MCDIR/bin/aima-mc-app-main" "$MCDIR/bin/aima-mc-app-main.real"
    :; }
  cp /opt/twin-sim/qemu-mc-wrapper.sh "$MCDIR/bin/aima-mc-app-main"
  mkdir -p "$MCDIR/bin/mc_param/src"   # a real copy: em_run.sh lists robots with find, which does not follow a symlink
  [ -L "$MCDIR/bin/mc_param/src/robot" ] && rm "$MCDIR/bin/mc_param/src/robot"
  [ -d "$MCDIR/bin/mc_param/src/robot" ] || cp -a /opt/mc-r9/param/robot "$MCDIR/bin/mc_param/src/robot"
fi
for f in $MCDIR/bin/aima-mc-app-main /opt/sim/bin/aima-sim-app; do
  [ -x "$f" ] || { echo "missing $f: the official AgiBot artifacts must be mounted (twin/scripts/fetch_vendor.sh)" >&2; exit 2; }
done
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; source /opt/aimdk/setup.bash; set -u   # ROS setup scripts use unset variables
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "${TWIN_GUI:-0}" != 1 ]; then      # headless: virtual framebuffer (software GL); GUI mode uses the host X server
  Xvfb :99 -screen 0 "${TWIN_SIM_RES:-1280x720x24}" -nolisten tcp >"$LOG/xvfb.log" 2>&1 &
  export DISPLAY=:99; sleep 1
fi

# Physical walls from the navigation map (TWIN_WORLD=map, default) or the vendor's flat floor (TWIN_WORLD=flat).
# NB: the vendor start_sim.sh clears the chosen configuration when -s is given together with -c: never combine them.
SIM_ARGS=(-s)
if [ "${TWIN_WORLD:-map}" = map ]; then
  rm -rf /tmp/twin-world
  python3 /opt/twin-sim/world.py --map "/maps/${TWIN_MAP:?TWIN_MAP required}/map.yaml" --start ${TWIN_START//,/ } \
      --robot /opt/sim/configuration/robot/lx2501_3_t2d5 --out /tmp/twin-world >"$LOG/world.json" \
    || { echo "[twin-sim] FAIL: world generation (see $LOG/world.json)" >&2; exit 1; }
  SIM_ARGS=(-c /tmp/twin-world)
fi
MC_PID=; SIM_PID=
start_mc()  { ( cd $MCDIR/bin  && unset AGIBOT_SOFTWARE_HOME && echo 0 | exec ./em_run.sh ) >"$LOG/mc.log"  2>&1 & MC_PID=$!; }
start_sim() { ( cd /opt/sim/bin && echo 0 | exec ./start_sim.sh "${SIM_ARGS[@]}" )                          >"$LOG/sim.log" 2>&1 & SIM_PID=$!; }
stop_sim()  { pkill -f aima-sim-app || true; sleep 3; pkill -9 -f aima-sim-app || true; wait "$SIM_PID" 2>/dev/null || true; }
cleanup()   { kill "$MC_PID" "$SIM_PID" 2>/dev/null || true; pkill -f aima-sim-app 2>/dev/null || true; pkill -f aima-mc-app 2>/dev/null || true; }
trap cleanup EXIT TERM INT

echo "[twin-sim] starting mc"; start_mc; sleep 8
echo "[twin-sim] starting sim (first pass, world=${TWIN_WORLD:-map})"; start_sim
if [ "${TWIN_WORLD:-map}" = map ]; then            # the process must really use the generated configuration
  for _ in $(seq 1 15); do pgrep -af aima-sim-app | grep -q /tmp/twin-world && break; sleep 1; done
  pgrep -af aima-sim-app | grep -q /tmp/twin-world || { echo "[twin-sim] FAIL: sim does not use the generated world" >&2; exit 1; }
fi
for _ in $(seq 1 30); do sleep 2; ros2 topic list 2>/dev/null | grep -q /aima/hal/joint/leg/state && break; done
ok=
for _ in $(seq 1 12); do
  ros2 run py_examples set_mc_action SD 2>&1 | grep -q 'set successfully' && { ok=1; break; }
  sleep 3
done
[ -n "$ok" ] || { echo "[twin-sim] FAIL: SD never accepted (see $LOG/mc.log)" >&2; exit 1; }
sleep 3
echo "[twin-sim] restarting sim with mc holding the stand"; stop_sim; start_sim
twin-sim-health --wait 60 || { echo "[twin-sim] FAIL: robot is not standing" >&2; exit 1; }
ros2 run py_examples set_mc_action LD 2>&1 | grep -q 'set successfully' || { echo "[twin-sim] FAIL: LD" >&2; exit 1; }
echo "[twin-sim] READY: standing, locomotion mode"
wait -n "$MC_PID" "$SIM_PID"
echo "[twin-sim] a core process exited (mc=$MC_PID sim=$SIM_PID)" >&2
exit 1
