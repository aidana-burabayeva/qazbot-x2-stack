#!/usr/bin/env bash
# One sim run for the frame A/B. SIM ONLY. usage: frame_ab.sh <label> <start "x,y,yaw"> <goal args or ""> [fixed_pkg_dir]
set -uo pipefail
LABEL=$1; START=$2; GOALS=${3:-}; FIX=${4:-}
BACKEND=/home/adn/QazBot/control_backend_p20_twin02; TWIN=/home/adn/QazBot/digital_twin; OUT=/home/adn/QazBot/robot_data/ab_frame/out; mkdir -p $OUT
export P20_BACKEND=$BACKEND TWIN_MAP=main_office TWIN_SIM_RES=320x180x24 LP_NUM_THREADS=1 SIM_CPUS=8 STACK_CPUS=4 TWIN_START=$START
OVR=/home/adn/QazBot/robot_data/ab_frame/fix.compose.yaml
if [ -n "$FIX" ]; then printf 'services:\n  stack:\n    volumes:\n      - %s:/opt/x2twin/overlay/lib/python3.10/site-packages/x2_twin_sim:ro\n' "$FIX" > $OVR; else printf 'services: {}\n' > $OVR; fi
C=(docker compose -p x2-twin -f $TWIN/docker/compose.yaml -f $BACKEND/tests/p20_sim/nav2.compose.yaml -f $OVR --env-file $TWIN/.env)
"${C[@]}" down >/dev/null 2>&1
"${C[@]}" up -d --wait --force-recreate sim stack >/dev/null 2>&1 || { echo "stack failed"; "${C[@]}" down >/dev/null 2>&1; exit 3; }
SX=${START%%,*}; REST=${START#*,}; SY=${REST%%,*}; SYAW=${REST#*,}
RUN=docker\ run\ --rm\ --user\ 1000:1000\ --network\ x2-twin_twin\ -e\ ROS_DOMAIN_ID=87\ -e\ TWIN_SANDBOX=1
SECS=${PROBE_SECS:-40}
$RUN -v /home/adn/QazBot/robot_data/ab_frame:/p:ro --entrypoint bash x2-twin-stack:p20 -lc ". /opt/ros/humble/setup.bash; python3 /p/pose_probe.py $SECS $SX $SY $SYAW" > $OUT/$LABEL.csv 2>$OUT/$LABEL.err &
PROBE=$!
if [ -n "$GOALS" ]; then
  docker run --rm --user 1000:1000 --network x2-twin_twin --entrypoint bash -e TWIN_SANDBOX=1 -e ROS_DOMAIN_ID=87 -e TWIN_STACK_DIR=/opt/x2twin/stack -e TWIN_RESULTS_DIR=/results \
    -v $BACKEND:/backend:ro -v $TWIN/results:/results -v /tmp/qazbot-p20-build/timing:/timing/build:ro -v /home/adn/QazBot/control_backend/ws/src/maps/main_office:/opt/x2twin/stack/maps/main_office:ro \
    x2-twin-stack:p20 -lc ". /opt/ros/humble/setup.bash; . /opt/x2twin/overlay/setup.sh; python3 /backend/tests/p20_sim/nav2_goals.py --out frameab_${LABEL}_$$ $GOALS" > $OUT/$LABEL.goals.log 2>&1
fi
wait $PROBE
"${C[@]}" down >/dev/null 2>&1
echo "done $LABEL rows=$(($(wc -l < $OUT/$LABEL.csv)-1))"
