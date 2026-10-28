#!/usr/bin/env bash
# Usage: run_config.sh EVDIR TAG PROFILE_XML  -- passive baseline localization (native_imu) + Trust, probe, stop owned PIDs.
set -u
EV=$1; TAG=$2; PROF=$3
NAV=/home/agi/control_backend/ws/src/navigation
BIN=/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping
MAP=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml
MAP_SHA=f6032f75ffd386361993a4b4864cf0816713fc27f6d037c88d929ae5d48562fb
PAT='fastlio_mapping|level_bridge|native_odom_bridge|nav2_amcl/amcl|lib/nav2_map_server/map_server|nav2_lifecycle_manager/lifecycle_manager|localization_trust|goal_coordinator|x2_motion_adapter|controller_server|planner_server|costmap|age_probe|chain_probe|ros2 bag'
log(){ echo "[$(date +%T.%N)] $*" | tee -a $EV/run_$TAG.txt; }
if pgrep -af "$PAT" | grep -v -E "pgrep|run_config"; then log "ABORT: stack component already running"; exit 3; fi
n=$(pgrep -fc 'hal_sensor_orin_t2d5.yaml --process_name=hal_sensor_orin$' ); [ "$n" = 1 ] || { log "ABORT: HAL count $n"; pgrep -af hal_sensor_orin; exit 4; }
cd $NAV
set +u; source ./env.sh >/dev/null 2>&1; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
log "profile $PROF $(sha256sum $PROF | cut -c1-16)"
setsid nohup ros2 launch $NAV/localization.launch.py lio_binary:=$BIN map_yaml:=$MAP lio_cpus:=5,6 seed_pose_file:=/nonexistent/qazbot_seed_disabled.json odom_source:=native_imu > $EV/localization_$TAG.log 2>&1 < /dev/null &
LSID=$!; echo $LSID > $EV/localization_$TAG.sid
for i in $(seq 1 180); do grep -q 'Managed nodes are active' $EV/localization_$TAG.log && grep -q 'native_odom_bridge started' $EV/localization_$TAG.log && break; grep -qE 'exited with code|Traceback|process has died' $EV/localization_$TAG.log && { log "localization failed"; break; }; sleep 0.5; done
log "localization ready: $(grep -c 'Managed nodes are active' $EV/localization_$TAG.log) levelled=$(grep -c levelled $EV/localization_$TAG.log)"
setsid nohup bash -c "exec python3 localization_trust_node.py --ros-args -p map_yaml:=$MAP -p expected_map_sha256:=$MAP_SHA -p admission_policy:=go2 -p odom_source:=native_imu -p odom_topic:=/qazbot_native/odom" > $EV/trust_$TAG.log 2>&1 < /dev/null &
TSID=$!; echo $TSID > $EV/trust_$TAG.sid
for i in $(seq 1 80); do grep -q 'localization_trust started' $EV/trust_$TAG.log && break; sleep 0.5; done
sleep 8
ps -eo pid,ppid,sid,etimes,args | awk -v a=$LSID -v b=$TSID '$3==a||$3==b' | cut -c1-300 > $EV/owned_pids_$TAG.txt
for p in $(awk '{print $1}' $EV/owned_pids_$TAG.txt); do echo "$p $(tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep FASTRTPS_DEFAULT) threads_shm=$(cat /proc/$p/task/*/comm 2>/dev/null | grep -c dds.shm)"; done > $EV/owned_env_$TAG.txt
log "owned: $(awk '{print $1}' $EV/owned_pids_$TAG.txt | tr '\n' ' ')"
P_LIO=$(pgrep -x fastlio_mapping | head -1); P_LB=$(pgrep -f 'lio/level_bridge.py' | head -1); P_AMCL=$(pgrep -f 'nav2_amcl/amcl' | head -1)
P_NAT=$(pgrep -f native_odom_bridge.py | head -1); P_TR=$(pgrep -f localization_trust_node.py | grep -v bash | tail -1); P_MS=$(pgrep -f 'map_server/map_server' | head -1)
echo "lio=$P_LIO level_bridge=$P_LB amcl=$P_AMCL native=$P_NAT trust=$P_TR map_server=$P_MS hal=204720" | tee $EV/key_pids_$TAG.txt
timeout 20 ros2 topic info -v /aima/hal/sensor/lidar_chest_front/lidar_pointcloud > $EV/topicinfo_raw_$TAG.txt 2>&1
timeout 20 ros2 topic info -v /aima/hal/sensor/lidar_chest_front/imu > $EV/topicinfo_imu_$TAG.txt 2>&1
timeout 20 ros2 topic info -v /qazbot_lio/scan > $EV/topicinfo_scan_$TAG.txt 2>&1
snap(){ # $1 label
  { echo "== $1 $(date +%s.%N)"; grep -A1 '^Udp:' /proc/net/snmp | tail -1;
    for p in $P_LIO $P_LB $P_AMCL $P_NAT $P_TR 204720; do for t in /proc/$p/task/*; do echo "sched $p $(basename $t) $(cat $t/comm 2>/dev/null | tr ' ' _) $(cat $t/schedstat 2>/dev/null) $(grep -E 'nonvoluntary' $t/status 2>/dev/null | awk '{print $2}')"; done; done
    for p in $P_LIO $P_LB $P_AMCL $P_NAT $P_TR; do for ino in $(ls -l /proc/$p/fd 2>/dev/null | grep -o 'socket:\[[0-9]*\]' | tr -dc '0-9\n'); do awk -v i=$ino -v p=$p 'NR>1 && $10==i {print "sock", p, $2, $5, $13}' /proc/net/udp; done; done; } >> $EV/snap_$TAG.txt
}
run_probe(){ # $1 name $2 dur $3 extra
  snap "start_$1"
  pidstat -u -h -p $P_LIO,$P_LB,$P_AMCL,$P_NAT,$P_TR,$P_MS,204720 1 $2 > $EV/pidstat_$1_$TAG.txt 2>&1 &
  PS1=$!
  pidstat -t -u -h -p $P_LIO,$P_LB 1 $2 > $EV/pidstat_threads_$1_$TAG.txt 2>&1 &
  PS2=$!
  top -b -d 1 -n $2 -w 220 > $EV/top_$1_$TAG.txt 2>&1 &
  PS3=$!
  timeout $(( $2 + 5 )) tegrastats --interval 1000 > $EV/tegrastats_$1_$TAG.txt 2>&1 &
  PS4=$!
  log "probe $1 start"
  python3 $EV/chain_probe.py $2 $EV/probe_$1_$TAG.jsonl $3 >> $EV/run_$TAG.txt 2>&1
  log "probe $1 end"
  wait $PS1 $PS2 $PS3; kill $PS4 2>/dev/null
  snap "end_$1"
}
run_probe full 120 --raw-cloud
run_probe noraw 60 ""
# stop owned only: trust then launch (SIGINT), then TERM leftovers among owned PIDs
OWNED=$(awk '{print $1}' $EV/owned_pids_$TAG.txt)
log "stopping trust (SIGINT $P_TR)"; kill -INT $P_TR 2>/dev/null; sleep 3
LPID=$LSID; log "SIGINT launch $LPID"; kill -INT $LPID 2>/dev/null
for i in $(seq 1 60); do left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done; [ -z "$left" ] && break; sleep 0.5; done
if [ -n "$left" ]; then log "SIGINT leftovers:$left"; kill -INT $left 2>/dev/null; sleep 6; left2=""; for p in $left; do kill -0 $p 2>/dev/null && left2="$left2 $p"; done; left=$left2; fi
if [ -n "$left" ]; then log "TERM leftovers:$left $(ps -o pid,args -p ${left// /,} | tail -n +2 | cut -c1-120 | tr '\n' ';')"; kill -TERM $left 2>/dev/null; sleep 3; fi
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
if [ -n "$left" ]; then for p in $left; do echo "== $p"; for t in /proc/$p/task/*; do echo "$(basename $t) $(cat $t/comm) $(cat $t/wchan)"; done; done > $EV/stuck_shutdown_$TAG.txt; log "KILL owned deadlocked (INT+TERM ignored):$left"; kill -KILL $left; sleep 2; fi
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
log "after stop leftovers:[${left}] stack_procs:[$(pgrep -af "$PAT" | grep -v -E 'pgrep|run_config' | cut -c1-80 | tr '\n' ';')] hal:[$(pgrep -f 'process_name=hal_sensor_orin$' | tr '\n' ' ')]"
