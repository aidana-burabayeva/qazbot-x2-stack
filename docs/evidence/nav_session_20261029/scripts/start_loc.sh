#!/usr/bin/env bash
# P4b 2026-10-28: passive X2 localization (FAST-LIO, level_bridge publish_tf=false, native_imu bridge owns odom->base_footprint,
# map_server D1 v3, AMCL) + Localization Trust (go2, pose_store off). NO Nav2, NO MC / Motion Adapter / Goal Coordinator, NO cmd_vel.
# ros2 launch is called directly (NOT run_localization.sh: it re-sources env.sh and silently switches to dds_qazbot.xml).
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261029_1425
NAV=/home/agi/control_backend/ws/src/navigation
BIN=/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping
MAP=/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml
MAP_SHA=83d05bafb541b42315a0b03dbbfffa9fc2511025c552b3430da20d6935ae8644
PROF=$EV/dds_nav_two_iface_log.xml
PAT='fastlio_mapping|level_bridge|native_odom_bridge|nav2_amcl/amcl|lib/nav2_map_server/map_server|localization_trust'
log(){ echo "[$(date +%T.%N)] $*" | tee -a $EV/start_run.txt; }
if pgrep -af "$PAT" | grep -v -E "pgrep|start.sh|^ *[0-9]+ /bin/bash -c|ros2 topic (hz|echo) "; then log "ABORT: stack component already running"; exit 3; fi
cd $NAV
set +u; source ./env.sh >/dev/null 2>&1; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
log "profile $PROF $(sha256sum $PROF | cut -c1-16) launch_sha=$(sha256sum $NAV/localization.launch.py | cut -c1-16)"
setsid nohup ros2 launch $NAV/localization.launch.py lio_binary:=$BIN map_yaml:=$MAP lio_cpus:=5,6 seed_pose_file:=/nonexistent/qazbot_seed_disabled.json odom_source:=native_imu > $EV/localization.log 2>&1 < /dev/null &
LSID=$!; echo $LSID > $EV/localization.pid
for i in $(seq 1 240); do grep -q 'Managed nodes are active' $EV/localization.log && grep -q 'native_odom_bridge started' $EV/localization.log && break; grep -qE 'exited with code|Traceback|process has died' $EV/localization.log && { log "localization FAILED"; break; }; sleep 0.5; done
log "localization: managed_active=$(grep -c 'Managed nodes are active' $EV/localization.log) levelled=$(grep -c levelled $EV/localization.log) native=$(grep -c 'native_odom_bridge started' $EV/localization.log)"
setsid nohup bash -c "exec python3 localization_trust_node.py --ros-args -p map_yaml:=$MAP -p expected_map_sha256:=$MAP_SHA -p admission_policy:=go2 -p odom_source:=native_imu -p odom_topic:=/qazbot_native/odom -p jump_xy_max:=1.0 -p jump_yaw_max:=0.5" > $EV/trust.log 2>&1 < /dev/null &
TSID=$!; echo $TSID > $EV/trust.pid
for i in $(seq 1 80); do grep -q 'localization_trust started' $EV/trust.log && break; sleep 0.5; done
log "trust: started=$(grep -c 'localization_trust started' $EV/trust.log)"
sleep 3
ps -eo pid,ppid,sid,etimes,args | awk -v a=$LSID -v b=$TSID '$3==a||$3==b' | cut -c1-400 > $EV/owned_pids.txt
for p in $(awk '{print $1}' $EV/owned_pids.txt); do echo "$p $(tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -E '^(FASTRTPS_DEFAULT_PROFILES_FILE|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION)=' | tr '\n' ' ') shm_threads=$(cat /proc/$p/task/*/comm 2>/dev/null | grep -c dds.shm)"; done > $EV/owned_env.txt
log "owned: $(awk '{print $1}' $EV/owned_pids.txt | tr '\n' ' ')"
