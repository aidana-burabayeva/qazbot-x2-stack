set -u
#!/usr/bin/env bash
# P27 2026-10-30: go2 T5 re-anchor params (reanchor window 1.5 s, sigma<=.35 score>=.35 2nd<=.30, hard 1.0 m/0.5 rad); backup *.bak_20261030_pre_p27
# P26 2026-10-30: Trust T5 1.0/0.5 -> 0.30 m/0.25 rad, go2 quality pause + READY seed gate params, amcl_update_min coupled to AMCL 0.20/0.15 (backup *.bak_20261030_pre_p26)
# P12 2026-10-28: restart Localization Trust ONLY (same command as start.sh, incl. T5 0.25 m / 0.25 rad operator override).
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
MAP=/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml
MAP_SHA=72ee4094b394241e7b2761878774efbdd99e93731d7cb52709b2231c3975b354
PROF=$EV/dds_nav_two_iface_log.xml
log(){ echo "[$(date +%T.%N)] restart_trust: $*" | tee -a $EV/start_run.txt; }
mv $EV/trust.log $EV/trust_pre_restart_$(date +%H%M%S).log
cd $NAV
set +u; source ./env.sh >/dev/null 2>&1; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
setsid nohup bash -c "exec python3 localization_trust_node.py --ros-args -p map_yaml:=$MAP -p expected_map_sha256:=$MAP_SHA -p admission_policy:=go2 -p odom_source:=native_imu -p odom_topic:=/qazbot_native/odom -p jump_xy_max:=0.30 -p jump_yaw_max:=0.25 -p amcl_update_min_d:=0.20 -p amcl_update_min_a:=0.15 -p quality_pause:=true -p q_sigma_xy_max:=0.35 -p q_second_mode_max:=0.30 -p q_scan_score_min:=0.20 -p q_persist_s:=1.0 -p ready_gate:=true -p ready_sigma_xy_max:=0.25 -p ready_second_mode_max:=0.15 -p ready_scan_score_min:=0.35 -p reanchor:=true -p reanchor_window_s:=1.5 -p reanchor_sigma_xy_max:=0.35 -p reanchor_scan_score_min:=0.35 -p reanchor_second_mode_max:=0.30 -p jump_xy_hard_max:=1.0 -p jump_yaw_hard_max:=0.5" > $EV/trust.log 2>&1 < /dev/null &
TSID=$!; echo $TSID > $EV/trust.pid
for i in $(seq 1 80); do grep -q 'localization_trust started' $EV/trust.log && break; sleep 0.5; done
log "trust: pid $TSID started=$(grep -c 'localization_trust started' $EV/trust.log) env=$(tr '\0' '\n' < /proc/$TSID/environ | grep -E '^(FASTRTPS_DEFAULT_PROFILES_FILE|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY)=' | tr '\n' ' ')"
