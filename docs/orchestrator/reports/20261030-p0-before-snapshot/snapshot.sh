#!/usr/bin/env bash
# Read-only runtime snapshot of the X2 (no ROS/DDS participant is created, no sudo, no writes outside OUT).
# Usage: snapshot.sh [OUT_DIR]   (default: <script dir>/run_<timestamp>)
# Diff two runs:  diff -r <run_A> <run_B>   (files are sorted, volatile fields such as etime are kept in separate files)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-$HERE/run_$(date +%Y%m%dT%H%M%S)}"
mkdir -p "$OUT"
date -Is > "$OUT/00_time.txt"; uptime >> "$OUT/00_time.txt"; cat /proc/sys/kernel/random/boot_id >> "$OUT/00_time.txt"
TRANS='snapshot.sh|claude|shell-snapshots|vscode-server|sleep [0-9]|^ *[0-9]+ +[0-9]+ +[a-z]+ +(ps|awk|sort|sha256sum|tr|grep|ugrep|cut|readlink|cat|date) |atop -w|gpioset --mode=time'
# 1) full non-kernel process list (stable fields only)
ps -eo pid,ppid,user,lstart,args --sort=pid | grep -v -E '\]$' | grep -v -E "$TRANS" | cut -c1-600 > "$OUT/10_processes.txt"
# 2) per-process exe/cwd/env-hash for robot/ROS-related processes
PAT='ros2|python|launch|fastlio|lio|amcl|nav2|trust|level_bridge|native_odom|adapter|goal_coordinator|map_server|lifecycle|controller_server|planner_server|bt_navigator|velocity_smoother|component_container|static_transform|aima|agibot|hal_|em-server|slam|perception|vectorflux|auto_integrated|map_manager|housekeeper|webrtc|docker|containerd|podman|qazbot'
{
for p in $(ps -eo pid=,args= | grep -E "$PAT" | grep -v -E "$TRANS" | awk '{print $1}'); do
  [ -d /proc/$p ] || continue
  u=$(ps -o user= -p $p 2>/dev/null); exe=$(readlink /proc/$p/exe 2>/dev/null || echo EPERM); cwd=$(readlink /proc/$p/cwd 2>/dev/null || echo EPERM)
  if [ -r /proc/$p/environ ]; then
    envsel=$(tr '\0' '\n' < /proc/$p/environ | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH|LD_LIBRARY_PATH|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE|FASTDDS_DEFAULT_PROFILES_FILE|ROS_LOCALHOST_ONLY)=' | sort)
    eh=$(printf '%s' "$envsel" | sha256sum | cut -c1-16)
  else envsel="EPERM"; eh="EPERM"; fi
  echo "PID=$p USER=$u EXE=$exe CWD=$cwd ENVHASH=$eh"
  echo "  CMD=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-500)"
  [ "$envsel" != EPERM ] && printf '%s\n' "$envsel" | sed 's/^/  ENV /'
done
} > "$OUT/20_proc_detail.txt"
# 3) systemd state
systemctl list-units --type=service --all --no-pager --plain --no-legend 2>/dev/null | awk '{print $1,$2,$3,$4}' | sort > "$OUT/30_systemd_system_services.txt"
systemctl --user list-units --type=service --all --no-pager --plain --no-legend 2>/dev/null | awk '{print $1,$2,$3,$4}' | sort > "$OUT/31_systemd_user_services.txt"
systemctl --user show qazbot-speech.service -p NRestarts -p ActiveState -p SubState --no-pager > "$OUT/32_volatile_speech_restarts.txt" 2>&1
{ systemctl list-timers --all --no-pager --plain --no-legend 2>/dev/null | awk '{print $NF}'; systemctl --user list-timers --all --no-pager --plain --no-legend 2>/dev/null | awk '{print "user:"$NF}'; } | sort > "$OUT/33_timers.txt"
ls -la ~/.config/systemd/user ~/.config/systemd/user/*.wants > "$OUT/34_user_unit_files.txt" 2>&1
# 4) cron, shell init, docker/podman (read-only)
{ crontab -l 2>&1; ls -la /etc/cron.d 2>&1; } > "$OUT/40_cron.txt"
{ sha256sum ~/.bashrc ~/.profile ~/.aima/env/bashrc 2>&1; grep -n -E 'source|^\s*\. ' ~/.bashrc ~/.profile 2>/dev/null; } > "$OUT/41_shell_init.txt"
{ systemctl is-active docker containerd 2>&1; docker ps -a --no-trunc 2>&1; podman ps -a 2>&1; } > "$OUT/42_containers.txt"
# 5) deployment / checkout layout (no hashing of trees; that is A2's job)
{ ls -la /home/agi/control_backend /home/agi/control_backend/ws /home/agi/control_backend/run /home/agi/control_backend/runtime 2>&1; cat /home/agi/control_backend/run/x2_motion_owner.lock 2>&1; } > "$OUT/50_deploy_layout.txt"
git -C /home/agi/control_backend/qazbot-x2-stack rev-parse --abbrev-ref HEAD > "$OUT/51_checkout_head.txt" 2>&1; git -C /home/agi/control_backend/qazbot-x2-stack rev-parse HEAD >> "$OUT/51_checkout_head.txt" 2>&1
git --no-optional-locks -C /home/agi/control_backend/qazbot-x2-stack status --porcelain=v1 --untracked-files=no >> "$OUT/51_checkout_head.txt" 2>&1
ls -d /home/agi/control_backend/.git > "$OUT/52_deploy_no_git.txt" 2>&1
# 6) ROS daemon / graph: NOT queried (no DDS participant). Record whether a ros2 daemon exists.
pgrep -af '_ros2_daemon|ros2-daemon' | grep -v -E "$TRANS" > "$OUT/60_ros2_daemon.txt" || echo "no ros2 daemon" > "$OUT/60_ros2_daemon.txt"
echo "$OUT"
