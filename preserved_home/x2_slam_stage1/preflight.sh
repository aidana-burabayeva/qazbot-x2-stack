#!/usr/bin/env bash
# Stage 1 read-only preflight. Sequential ros2 CLI calls, sleep 1 between them.
# Topics/services: https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html ;
# lidar/imu topics: /home/agi/aimdk (discovery report), /agibot/info/robot_mode: discovery report T1.
# Publishes nothing, calls no services.
LIDAR=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
IMU=/aima/hal/sensor/lidar_chest_front/imu
SRV=/aimdk_5Fmsgs/srv/GetStoredMapByName
CMD=/integrated_command
MODE=/agibot/info/robot_mode
FAIL=0; WARN=0
res() { echo "[$1] $2"; [ "$1" = "NO-GO" ] && FAIL=1; [ "$1" = "WARN" ] && WARN=1; }
step() { sleep 1; }

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /home/agi/aimdk/install/setup.bash ] && source /home/agi/aimdk/install/setup.bash

echo "== env =="
echo "ROS_DISTRO=${ROS_DISTRO:-unset} RMW=${RMW_IMPLEMENTATION:-default} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0(default)}"
echo "host=$(hostname) ip=$(hostname -I 2>/dev/null)"
[ "${ROS_DISTRO:-}" = humble ] && res GO "distro humble" || res NO-GO "ROS_DISTRO is not humble"

echo "== aimdk_msgs =="
if python3 -c "import aimdk_msgs.srv as s; s.GetStoredMapByName" 2>/dev/null; then
  res GO "aimdk_msgs importable (GetStoredMapByName type present)"
else
  res NO-GO "aimdk_msgs not sourced (source /home/agi/aimdk/install/setup.bash)"
fi

hz() { # topic
  timeout 10 ros2 topic hz --no-daemon "$1" 2>&1 | grep -m1 "average rate" ; }
echo "== lidar =="; step
out=$(hz $LIDAR); [ -n "$out" ] && res GO "lidar $out" || res NO-GO "no lidar messages in 10 s on $LIDAR"
echo "== imu =="; step
out=$(hz $IMU); [ -n "$out" ] && res GO "imu $out" || res NO-GO "no IMU messages in 10 s on $IMU"

echo "== $CMD =="; step
info=$(timeout 15 ros2 topic info --no-daemon -v $CMD 2>&1)
echo "$info"
subs=$(echo "$info" | sed -n 's/^Subscription count: *//p' | head -1)
if [ -n "$subs" ] && [ "$subs" -ge 1 ] 2>/dev/null; then res GO "$CMD has $subs subscriber(s)"
else res NO-GO "$CMD has no subscriber (blocker B2)"; fi

echo "== service =="; step
if timeout 15 ros2 service list --no-daemon 2>/dev/null | grep -qx "$SRV"; then res GO "$SRV present"
else res NO-GO "$SRV absent (blocker B1)"; fi

echo "== robot_mode =="; step
mode=$(timeout 10 ros2 topic echo --once --no-daemon $MODE 2>&1 | head -20)
if [ -n "$mode" ] && ! echo "$mode" | grep -qi "does not exist\|WARNING"; then echo "$mode"; res GO "$MODE received (inspect values manually)"
else res WARN "no data on $MODE"; fi

echo "== SUMMARY =="
if [ $FAIL -eq 0 ]; then echo "GO"; exit 0; else echo "NO-GO"; exit 1; fi
