#!/bin/bash
# Start/stop the A/B raw LaserScan bridge (publishes only /qazbot_raw/*). Usage: raw_scan_bridge_ctl.sh start|stop|status
E=/home/agi/control_backend/docs/evidence/raw_scan_ab_20261029
PIDF=$E/raw_scan_bridge.pid
case "$1" in
start)
  [ -f $PIDF ] && kill -0 $(cat $PIDF) 2>/dev/null && { echo "already running pid $(cat $PIDF)"; exit 0; }
  source /opt/ros/humble/setup.bash
  export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
    FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_20261029_1425/dds_nav_two_iface_log.xml
  setsid nohup python3 /home/agi/control_backend/ws/src/lio/raw_scan_bridge.py --ros-args \
    -p scan_every:=2 -p range_max:=8.0 -p low_slice:="${LOW_SLICE:--0.80,-0.50,0.015}" \
    >> $E/raw_scan_bridge.log 2>&1 < /dev/null &
  echo $! > $PIDF; echo "started pid $!";;
stop)
  [ -f $PIDF ] && kill -INT $(cat $PIDF) 2>/dev/null; sleep 2; [ -f $PIDF ] && kill $(cat $PIDF) 2>/dev/null; rm -f $PIDF; echo stopped;;
status)
  [ -f $PIDF ] && kill -0 $(cat $PIDF) 2>/dev/null && { p=$(cat $PIDF); echo "running pid $p"; ps -o pid,pcpu,rss,etime,args -p $p; } || echo "not running";;
esac
