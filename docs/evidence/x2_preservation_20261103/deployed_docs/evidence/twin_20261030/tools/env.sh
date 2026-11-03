source /opt/ros/humble/setup.bash
set +u; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_20261030_1623/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
