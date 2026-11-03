# Source this to get the robot environment: ROS 2, aimdk_msgs, and the firmware's DDS whitelist profile.
set +e
[ -f "$HOME/.aima/env/bashrc" ] && . "$HOME/.aima/env/bashrc"
. /opt/ros/humble/setup.bash
# The firmware profile whitelists the internal interface only. Without it Fast DDS also binds wifi0 and a
# gesture meant for this robot can reach every X2 on the venue Wi-Fi.
if [ -z "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  echo "no FASTRTPS_DEFAULT_PROFILES_FILE from ~/.aima/env/bashrc - refusing to join every robot's DDS graph" >&2
  return 1 2>/dev/null || exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
C=/agibot/software/common
if [ -r "$C/local/lib/python3.10/dist-packages/aimdk_msgs/__init__.py" ]; then
  export PYTHONPATH="$C/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$C/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export AMENT_PREFIX_PATH="$C${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
else
  # Not readable: use this service's own copy of the message package (robot/make_overlay.sh creates it).
  O="$HOME/qazbot-speech/overlay/aimdk_msgs"
  if [ -f "$O/local/lib/python3.10/dist-packages/aimdk_msgs/__init__.py" ]; then
    export PYTHONPATH="$O/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$O/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export AMENT_PREFIX_PATH="$O${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  fi
fi
