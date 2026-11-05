#!/usr/bin/env bash
# Run ON the robot (X2 .66). Read-only. Confirms (1) the installed aimdk_msgs resolves with the overlay,
# (2) the definitions used by the motion adapter are identical to the SDK sources in ~/aimdk,
# (3) the live MC services and topics carry those types. Exit 1 on any difference.
# (no set -u: ROS setup.bash reads unset variables)
source /opt/ros/humble/setup.bash
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${AIMDK_ENV:-$HERE/../../ws/src/navigation/aimdk_env.sh}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml
rc=0
echo "prefix: $(ros2 pkg prefix aimdk_msgs 2>&1)"
python3 -c "import aimdk_msgs.msg as m, aimdk_msgs.srv as s; print('python import ok')" || rc=1
C=/agibot/software/common/share/aimdk_msgs; S="$HOME/aimdk/src/aimdk_msgs"
for f in McLocomotionVelocity.msg McInputSource.msg McCommonState.msg McLocomotionSpeedStatus.msg JointStateArray.msg \
         JointState.msg SetMcInputSource.srv GetCurrentInputSource.srv GetMcLocomotionSpeedStatus.srv; do
  a=$(find "$C" -name "$f" | head -1); b=$(find "$S" -name "$f" -not -path '*prebuilt*' | head -1)
  if [ -n "$a" ] && [ -n "$b" ] && diff -q "$a" "$b" >/dev/null; then echo "SAME     $f"; else echo "DIFFERENT_OR_MISSING $f"; rc=1; fi
done
ros2 service list -t 2>/dev/null | grep -E 'SetMcInputSource|GetCurrentInputSource|GetMcLocomotionSpeedStatus' || rc=1
ros2 topic info /aima/mc/locomotion/velocity 2>/dev/null | head -1
ros2 topic info /aima/mc/common/state 2>/dev/null | head -1
echo "versions: $(grep AGIBOT_SOFTWARE_VERSION ~/.aima/env/bashrc) | interface $(grep -o '"interface": *"[^"]*"' /agibot/software/common/version.json) | aimdk tree $(head -1 ~/aimdk/version)"
exit $rc
