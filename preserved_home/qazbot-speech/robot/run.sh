#!/bin/bash
# Start the speech service on an X2 Ultra develop unit. By hand:  robot/run.sh --robot [--replay clip.wav]
set -e
cd "$(dirname "$0")/.."
. robot/env.sh
python3 -c 'import aimdk_msgs, rclpy' || { echo "no aimdk_msgs/rclpy on this unit" >&2; exit 1; }
export PYTHONPATH="$PWD/src:$PWD/lib${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -u -m qazbot_speech --env "${QAZBOT_ENV:-$PWD/.env}" "$@"
