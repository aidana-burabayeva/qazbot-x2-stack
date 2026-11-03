#!/usr/bin/env bash
# Gives this service its OWN copy of the robot's message package (aimdk_msgs) in ~/qazbot-speech/overlay, so it
# does not depend on any other workspace staying in place. Run on the robot; deploy.sh runs it. Idempotent.
set -e
DEST="$HOME/qazbot-speech/overlay/aimdk_msgs"
if [ -f "$DEST/local/lib/python3.10/dist-packages/aimdk_msgs/__init__.py" ]; then
  echo "overlay present: $DEST"; exit 0
fi
# The firmware's own copy is used directly when it is readable (see env.sh); this is for robots where it is not.
for src in "$HOME"/aimdk/install/aimdk_msgs "$HOME"/*/install/aimdk_msgs; do
  [ -f "$src/local/lib/python3.10/dist-packages/aimdk_msgs/__init__.py" ] || continue
  mkdir -p "$(dirname "$DEST")"
  cp -a "$src" "$DEST"
  echo "overlay created from $src"; exit 0
done
echo "no aimdk_msgs package found to copy (needed only where /agibot/software/common is not readable)" >&2
exit 0
