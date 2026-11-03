#!/usr/bin/env bash
# Keeps the vendor assistant (Lingxi, app `agent` on the interaction unit) silent: language en, run mode
# only_voice (assistant off, denoised audio still provided), no greeting. These reset whenever the agent
# restarts, so they are sent again for every new agent process. The call is the vendor's own workaround:
# https://x2-aimdk.agibot.com/en/latest/faq/temp_works.html
INTERVAL="${LINGXI_CHECK_SEC:-15}"
HERE="$(cd "$(dirname "$0")" && pwd)"
set +u
. "$HERE/env.sh" >/dev/null 2>&1 || { echo "lingxi_off: no robot environment" >&2; exit 1; }
FACE=(ssh -n -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new
      -i "$HOME/.ssh/id_ed25519" -o IdentitiesOnly=yes agi@10.0.1.42)

quieten() {
  timeout 30 ros2 service call /aimdk_5Fmsgs/srv/SetAgentPropertiesRequest \
    aimdk_msgs/srv/SetAgentPropertiesRequest \
    "contents: {properties: [{key: {value: 1}, value: en}, {key: {value: 2}, value: only_voice}, {key: {value: 3}, value: \"false\"}]}" \
    >/dev/null 2>&1
}

done_pid=""
while true; do
  pid="$("${FACE[@]}" 'pgrep -f "[a]ima-agent-app" | head -1' 2>/dev/null)"
  if [ -n "$pid" ] && [ "$pid" != "$done_pid" ]; then
    sleep "$INTERVAL"                                  # a fresh agent needs a moment to bring its service up
    if quieten; then
      echo "Lingxi (agent pid $pid) set silent: only_voice, English, no greeting."
      done_pid="$pid"
    else
      echo "Lingxi (agent pid $pid): could not set it silent yet, will retry." >&2
    fi
  fi
  sleep "$INTERVAL"
done
