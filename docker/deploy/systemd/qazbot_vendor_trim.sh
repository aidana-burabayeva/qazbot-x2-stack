#!/bin/bash
# Stops vendor apps not needed by navigation after every boot (EM default_apps are root-owned firmware config).
# AgiBot docs: ~/aimdk/extra/mc-rl/README_en.md (Step 0.5 / Teardown) use `aima em stop-app/start-app`; docs otherwise silent on trimming default_apps.
# Same method as the manual trim (P8a): `aima em stop-app`. Repeated passes cover EM's staggered start.
AIMA="$(command -v aima || echo /home/agi/.local/bin/aima)"
APPS="hal_camera orbbec_camera perception drp lx_webrtc_app"
LOG="${XDG_STATE_HOME:-$HOME/.local/state}/qazbot/vendor_trim.log"
mkdir -p "$(dirname "$LOG")"
log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

log "start"
for i in $(seq 1 60); do "$AIMA" em list-apps >/dev/null 2>&1 && break; sleep 5; done
log "em reachable (wait $((i*5)) s max)"
for pass in 1 2 3 4 5 6; do
  for a in $APPS; do
    "$AIMA" em stop-app "$a" 2>&1 | grep -q "success" && log "pass $pass: stopped $a"
  done
  sleep 20
done
left="$(pgrep -af 'hal_camera_orin|camera_container|aima-drp|lx_webrtc|perception' | grep -v pgrep | cut -c1-80)"
log "done; still running: ${left:-none}"
