#!/usr/bin/env bash
# P27 ROLLBACK (2026-10-30): restore every *.bak_20261030_pre_p27 file (Trust T5 re-anchor), then restart Trust ONLY
# (restart_trust.sh, restored = P26 state). GC and adapter were NOT changed by P27 (they reuse the P26 quality_pause path).
# Robot STANDING, GC not busy. Afterwards: operator initialpose (Trust restart clears READY).
# Usage: ./rollback_p27.sh --dry-run          print all actions, change nothing
#        CONFIRM=1 ./rollback_p27.sh          restore files + restart Trust
#        CONFIRM=1 ./rollback_p27.sh --files-only   restore files only (no restart)
# Full P26 rollback afterwards: rollback_p26.sh (restores the older *.bak_20261030_pre_p26 files).
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
MIR=/home/agi/control_backend/qazbot-x2-stack/ws/src/navigation
S=bak_20261030_pre_p27
MODE=${1:-}
DRY=0; [ "$MODE" = --dry-run ] && DRY=1
[ "$DRY" = 1 ] || [ "${CONFIRM:-}" = 1 ] || { echo "refusing: CONFIRM=1 or --dry-run required"; exit 1; }
log(){ echo "[$(date +%T.%N)] rollback_p27: $*" | tee -a $EV/start_run.txt; }
run(){ if [ "$DRY" = 1 ]; then echo "DRY: $*"; else eval "$@" || { log "FAILED: $*"; exit 9; }; fi; }
FILES="$NAV/loc_trust.py $NAV/localization_trust_node.py $MIR/loc_trust.py $MIR/localization_trust_node.py
$EV/restart_trust.sh $EV/start.sh $EV/start_loc.sh"
for f in $FILES; do [ -f $f.$S ] || { log "ABORT: missing backup $f.$S"; exit 2; }; done
log "restore $(echo $FILES | wc -w) files from *.$S (dry=$DRY)"
for f in $FILES; do run "cp -p $f.$S $f"; done
[ "$DRY" = 1 ] || for f in $NAV/loc_trust.py $NAV/localization_trust_node.py; do python3 -m py_compile $f || { log "ABORT: $f restored but does not compile"; exit 3; }; done
[ "$MODE" = --files-only ] && { log "files restored; no restart (--files-only)"; exit 0; }
if [ "$DRY" = 1 ]; then echo "DRY: bash $EV/restart_trust.sh   # Trust restart with restored (P26) code + params"; exit 0; fi
bash $EV/restart_trust.sh || { log "restart_trust failed rc=$?"; exit 4; }
log "rollback complete -> operator initialpose needed"
