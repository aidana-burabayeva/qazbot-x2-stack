#!/usr/bin/env bash
# P26 ROLLBACK (2026-10-30): restore every *.bak_20261030_pre_p26 file, then restart what runs the restored code:
#   1. localization + Trust + seed_loop     (restart_loc.sh: AMCL params, Trust T5/quality gate, seed_loop 12 nomotion)
#   2. nav_full incl. goal_coordinator     (restart_gc.sh, xy_tol 0.35 as now; adapter untouched)
#   3. motion adapter + ONE MC DELETE/ADD  (p20_restart_adapter.sh, source sdk_qazbot_nav2_1030a)
# Robot STANDING, GC not busy. Afterwards: operator initialpose. Each restart script re-checks its own preconditions.
# Usage: ./rollback_p26.sh --dry-run          print all actions, change nothing
#        CONFIRM=1 ./rollback_p26.sh          restore files + restart 1-3
#        CONFIRM=1 ./rollback_p26.sh --files-only   restore files only (no restarts)
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
MIR=/home/agi/control_backend/qazbot-x2-stack/ws/src/navigation
S=bak_20261030_pre_p26
MODE=${1:-}
DRY=0; [ "$MODE" = --dry-run ] && DRY=1
[ "$DRY" = 1 ] || [ "${CONFIRM:-}" = 1 ] || { echo "refusing: CONFIRM=1 or --dry-run required"; exit 1; }
log(){ echo "[$(date +%T.%N)] rollback_p26: $*" | tee -a $EV/start_run.txt; }
run(){ if [ "$DRY" = 1 ]; then echo "DRY: $*"; else eval "$@" || { log "FAILED: $*"; exit 9; }; fi; }
FILES="$NAV/loc_trust.py $NAV/localization_trust_node.py $NAV/goal_coordinator.py $NAV/x2_motion_adapter.py $NAV/localization.launch.py
$EV/seed_loop.py $EV/start.sh $EV/start_loc.sh $EV/restart_trust.sh
$MIR/loc_trust.py $MIR/localization_trust_node.py $MIR/goal_coordinator.py $MIR/localization.launch.py"
for f in $FILES; do [ -f $f.$S ] || { log "ABORT: missing backup $f.$S"; exit 2; }; done
log "restore $(echo $FILES | wc -w) files from *.$S (dry=$DRY)"
for f in $FILES; do run "cp -p $f.$S $f"; done
[ "$DRY" = 1 ] || for f in $NAV/loc_trust.py $NAV/localization_trust_node.py $NAV/goal_coordinator.py $NAV/x2_motion_adapter.py $NAV/localization.launch.py $EV/seed_loop.py; do python3 -m py_compile $f || { log "ABORT: $f restored but does not compile"; exit 3; }; done
[ "$MODE" = --files-only ] && { log "files restored; no restarts (--files-only)"; exit 0; }
if [ "$DRY" = 1 ]; then
  echo "DRY: CONFIRM=1 bash $EV/restart_loc.sh   # stop loc launch+Trust+seed_loop, start_loc.sh (restored), seed_loop"
  echo "DRY: CONFIRM=1 bash $EV/restart_gc.sh    # nav_full restart with restored goal_coordinator.py"
  echo "DRY: CONFIRM=1 bash $EV/p20_restart_adapter.sh   # adapter restart with restored x2_motion_adapter.py (MC DELETE+ADD)"
  echo "--- restart_loc.sh dry run:"; DRY=1 bash $EV/restart_loc.sh
  exit 0
fi
CONFIRM=1 bash $EV/restart_loc.sh || { log "restart_loc failed rc=$?"; exit 4; }
CONFIRM=1 bash $EV/restart_gc.sh || { log "restart_gc failed rc=$?"; exit 5; }
CONFIRM=1 bash $EV/p20_restart_adapter.sh || { log "adapter restart failed rc=$?"; exit 6; }
log "rollback complete -> operator initialpose needed"
