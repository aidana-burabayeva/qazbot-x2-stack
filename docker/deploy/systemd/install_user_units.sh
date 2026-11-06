#!/bin/bash
# Idempotent install of host-level QazBot user units (no sudo). Usage: install_user_units.sh [--dry-run] [--uninstall]
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="${XDG_CONFIG_HOME:-$HOME/.config}"
BIN_DST="$CFG/qazbot/qazbot_vendor_trim.sh"
UNIT_DST="$CFG/systemd/user/qazbot-vendor-trim.service"
DRY=0; UNINSTALL=0
for a in "$@"; do case "$a" in
  --dry-run) DRY=1;; --uninstall) UNINSTALL=1;;
  *) echo "usage: $0 [--dry-run] [--uninstall]" >&2; exit 2;; esac; done
run() { if [ "$DRY" = 1 ]; then echo "[dry-run] $*"; else "$@"; fi; }
if [ "$UNINSTALL" = 1 ]; then
  run systemctl --user disable qazbot-vendor-trim.service
  echo "installed files left in place: $BIN_DST $UNIT_DST"; exit 0
fi
sync_file() { # src dst mode
  if [ -f "$2" ] && cmp -s "$1" "$2"; then echo "unchanged: $2"; return; fi
  if [ -f "$2" ]; then echo "--- diff $2 vs $1"; diff -u "$2" "$1" || true; else echo "new: $2"; fi
  run install -D -m "$3" "$1" "$2"
}
sync_file "$SRC/qazbot_vendor_trim.sh" "$BIN_DST" 755
sync_file "$SRC/qazbot-vendor-trim.service" "$UNIT_DST" 644
run systemctl --user daemon-reload
run systemctl --user enable qazbot-vendor-trim.service
