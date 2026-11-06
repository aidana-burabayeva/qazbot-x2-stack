#!/usr/bin/env bash
# Swap <prefix>/current and <prefix>/previous (the release that was current before the last deploy).
#   docker/deploy/rollback.sh [--prefix DIR]
# Only switches symlinks; restart the stack from <prefix>/current afterwards (operator). Running rollback twice
# returns to the original state. The pre-consolidation production (legacy control_backend directory, fb18dc1) is
# not a release: rolling back to it means restarting the legacy commands (docs/RELEASE.md, "legacy rollback").
set -euo pipefail
PREFIX="${QAZBOT_PROD_PREFIX:-}"
while [ $# -gt 0 ]; do case "$1" in --prefix) PREFIX="$2"; shift 2;; *) echo "unknown arg $1" >&2; exit 2;; esac; done
[ -n "$PREFIX" ] || { echo "REFUSED: no release prefix; pass --prefix DIR or export QAZBOT_PROD_PREFIX explicitly (no default: the production tree is never implied)" >&2; exit 2; }
CUR="$(readlink "$PREFIX/current" 2>/dev/null || true)"
PREV="$(readlink "$PREFIX/previous" 2>/dev/null || true)"
[ -n "$PREV" ] || { echo "REFUSED: no $PREFIX/previous" >&2; exit 3; }
[ -f "$PREFIX/$PREV/manifest.yaml" ] || { echo "REFUSED: previous $PREV has no manifest" >&2; exit 3; }
ln -sfn "$PREV" "$PREFIX/.current.tmp" && mv -T "$PREFIX/.current.tmp" "$PREFIX/current"
if [ -n "$CUR" ]; then ln -sfn "$CUR" "$PREFIX/.previous.tmp" && mv -T "$PREFIX/.previous.tmp" "$PREFIX/previous"; fi
echo "current -> $PREV ; previous -> ${CUR:-none}"
echo "next: restart the stack from $PREFIX/current (operator), then docker/deploy/status.sh"
