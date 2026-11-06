#!/usr/bin/env bash
# Read-only: which control_backend release is selected and which one is actually running.
#   docker/deploy/status.sh [--prefix DIR] [--legacy-root DIR]
# 1) current/previous symlinks and the sha in their manifest.yaml
# 2) running processes whose executable, command line or cwd lies under <prefix>/releases/<sha>/ (-> that sha)
#    or under the legacy pre-consolidation directory (-> "legacy", i.e. fb18dc1 + live-only files, see P1S).
set -euo pipefail
PREFIX="${QAZBOT_PROD_PREFIX:-$HOME/prod}"; LEGACY="${QAZBOT_LEGACY_ROOT:-$HOME/control_backend}"
while [ $# -gt 0 ]; do case "$1" in
  --prefix) PREFIX="$2"; shift 2;; --legacy-root) LEGACY="$2"; shift 2;; *) echo "unknown arg $1" >&2; exit 2;; esac; done
msha() { sed -n 's/^sha: //p' "$1/manifest.yaml" 2>/dev/null | head -1; }
for l in current previous; do
  t="$(readlink "$PREFIX/$l" 2>/dev/null || true)"
  if [ -n "$t" ]; then echo "$l -> $t (manifest sha: $(msha "$PREFIX/$t" || true))"; else echo "$l -> (none)"; fi
done
RREAL="$(realpath -m "$PREFIX/releases")"; LREAL="$(realpath -m "$LEGACY")"
declare -A seen=()
for p in /proc/[0-9]*; do
  pid="${p#/proc/}"
  cmd="$( { tr '\0' ' ' < "$p/cmdline"; } 2>/dev/null || true)"; [ -n "$cmd" ] || continue   # process may vanish
  read -r -a cmd_a <<< "$cmd"   # array split: no pathname expansion of argv words
  exe="$(readlink "$p/exe" 2>/dev/null || true)"; cwd="$(readlink "$p/cwd" 2>/dev/null || true)"
  where=""
  for s in "$exe" "$cwd" "${cmd_a[@]}"; do
    case "$s" in
      "$RREAL"/*) r="${s#"$RREAL"/}"; where="release ${r%%/*}"; break;;
      "$PREFIX/current"/*) where="release $(basename "$(realpath "$PREFIX/current")")"; break;;
      "$LREAL"/*) where="legacy ($LEGACY)"; break;;
    esac
  done
  [ -n "$where" ] || continue
  case "$cmd" in *docker/deploy/status.sh*) continue;; esac
  printf '%-7s %-52s %s\n' "$pid" "$where" "$(echo "$cmd" | cut -c1-110)"
  seen["$where"]=1
done
if [ ${#seen[@]} -eq 0 ]; then echo "running: none"; else echo "running: ${!seen[*]}"; fi
