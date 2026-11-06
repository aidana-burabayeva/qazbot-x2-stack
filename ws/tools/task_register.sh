#!/usr/bin/env bash
# Register paths a task created, so task_cleanup.sh may delete them later (and nothing else).
#   ws/tools/task_register.sh <task_id> <path> [<path> ...]
# Manifest: <repo>/.task/<task_id>.manifest (gitignored), one absolute path per line, with a header comment.
set -euo pipefail
ID="${1:?usage: task_register.sh <task_id> <path>...}"; shift
[[ "$ID" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "bad task id: $ID" >&2; exit 2; }
[ $# -gt 0 ] || { echo "no paths given" >&2; exit 2; }
REPO="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
DIR="${QAZBOT_TASK_DIR:-$REPO/.task}"; M="$DIR/$ID.manifest"
mkdir -p "$DIR"
[ -f "$M" ] || echo "# task $ID: paths created by this task (registered $(date -u +%FT%TZ) by $(id -un)@$(hostname))" > "$M"
for p in "$@"; do
  a="$(realpath -m -- "$p")"
  case "$a" in /|/home|/home/*/|"$HOME"|/tmp|/usr*|/opt*|/etc*|/agibot*) echo "refusing to register $a" >&2; exit 3;; esac
  grep -qxF -- "$a" "$M" || echo "$a" >> "$M"
  echo "registered $a"
done
