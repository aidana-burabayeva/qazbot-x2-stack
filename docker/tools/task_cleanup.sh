#!/usr/bin/env bash
# Delete ONLY the paths registered for a task (docker/tools/task_register.sh), each after safety checks.
#   docker/tools/task_cleanup.sh <task_id> [--dry-run]
# A path is deleted only if ALL hold, otherwise it is kept and reported:
#   1. it is inside an allowed root: QAZBOT_CLEANUP_ROOTS (colon list), default "/tmp:$HOME/lab:$HOME/qazbot_scratch"
#   2. it is not (inside) a protected path: QAZBOT_PROTECTED (colon list), default
#      "$HOME/control_backend:$HOME/worktrees:$HOME/aimdk:$HOME/prod/current:$HOME/prod/previous:$HOME/qazbot_archive",
#      and not the target of <prod>/current or <prod>/previous (QAZBOT_PROD_PREFIX, default $HOME/prod)
#   3. no process has it (or a file below it) open, as cwd, exe or mapped (lsof; fuser fallback)
#   4. if it is a git checkout/worktree: no uncommitted or untracked changes, no stash, no commit that is not on a remote
# Paths that no longer exist are dropped from the manifest. Exit status 1 when anything was kept.
set -euo pipefail
ID="${1:?usage: task_cleanup.sh <task_id> [--dry-run]}"; DRY=0; [ "${2:-}" = "--dry-run" ] && DRY=1
REPO="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
M="${QAZBOT_TASK_DIR:-$REPO/.task}/$ID.manifest"
[ -f "$M" ] || { echo "no manifest $M" >&2; exit 2; }
IFS=: read -r -a ROOTS <<< "${QAZBOT_CLEANUP_ROOTS:-/tmp:$HOME/lab:$HOME/qazbot_scratch}"
IFS=: read -r -a PROT <<< "${QAZBOT_PROTECTED:-$HOME/control_backend:$HOME/worktrees:$HOME/aimdk:$HOME/prod/current:$HOME/prod/previous:$HOME/qazbot_archive}"
PROD="${QAZBOT_PROD_PREFIX:-$HOME/prod}"
for l in current previous; do t="$(realpath -e "$PROD/$l" 2>/dev/null || true)"; [ -n "$t" ] && PROT+=("$t"); done
under() { [ "$1" = "$2" ] || [[ "$1" == "$2"/* ]]; }
in_use() {  # prints holders; returns 0 when something uses the path
  local p="$1" out=""
  if command -v lsof >/dev/null; then
    if [ -d "$p" ]; then out="$(lsof -w +D "$p" 2>/dev/null | tail -n +2 || true)"; else out="$(lsof -w -- "$p" 2>/dev/null | tail -n +2 || true)"; fi
  else out="$(fuser -m -- "$p" 2>/dev/null || true)"; fi
  # cwd / exe of processes (lsof +D covers them; this is a cheap second check)
  for d in /proc/[0-9]*; do
    for k in cwd exe; do local r; r="$(readlink "$d/$k" 2>/dev/null || true)"; [ -n "$r" ] && under "$r" "$p" && out+=$'\n'"pid ${d#/proc/} $k $r"; done
  done
  [ -n "${out//[$'\n ']/}" ] && { echo "$out" | head -5; return 0; }
  return 1
}
git_unsafe() {  # prints reason; returns 0 when a git checkout under p holds unique work
  local g reason=""
  while IFS= read -r g; do
    local w; w="$(dirname "$g")"
    [ -n "$(git -C "$w" status --porcelain 2>/dev/null)" ] && reason+="$w: uncommitted/untracked changes; "
    [ -n "$(git -C "$w" stash list 2>/dev/null)" ] && reason+="$w: stash entries; "
    [ -n "$(git -C "$w" log --oneline --branches --not --remotes 2>/dev/null | head -1)" ] && reason+="$w: commits not on any remote; "
    [ -n "$(git -C "$w" log --oneline -1 HEAD --not --remotes 2>/dev/null)" ] && reason+="$w: detached HEAD not on a remote; "
  done < <(find "$1" -maxdepth 6 -name .git \( -type d -o -type f \) 2>/dev/null)
  [ -n "$reason" ] && { echo "$reason"; return 0; }
  return 1
}
kept=0; TMP="$(mktemp)"; grep '^#' "$M" > "$TMP" || true
while IFS= read -r p; do
  [[ -z "$p" || "$p" == \#* ]] && continue
  if [ ! -e "$p" ] && [ ! -L "$p" ]; then echo "gone     $p"; continue; fi
  r="$(realpath -e -- "$p")"; why=""
  ok_root=0; for a in "${ROOTS[@]}"; do [ -n "$a" ] && under "$r" "$(realpath -m "$a")" && [ "$r" != "$(realpath -m "$a")" ] && ok_root=1; done
  [ $ok_root -eq 1 ] || why="outside allowed roots"
  for x in "${PROT[@]}"; do [ -n "$x" ] || continue; xr="$(realpath -m "$x")"; { under "$r" "$xr" || under "$xr" "$r"; } && why="protected ($x)"; done
  [ -z "$why" ] && h="$(in_use "$r")" && why="in use: $(echo "$h" | head -1)"
  [ -z "$why" ] && g="$(git_unsafe "$r")" && why="git: $g"
  if [ -n "$why" ]; then echo "KEPT     $p  [$why]"; echo "$p" >> "$TMP"; kept=1; continue; fi
  if [ $DRY -eq 1 ]; then echo "would rm $p"; echo "$p" >> "$TMP"; else chmod -R u+w -- "$r" 2>/dev/null || true; rm -rf -- "$r"; echo "removed  $p"; fi   # releases are a-w
done < "$M"
mv "$TMP" "$M"
exit $kept
