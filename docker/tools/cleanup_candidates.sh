#!/usr/bin/env bash
# READ-ONLY report of debris that may be cleaned up. Deletes nothing.
#   docker/tools/cleanup_candidates.sh [root ...]      (default: this repository and $HOME/lab)
# Lists, with sizes: untracked/ignored files in git checkouts, colcon build/install/log dirs, __pycache__,
# *.bak*, core dumps, bags (*.bag, *.db3, *.mcap), prunable git worktrees, releases that are neither current nor
# previous (QAZBOT_PROD_PREFIX, default $HOME/prod) and registered-but-not-cleaned task manifests.
# Anything you decide to delete must be registered to a task and removed with task_cleanup.sh, or approved by the
# operator.
set -uo pipefail
REPO="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
[ $# -gt 0 ] && ROOTS=("$@") || ROOTS=("$REPO" "$HOME/lab")
PROD="${QAZBOT_PROD_PREFIX:-$HOME/prod}"
sz() { du -sh -- "$1" 2>/dev/null | cut -f1; }
for root in "${ROOTS[@]}"; do
  [ -d "$root" ] || continue
  echo "== $root"
  while IFS= read -r g; do
    w="$(dirname "$g")"
    echo "-- git checkout $w ($(git -C "$w" rev-parse --abbrev-ref HEAD 2>/dev/null))"
    git -C "$w" status --porcelain --ignored 2>/dev/null | sed 's/^/   /' | head -50
    git -C "$w" worktree list --porcelain 2>/dev/null | awk '/^worktree/{w=$2} /^prunable/{print "   prunable worktree " w}'
    n="$(git -C "$w" log --oneline --branches --not --remotes 2>/dev/null | wc -l)"; [ "$n" -gt 0 ] && echo "   $n local commit(s) not on any remote"
  done < <(find "$root" -maxdepth 4 -name .git \( -type d -o -type f \) 2>/dev/null)
  find "$root" -xdev \( -name .git -prune \) -o \( -type d \( -name build -o -name install -o -name log -o -name __pycache__ -o -name build_\* \) -print -prune \) 2>/dev/null \
    | while IFS= read -r d; do echo "   dir  $(sz "$d")  $d"; done
  find "$root" -xdev \( -name .git -prune \) -o -type f \( -name '*.bak*' -o -name 'core' -o -name 'core.[0-9]*' -o -name '*.bag' -o -name '*.db3' -o -name '*.mcap' \) -print 2>/dev/null \
    | head -200 | while IFS= read -r f; do echo "   file $(sz "$f")  $f"; done
done
if [ -d "$PROD/releases" ]; then
  echo "== releases in $PROD"
  cur="$(realpath -m "$PROD/current")"; prev="$(realpath -m "$PROD/previous")"
  for r in "$PROD"/releases/*; do
    [ -d "$r" ] || continue; rr="$(realpath "$r")"
    if [ "$rr" = "$cur" ]; then s=current; elif [ "$rr" = "$prev" ]; then s=previous; else s="CANDIDATE (not current/previous)"; fi
    echo "   $(sz "$r")  $r  $s"
  done
fi
if ls "$REPO"/.task/*.manifest >/dev/null 2>&1; then
  echo "== registered task manifests (clean with task_cleanup.sh <id>)"
  for m in "$REPO"/.task/*.manifest; do echo "   $(basename "$m" .manifest): $(grep -vc '^#' "$m") path(s)"; done
fi
