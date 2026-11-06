#!/usr/bin/env bash
# Regression test of task_cleanup.sh git safety (runs entirely in its own mktemp dir, removes it at the end).
#   ws/tools/test_task_cleanup.sh        # exit 0 = all cases pass
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
T="$(mktemp -d)"; trap 'chmod -R u+w "$T" 2>/dev/null; rm -rf "$T"' EXIT
export QAZBOT_TASK_DIR="$T/task" QAZBOT_CLEANUP_ROOTS="$T/allowed" QAZBOT_PROTECTED="$T/none"
export GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@t GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@t
mkdir -p "$T/allowed" "$T/elsewhere"
git init -q --bare "$T/origin.git"
mkrepo() {  # $1 name -> clone with one pushed commit
  git clone -q "$T/origin.git" "$T/allowed/$1" 2>/dev/null
  git -C "$T/allowed/$1" checkout -q -B main
  echo a > "$T/allowed/$1/f"; git -C "$T/allowed/$1" add f; git -C "$T/allowed/$1" commit -qm init
  git -C "$T/allowed/$1" push -q origin main 2>/dev/null
}
fail=0
check() {  # $1 label, $2 expected KEPT|removed, $3 task id, $4 path
  "$HERE/task_register.sh" "$3" "$4" >/dev/null
  local out; out="$("$HERE/task_cleanup.sh" "$3" 2>&1)"
  if [ "$2" = KEPT ]; then { [[ "$out" == *KEPT* ]] && [ -e "$4" ]; } || { echo "FAIL $1: expected KEPT, got: $out"; fail=1; return; }
  else { [[ "$out" == *removed* ]] && [ ! -e "$4" ]; } || { echo "FAIL $1: expected removed, got: $out"; fail=1; return; }; fi
  echo "ok   $1"
}
mkrepo plain;  check "clean pushed repo is removed" removed t1 "$T/allowed/plain"
mkrepo dirtywt; git -C "$T/allowed/dirtywt" worktree add -q "$T/elsewhere/wt_dirty" -b wtb >/dev/null 2>&1
git -C "$T/elsewhere/wt_dirty" push -q origin wtb 2>/dev/null; echo change >> "$T/elsewhere/wt_dirty/f"
check "repo with linked worktree (uncommitted change) elsewhere is KEPT" KEPT t2 "$T/allowed/dirtywt"
[ -n "$(git -C "$T/elsewhere/wt_dirty" status --porcelain)" ] || { echo "FAIL: worktree change lost"; fail=1; }
mkrepo cleanwt; git -C "$T/allowed/cleanwt" worktree add -q "$T/elsewhere/wt_clean" -b wtc >/dev/null 2>&1
git -C "$T/elsewhere/wt_clean" push -q origin wtc 2>/dev/null
check "repo with clean linked worktree elsewhere is KEPT (would orphan it)" KEPT t3 "$T/allowed/cleanwt"
mkrepo box_repo; mkdir "$T/allowed/box"; mv "$T/allowed/box_repo" "$T/allowed/box/repo"
git -C "$T/allowed/box/repo" worktree add -q "$T/allowed/box/wt" -b wti >/dev/null 2>&1; git -C "$T/allowed/box/wt" push -q origin wti 2>/dev/null
check "dir holding a repo and its clean linked worktree (both inside) is removed" removed t4 "$T/allowed/box"
mkrepo innerdirty; git -C "$T/allowed/innerdirty" worktree add -q "$T/allowed/innerdirty/wt_in" -b wtd >/dev/null 2>&1
git -C "$T/allowed/innerdirty/wt_in" push -q origin wtd 2>/dev/null; echo x > "$T/allowed/innerdirty/wt_in/new"
check "repo with dirty linked worktree inside is KEPT" KEPT t5 "$T/allowed/innerdirty"
exit $fail
