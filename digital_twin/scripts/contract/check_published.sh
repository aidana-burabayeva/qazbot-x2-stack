#!/usr/bin/env bash
# Compare the navigation stack in the repo tree with the PUBLISHED robot baseline (origin/robot/stable). Git only: no ssh, no robot contact.
#   scripts/contract/check_published.sh [REF=origin/robot/stable]
# exit 0 = every robot file is present and identical in the tree (files only in the tree, e.g. sim maps, are listed as info), 1 = differs, 2 = ref unavailable.
# The physical X2 is compared by the operator separately (check_robot.sh, manual procedure; never called from nightly/lab).
set -uo pipefail
cd "$(dirname "$0")/../.."
REF=${1:-${ROBOT_REF:-origin/robot/stable}}
BACKEND=$([ -d ../control_backend/ws ] && echo ../control_backend || echo ..)
git -C "$BACKEND" rev-parse --verify -q "$REF^{commit}" >/dev/null || { echo "ref $REF unavailable (git fetch origin robot/stable)"; exit 2; }
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
mkdir "$tmp/ref"
git -C "$BACKEND" archive "$REF" ws/src | tar -x -C "$tmp/ref" || { echo "git archive $REF failed"; exit 2; }
STACK_LIST=$(grep -v '^#' scripts/contract/stack_list.sh)
(cd "$tmp/ref/ws/src" && bash -c "$STACK_LIST") | awk '{print $2" "$1}' | sort >"$tmp/robot"
(cd "$BACKEND/ws/src" && bash -c "$STACK_LIST") | awk '{print $2" "$1}' | sort >"$tmp/tree"
echo "ref=$REF $(git -C "$BACKEND" rev-parse --short "$REF")  robot_files=$(wc -l <"$tmp/robot")  tree_files=$(wc -l <"$tmp/tree")"
drift=$(comm -23 "$tmp/robot" "$tmp/tree" | awk '{print $1}' | sort -u)
extra=$(comm -13 "$tmp/robot" "$tmp/tree" | awk '{print $1}' | sort -u | comm -13 <(echo "$drift") -)
[ -n "$extra" ] && echo "info: only in tree (not in $REF): $(echo "$extra" | wc -l) file(s), e.g. $(echo "$extra" | head -3 | tr '\n' ' ')"
[ -z "$drift" ] && { echo "IN SYNC with $REF"; exit 0; }
echo "DRIFT (tree differs from $REF):"; echo "$drift" | sed 's/^/  /'
exit 1
