#!/usr/bin/env bash
# Compare the navigation stack on the physical X2 with the repo tree. READ-ONLY on the robot (find + sha256sum, nothing is written).
#   scripts/contract/check_robot.sh [ROBOT=agi@192.168.1.72] [ROBOT_SRC=/home/agi/control_backend/ws/src]
# exit 0 = identical, 1 = differs (files listed), 2 = robot unreachable. Needs key-based ssh (BatchMode, never prompts for a password).
set -uo pipefail
cd "$(dirname "$0")/../.."
ROBOT=${ROBOT:-agi@192.168.1.72}
ROBOT_SRC=${ROBOT_SRC:-/home/agi/control_backend/ws/src}
BACKEND=$([ -d ../control_backend/ws ] && echo ../control_backend || echo ..)
LIST=scripts/contract/stack_list.sh
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
(cd "$BACKEND/ws/src" && bash "$OLDPWD/$LIST") >"$tmp/tree"
timeout 40 ssh -o BatchMode=yes -o ConnectTimeout=8 "$ROBOT" "cd '$ROBOT_SRC' && bash -s" <"$LIST" >"$tmp/robot" 2>"$tmp/err" || { echo "robot unreachable: $(head -1 "$tmp/err")"; exit 2; }
[ -s "$tmp/robot" ] || { echo "robot returned an empty listing ($ROBOT_SRC missing?)"; exit 2; }
h() { sha256sum <"$1" | cut -c1-16; }
echo "tree=$(h "$tmp/tree") ($(wc -l <"$tmp/tree") files)  robot=$(h "$tmp/robot") ($(wc -l <"$tmp/robot") files)"
d=$(diff <(awk '{print $2" "$1}' "$tmp/robot") <(awk '{print $2" "$1}' "$tmp/tree") | awk '/^[<>]/{print $2}' | sort -u)
[ -z "$d" ] && { echo "IN SYNC"; exit 0; }
echo "DRIFT (robot differs from repo tree):"; echo "$d" | sed 's/^/  /'
exit 1
