#!/usr/bin/env bash
# Read-only status of this workspace and of every running ROS process. Changes nothing, signals nothing.
#   ws/tools/status.sh            exit 0: no ROS process runs from outside <repo>/ws/install; 1: at least one FOREIGN
# Prints: HEAD, x2-release-* tag, dirty, the sha of ws/install/qazbot_build_manifest.yaml, the production-gate
# verdict for this install (bringup_info.py --release-check), then one line per running ROS process (cmdline with
# --ros-args, ros2 launch/run, or an executable/script under an install or /opt/ros):
#   OK        executable/script and every AMENT_PREFIX_PATH entry (readable for own processes) are under
#             <repo>/ws/install, /opt/ros, /usr or /agibot (vendor aimdk_msgs prefix of the adapter)
#   FOREIGN   anything else, with a category: candidates, worktree, tmp, legacy, repo-not-install, other-install
#   vendor/system  executable under /agibot, or only system paths (/opt/ros, /usr) and nothing of a workspace
#             (typically AgiBot processes of user run): only counted
set -uo pipefail
REPO="$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
INST="$REPO/ws/install"; M="$INST/qazbot_build_manifest.yaml"
HEAD="$(git -C "$REPO" rev-parse HEAD)"
TAGS="$(git -C "$REPO" tag --points-at HEAD --list 'x2-release-*' | tr '\n' ' ')"
DIRTY=false; [ -z "$(git -C "$REPO" status --porcelain)" ] || DIRTY=true
mval() { sed -n "s/^$1: //p" "$M" 2>/dev/null | head -1; }
MSHA="$(mval sha)"
echo "workspace  $REPO"
echo "HEAD       $HEAD  tag: ${TAGS:-none}  dirty: $DIRTY"
if [ -f "$M" ]; then
  echo "manifest   sha $MSHA dirty: $(mval dirty) built: $(mval date_utc)  $([ "$MSHA" = "$HEAD" ] && echo '(== HEAD)' || echo '(!= HEAD: rebuild with ws/tools/build.sh)')"
else
  echo "manifest   none ($M missing: ws/tools/build.sh)"
fi
if [ -f "$INST/qazbot_bringup/lib/qazbot_bringup/bringup_info.py" ]; then
  echo "gate       $(python3 "$INST/qazbot_bringup/lib/qazbot_bringup/bringup_info.py" --release-check)"
fi

category() {
  case "$1" in
    "$REPO"/*) echo repo-not-install;;
    */candidates*/*) echo candidates;;
    */worktrees/*) echo worktree;;
    /tmp/*|/var/tmp/*) echo tmp;;
    *_legacy*/*|*/runtime/*|*/ws_cpp_build/*) echo legacy;;
    *) if d="$(echo "$1" | grep -o '^.*/control_backend[^/]*/')" && [ ! -e "$d.git" ]; then echo legacy; else echo other-install; fi;;
  esac
}
ok_path() { case "$1" in "$INST"|"$INST"/*|/opt/ros/*|/usr/*|/bin/*|/agibot/*) return 0;; esac; return 1; }
n_ok=0; n_bad=0; n_vendor=0
for p in /proc/[0-9]*; do
  pid="${p#/proc/}"; [ "$pid" = "$$" ] && continue
  cmd="$( { tr '\0' ' ' < "$p/cmdline"; } 2>/dev/null || true)"; [ -n "$cmd" ] || continue
  case "$cmd" in *--ros-args*|*"ros2 launch"*|*"ros2 run"*|*/bin/ros2\ *|*/install/*|*/opt/ros/*) ;; *) continue;; esac
  case "$cmd" in *ws/tools/status.sh*) continue;; esac
  read -r -a argv <<< "$cmd"
  exe="$(readlink "$p/exe" 2>/dev/null || true)"
  case "$exe ${argv[0]}" in /agibot/*|*" /agibot/"*) n_vendor=$((n_vendor + 1)); continue;; esac
  # judged paths: executable, argv[0], the script of an interpreter (first non-option argument), AMENT_PREFIX_PATH
  paths=("$exe" "${argv[0]}")
  case "${argv[0]##*/}" in python*|bash|sh) for a in "${argv[@]:1}"; do case "$a" in -*) ;; *) paths+=("$a"); break;; esac; done;; esac
  ament="$( { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | sed -n 's/^AMENT_PREFIX_PATH=//p')"
  IFS=: read -r -a ap <<< "$ament"; paths+=("${ap[@]}")
  bad=""; ours=0
  for s in "${paths[@]}"; do
    case "$s" in "$INST"|"$INST"/*) ours=1;; esac
    case "$s" in /*) ok_path "$s" || { bad="$s"; break; };; esac
  done
  if [ -z "$bad" ] && [ "$ours" = 0 ]; then
    n_vendor=$((n_vendor + 1))
  elif [ -z "$bad" ]; then
    printf '%-8s OK       %s\n' "$pid" "${cmd:0:120}"; n_ok=$((n_ok + 1))
  else
    printf '%-8s FOREIGN  %-16s %s\n         %s\n' "$pid" "$(category "$bad")" "$bad" "${cmd:0:120}"; n_bad=$((n_bad + 1))
  fi
done
echo "ROS processes: $n_ok from $INST, $n_bad FOREIGN, $n_vendor vendor/system (not listed)"
[ "$n_bad" -eq 0 ]
