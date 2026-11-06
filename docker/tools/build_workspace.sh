#!/usr/bin/env bash
# CANONICAL build of the whole control_backend workspace from a clean clone (lab, container and release use this one).
#   docker/tools/build_workspace.sh --prefix DIR [--deps DIR] [--deb-cache DIR] [--git-cache DIR] [--no-fetch]
#                                   [--jobs N] [--build-root DIR] [--release] [--profile NAME] [--cmake-args ...]
# Steps: (1) pinned runtime debs -> <prefix>/deps (skipped with --deps DIR); (2) ws/third_party/build_third_party.sh:
# ws/third_party.repos + patches -> FAST-LIO and patched Smac into <prefix>/install; (3) colcon build ws/src into
# the SAME <prefix>/install (overlay on top); (4) <prefix>/manifest.yaml (sha, dirty, third-party pins, patch sha256).
# Afterwards `source <prefix>/install/setup.bash` is all that is needed: lio_binary, projector_prefix and the Smac
# overlay then resolve through the ament index (no QAZBOT_SMAC_PREFIX needed). --prefix is required (outside the repo).
# --release: Release, BUILD_TESTING=OFF (no test deps needed); default is Release with tests. --build-root: where
# build/log/tp_src live (default <prefix>/work). Does not touch /home/agi/prod or any running process.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"; WS="$REPO/ws"
PREFIX=""; DEPS=""; DEB_CACHE=""; GIT_CACHE=""; NOFETCH=0; JOBS=2; WORK=""; RELEASE=0; PROFILE="x2_d1"
while [ $# -gt 0 ]; do case "$1" in
  --prefix) PREFIX="$2"; shift 2;; --deps) DEPS="$2"; shift 2;; --deb-cache) DEB_CACHE="$2"; shift 2;;
  --git-cache) GIT_CACHE="$2"; shift 2;; --no-fetch) NOFETCH=1; shift;; --jobs) JOBS="$2"; shift 2;;
  --build-root) WORK="$2"; shift 2;; --release) RELEASE=1; shift;; --profile) PROFILE="$2"; shift 2;;
  *) echo "unknown arg $1" >&2; exit 2;; esac; done
[ -n "$PREFIX" ] || { echo "REFUSED: --prefix DIR is required (build outside the repository)" >&2; exit 2; }
PREFIX="$(realpath -m -- "$PREFIX")"; WORK="$(realpath -m -- "${WORK:-$PREFIX/work}")"
case "$PREFIX/" in "$REPO"/*) echo "REFUSED: --prefix must be outside the repository ($REPO)" >&2; exit 2;; esac
mkdir -p "$PREFIX" "$WORK"
if [ -z "$DEPS" ]; then
  DEPS="$PREFIX/deps"
  if [ "$NOFETCH" = 0 ] || [ ! -d "$DEPS" ]; then
    "$WS/third_party/fetch_runtime_debs.sh" "$DEPS" ${DEB_CACHE:+--cache "$DEB_CACHE"}
  fi
fi
nice -n 19 "$WS/third_party/build_third_party.sh" --src "$WORK/tp_src" --build-base "$WORK/build_tp" \
  --install-base "$PREFIX/install" --deps "$DEPS" --jobs "$JOBS" ${GIT_CACHE:+--git-cache "$GIT_CACHE"} \
  $([ "$NOFETCH" = 1 ] && echo --no-fetch)
set +u; source /opt/ros/humble/setup.bash; set -u
CM=(-DCMAKE_BUILD_TYPE=Release); [ "$RELEASE" = 1 ] && CM+=(-DBUILD_TESTING=OFF)
MAKEFLAGS="-j$JOBS" nice -n 19 colcon --log-base "$WORK/log" build --base-paths "$WS/src" --build-base "$WORK/build" \
  --install-base "$PREFIX/install" --parallel-workers "$JOBS" --cmake-args "${CM[@]}" --event-handlers console_direct-
SHA="$(git -C "$REPO" rev-parse HEAD)"
DIRTY=false; [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || DIRTY=true
python3 - "$REPO" "$PREFIX" "$SHA" "$DIRTY" "$PROFILE" <<'PY'
import datetime, getpass, hashlib, json, os, socket, subprocess, sys, yaml
repo, out, sha, dirty, profile = sys.argv[1:]
ws = os.path.join(repo, 'ws')
fsha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
dpkg = subprocess.run(['dpkg-query', '-W', '-f', '${Package} ${Version}\n'], capture_output=True, text=True).stdout
ros = dict(l.split(' ', 1) for l in dpkg.splitlines() if l.startswith('ros-humble-'))
repos = yaml.safe_load(open(os.path.join(ws, 'third_party.repos')))['repositories']
pd = os.path.join(ws, 'third_party', 'patches')
patches = {f'{d}/{f}': fsha(os.path.join(pd, d, f)) for d in sorted(os.listdir(pd)) for f in sorted(os.listdir(os.path.join(pd, d)))}
inst = os.path.join(out, 'install')
j = lambda p: json.load(open(p)) if os.path.isfile(p) else None
manifest = {
    'sha': sha, 'dirty': dirty == 'true',
    'date_utc': datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'), 'profile': profile,
    'builder': f'{getpass.getuser()}@{socket.gethostname()}', 'arch': os.uname().machine,
    'commit_subject': subprocess.run(['git', '-C', repo, 'log', '-1', '--format=%s'], capture_output=True, text=True).stdout.strip(),
    'third_party': {k: {'url': v['url'], 'version': v['version']} for k, v in repos.items()},
    'third_party_patches_sha256': patches,
    'third_party_builds': {'fast_lio': j(os.path.join(inst, 'fast_lio', 'share', 'fast_lio', 'qazbot_build.json')),
                           'nav2_smac_planner': j(os.path.join(inst, 'nav2_smac_planner', 'qazbot_bounds_fix.json'))},
    'runtime_debs_lock_sha256': fsha(os.path.join(ws, 'third_party', 'runtime_debs.lock')),
    'dpkg_ros_humble': ros,
}
with open(os.path.join(out, 'manifest.yaml'), 'w') as f:
    f.write('# control_backend build manifest (docker/tools/build_workspace.sh)\n')
    yaml.safe_dump(manifest, f, sort_keys=False)
PY
cat <<MSG
build complete: $PREFIX/install  (manifest: $PREFIX/manifest.yaml, sha ${SHA:0:12}, dirty=$DIRTY)
use:  source /opt/ros/humble/setup.bash && source $PREFIX/install/setup.bash
      QAZBOT_RUNTIME=$DEPS source $PREFIX/install/qazbot_bringup/lib/qazbot_bringup/env.sh   # robot env (optional for lab)
test: ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon --log-base $WORK/log test --base-paths $WS/src --build-base $WORK/build --install-base $PREFIX/install
MSG
