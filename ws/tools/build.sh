#!/usr/bin/env bash
# THE build of control_backend, in place: <repo>/ws/{build,install,log} (standard colcon, run from <repo>/ws).
#   ws/tools/build.sh [--clean] [--deb-cache DIR] [--git-cache DIR] [--jobs N]
# Run it from any directory; the repository is the one containing this script. Steps:
#   1. refuse while any process runs from <repo>/ws/install (stop the stack first; never rebuild under a running stack)
#   2. --clean: delete ws/build ws/install ws/log (ws/src/third_party is kept; it is verified in step 4)
#   3. pinned runtime debs (ws/third_party/runtime_debs.lock, sha256) -> ws/install/_runtime_deps (recreated every build;
#      env.sh uses it). Offline: --deb-cache DIR or $QAZBOT_DEB_CACHE (e.g. the archive third_party_debs directory).
#   4. ws/third_party/import_sources.py: ws/third_party.repos at the pinned commits + ws/third_party/patches
#      -> ws/src/third_party (gitignored, COLCON_IGNORE; idempotent, refuses a modified checkout)
#   5. colcon (cwd ws/, defaults build/ install/ log/): fast_lio (Release, USE_LIVOX=OFF), nav2_smac_planner
#      (RelWithDebInfo) from ws/src/third_party, then every package of ws/src (Release, tests built)
#   6. provenance: nav2_smac_planner/qazbot_bounds_fix.json, fast_lio/share/fast_lio/qazbot_build.json and
#      ws/install/qazbot_build_manifest.yaml (sha, dirty, describe, release tag, date, third-party pins + patch sha256,
#      runtime_debs_lock_sha256, dpkg ros-humble versions) -- the production gate (bringup_info.py) checks it.
# Any failure stops the build with a non-zero exit (set -e). nice -n 19, --parallel-workers N (default 2).
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"; WS="$REPO/ws"
CLEAN=0; DEB_CACHE="${QAZBOT_DEB_CACHE:-}"; GIT_CACHE="${QAZBOT_TP_GIT_CACHE:-}"; JOBS=2
while [ $# -gt 0 ]; do case "$1" in
  --clean) CLEAN=1; shift;; --deb-cache) DEB_CACHE="$2"; shift 2;; --git-cache) GIT_CACHE="$2"; shift 2;;
  --jobs) JOBS="$2"; shift 2;;
  -h|--help) sed -n '2,16p' "$0"; exit 0;; *) echo "unknown arg $1" >&2; exit 2;; esac; done

# 1. never rebuild under a running stack
for p in /proc/[0-9]*; do
  [ "${p#/proc/}" = "$$" ] && continue
  exe="$(readlink "$p/exe" 2>/dev/null || true)"; cmd="$( { tr '\0' ' ' < "$p/cmdline"; } 2>/dev/null || true)"
  case "$exe $cmd" in *"$WS/install/"*)
    echo "REFUSED: pid ${p#/proc/} runs from $WS/install: ${cmd:0:120}" >&2
    echo "stop the stack (Ctrl-C / SIGINT to its ros2 launch) and run ws/tools/status.sh first" >&2; exit 3;; esac
done

cd "$WS"
BEFORE="$(git -C "$REPO" status --porcelain)"
[ "$CLEAN" = 1 ] && { echo "--clean: removing ws/build ws/install ws/log"; rm -rf build install log; }

# 3. runtime debs
rm -rf install/_runtime_deps
mkdir -p install build
"$WS/third_party/fetch_runtime_debs.sh" "$WS/install/_runtime_deps" --download-dir "$WS/build/_runtime_debs" \
  ${DEB_CACHE:+--cache "$DEB_CACHE"}

# 4. third-party sources
python3 "$WS/third_party/import_sources.py" ${GIT_CACHE:+--git-cache "$GIT_CACHE"}

# 5. colcon; build environment = run-time environment of env.sh (ROS + pinned runtime debs)
set +u; source /opt/ros/humble/setup.bash; set -u
D="$WS/install/_runtime_deps"
for g in "$D/user_pkgs" "$D/qazbot_nav_deps/root" "$D/nav2_bt_deps/root"; do
  P="$g/opt/ros/humble"; [ -d "$P" ] || { echo "missing $P" >&2; exit 1; }
  export AMENT_PREFIX_PATH="$P:$AMENT_PREFIX_PATH" CMAKE_PREFIX_PATH="$P:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$P/lib:$P/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
done
export MAKEFLAGS="-j$JOBS" CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
COLCON=(nice -n 19 colcon build --parallel-workers "$JOBS" --event-handlers console_direct-)
"${COLCON[@]}" --paths src/third_party/FAST_LIO --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_LIVOX=OFF -DBUILD_TESTING=OFF
"${COLCON[@]}" --paths src/third_party/navigation2/nav2_smac_planner --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
"${COLCON[@]}" --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release

# 6. provenance + manifest
SHA="$(git -C "$REPO" rev-parse HEAD)"
DIRTY=false; [ -z "$(git -C "$REPO" status --porcelain)" ] || DIRTY=true
python3 - "$REPO" "$SHA" "$DIRTY" <<'PY'
import datetime, getpass, hashlib, json, os, socket, subprocess, sys, yaml
repo, sha, dirty = sys.argv[1:]
ws = os.path.join(repo, 'ws'); inst = os.path.join(ws, 'install'); tp = os.path.join(ws, 'src', 'third_party')
fsha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
git = lambda *a: subprocess.run(['git', '-C', repo, *a], capture_output=True, text=True).stdout.strip()
repos = yaml.safe_load(open(os.path.join(ws, 'third_party.repos')))['repositories']
pd = os.path.join(ws, 'third_party', 'patches')
patches = {f'{d}/{f}': fsha(os.path.join(pd, d, f)) for d in sorted(os.listdir(pd)) for f in sorted(os.listdir(os.path.join(pd, d)))}
smac = os.path.join(inst, 'nav2_smac_planner')
bounds = {'upstream': 'ros-navigation/navigation2', 'version': '1.1.18', 'commit': repos['navigation2']['version'],
          'fix': 'Check both worldToMap return values before using cell indices',
          'library_sha256': fsha(os.path.join(smac, 'lib', 'libnav2_smac_planner_2d.so')),
          'source_sha256': fsha(os.path.join(tp, 'navigation2', 'nav2_smac_planner', 'src', 'smac_planner_2d.cpp')),
          'patch_sha256': patches['nav2_smac_planner/0001-smac2d-world-to-map-bounds.patch'],
          'build_type': 'RelWithDebInfo', 'physical_qualification': False}
json.dump(bounds, open(os.path.join(smac, 'qazbot_bounds_fix.json'), 'w'), indent=1)
fl = os.path.join(inst, 'fast_lio')
lio = {'upstream': 'hku-mars/FAST_LIO', 'commit': repos['FAST_LIO']['version'],
       'ikd_tree': repos['FAST_LIO/include/ikd-Tree']['version'],
       'patches': {k.split('/', 1)[1]: v for k, v in patches.items() if k.startswith('fast_lio/')},
       'binary_sha256': fsha(os.path.join(fl, 'lib', 'fast_lio', 'fastlio_mapping')),
       'build_type': 'Release', 'use_livox': False}
json.dump(lio, open(os.path.join(fl, 'share', 'fast_lio', 'qazbot_build.json'), 'w'), indent=1)
dpkg = subprocess.run(['dpkg-query', '-W', '-f', '${Package} ${Version}\n'], capture_output=True, text=True).stdout
tags = [t for t in git('tag', '--points-at', 'HEAD', '--list', 'x2-release-*').splitlines() if t]
manifest = {
    'sha': sha, 'dirty': dirty == 'true',
    'describe': git('describe', '--tags', '--always', '--dirty') or None,
    'release_tag': tags[-1] if tags else None,
    'workspace': repo,
    'date_utc': datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
    'builder': f'{getpass.getuser()}@{socket.gethostname()}', 'arch': os.uname().machine,
    'commit_subject': git('log', '-1', '--format=%s'),
    'third_party': {k: {'url': v['url'], 'version': v['version']} for k, v in repos.items()},
    'third_party_patches_sha256': patches,
    'third_party_builds': {'fast_lio': lio, 'nav2_smac_planner': bounds},
    'runtime_debs_lock_sha256': fsha(os.path.join(ws, 'third_party', 'runtime_debs.lock')),
    'dpkg_ros_humble': dict(l.split(' ', 1) for l in dpkg.splitlines() if l.startswith('ros-humble-')),
}
with open(os.path.join(inst, 'qazbot_build_manifest.yaml'), 'w') as f:
    f.write('# control_backend build manifest (ws/tools/build.sh); checked by qazbot_bringup bringup_info.py\n')
    yaml.safe_dump(manifest, f, sort_keys=False)
PY
[ "$(git -C "$REPO" status --porcelain)" = "$BEFORE" ] || { echo "ERROR: the build changed files outside the ignored build dirs:" >&2; git -C "$REPO" status --porcelain >&2; exit 4; }
cat <<MSG
build complete: $WS/install  (manifest ws/install/qazbot_build_manifest.yaml: sha ${SHA:0:12}, dirty=$DIRTY)
run:   source ws/install/qazbot_bringup/lib/qazbot_bringup/env.sh       (README "Start / stop / status")
test:  cd ws && source install/setup.bash && ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon test --base-paths src && colcon test-result
MSG
