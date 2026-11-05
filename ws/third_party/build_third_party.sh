#!/usr/bin/env bash
# Reproducible build of the patched third-party packages (FAST-LIO, Smac 2D bounds fix).
#   ws/third_party/build_third_party.sh [--src DIR] [--build-base DIR] [--install-base DIR] [--deps DIR] [--jobs N]
#                                       [--no-fetch]   (offline: sources already fetched into --src)
#                                       [--git-cache DIR]  (local clones laid out like --src; used before the network)
# --deps: root produced by fetch_runtime_debs.sh (Nav2 1.1.18 debs that are not dpkg-installed on X2).
# Sources: ws/third_party.repos (exact commits) + ws/third_party/patches. Flags = the live builds (CMakeCache):
# fast_lio Release, USE_LIVOX=OFF; nav2_smac_planner RelWithDebInfo. Writes qazbot_bounds_fix.json (checked by
# qazbot_navigation nav_full.launch.py) and share/fast_lio/qazbot_build.json.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
SRC="$HERE/src"; BUILD="$HERE/build"; INSTALL="$HERE/install"; DEPS=""; JOBS=2; FETCH=1; GIT_CACHE="${QAZBOT_TP_GIT_CACHE:-}"
while [ $# -gt 0 ]; do case "$1" in
  --src) SRC="$2"; shift 2;; --build-base) BUILD="$2"; shift 2;; --install-base) INSTALL="$2"; shift 2;;
  --deps) DEPS="$2"; shift 2;; --jobs) JOBS="$2"; shift 2;; --no-fetch) FETCH=0; shift;; --git-cache) GIT_CACHE="$2"; shift 2;; *) echo "unknown arg $1" >&2; exit 2;; esac; done
set +u; source /opt/ros/humble/setup.bash; set -u
if [ -n "$DEPS" ]; then
  for g in nav2_bt_deps/root qazbot_nav_deps/root user_pkgs; do
    P="$DEPS/$g/opt/ros/humble"; [ -d "$P" ] || { echo "missing $P" >&2; exit 1; }
    export AMENT_PREFIX_PATH="$P:$AMENT_PREFIX_PATH" CMAKE_PREFIX_PATH="$P:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$P/lib:$P/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
  done
fi
mkdir -p "$SRC"
python3 - "$WS/third_party.repos" "$SRC" "$FETCH" "$GIT_CACHE" <<'PY'
import os, subprocess, sys, yaml
repos, src, fetch, cache = yaml.safe_load(open(sys.argv[1]))['repositories'], sys.argv[2], sys.argv[3] == '1', sys.argv[4]
# abort stalled HTTPS transfers (< 1 kB/s for 60 s) instead of hanging; retried below
NET = ['-c', 'http.lowSpeedLimit=1000', '-c', 'http.lowSpeedTime=60']
def git(*a, cwd=None):
    return subprocess.run(['git', *a], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()
for path, r in repos.items():          # parents before nested paths (dict order of the file)
    d = os.path.join(src, path)
    if not os.path.isdir(os.path.join(d, '.git')):
        os.makedirs(d, exist_ok=True)
        git('init', '-q', cwd=d); git('remote', 'add', 'origin', r['url'], cwd=d)
    if fetch:
        c = os.path.join(cache, path) if cache else ''
        if c and os.path.isdir(os.path.join(c, '.git')) and subprocess.run(
                ['git', '-C', c, 'cat-file', '-e', r['version'] + '^{commit}'], capture_output=True).returncode == 0:
            git('fetch', '-q', '--depth', '1', c, r['version'], cwd=d)   # local cache; the sha check below still applies
        else:
            for attempt in range(3):
                if subprocess.run(['git', *NET, 'fetch', '-q', '--depth', '1', 'origin', r['version']], cwd=d).returncode == 0:
                    break
            else:
                sys.exit(f'{path}: fetch of {r["version"]} failed')
    git('checkout', '-q', '--force', r['version'], cwd=d)
    git('clean', '-fdq', cwd=d)      # drop earlier patch results; nested repos (ikd-Tree) are kept by git clean -f
    head = git('rev-parse', 'HEAD', cwd=d)
    if head != r['version']:
        sys.exit(f'{path}: HEAD {head} != pinned {r["version"]}')
    print(f'{path} @ {head}')
PY
apply() {  # $1 repo dir (freshly checked out + cleaned above), $2 patch dir
  for p in "$2"/*.patch; do
    git -C "$1" apply --check "$p" || { echo "patch does not apply: $p" >&2; exit 1; }
    git -C "$1" apply "$p"; echo "applied $(basename "$p")"
  done
}
apply "$SRC/FAST_LIO" "$HERE/patches/fast_lio"
apply "$SRC/navigation2" "$HERE/patches/nav2_smac_planner"
# Only these two packages are built (base path = the package itself; the other navigation2 packages come from
# /opt/ros/humble + the pinned runtime debs, exactly as in the live build environment).
COMMON=(--build-base "$BUILD" --install-base "$INSTALL" --parallel-workers 1 --event-handlers console_direct-)
export MAKEFLAGS="-j$JOBS" CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon --log-base "$BUILD/log" build "${COMMON[@]}" --base-paths "$SRC/FAST_LIO" --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_LIVOX=OFF -DBUILD_TESTING=OFF
colcon --log-base "$BUILD/log" build "${COMMON[@]}" --base-paths "$SRC/navigation2/nav2_smac_planner" --packages-select nav2_smac_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
python3 - "$WS" "$SRC" "$INSTALL" <<'PY'
import hashlib, json, os, sys, yaml
ws, src, inst = sys.argv[1:]
sha = lambda f: hashlib.sha256(open(f, 'rb').read()).hexdigest()
repos = yaml.safe_load(open(os.path.join(ws, 'third_party.repos')))['repositories']
patch = os.path.join(ws, 'third_party', 'patches', 'nav2_smac_planner', '0001-smac2d-world-to-map-bounds.patch')
smac = os.path.join(inst, 'nav2_smac_planner')
json.dump({'upstream': 'ros-navigation/navigation2', 'version': '1.1.18', 'commit': repos['navigation2']['version'],
           'fix': 'Check both worldToMap return values before using cell indices',
           'library_sha256': sha(os.path.join(smac, 'lib', 'libnav2_smac_planner_2d.so')),
           'source_sha256': sha(os.path.join(src, 'navigation2', 'nav2_smac_planner', 'src', 'smac_planner_2d.cpp')),
           'patch_sha256': sha(patch), 'build_type': 'RelWithDebInfo', 'physical_qualification': False},
          open(os.path.join(smac, 'qazbot_bounds_fix.json'), 'w'), indent=1)
fl = os.path.join(inst, 'fast_lio')
json.dump({'upstream': 'hku-mars/FAST_LIO', 'commit': repos['FAST_LIO']['version'],
           'ikd_tree': repos['FAST_LIO/include/ikd-Tree']['version'],
           'patches': {p: sha(os.path.join(ws, 'third_party', 'patches', 'fast_lio', p))
                       for p in sorted(os.listdir(os.path.join(ws, 'third_party', 'patches', 'fast_lio')))},
           'binary_sha256': sha(os.path.join(fl, 'lib', 'fast_lio', 'fastlio_mapping')),
           'build_type': 'Release', 'use_livox': False},
          open(os.path.join(fl, 'share', 'fast_lio', 'qazbot_build.json'), 'w'), indent=1)
print('manifests written')
PY
