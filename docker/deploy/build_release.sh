#!/usr/bin/env bash
# Build an immutable, SHA-pinned release of control_backend into <prefix>/releases/<sha>/.
#   docker/deploy/build_release.sh --sha <commit> [--repo <url|path>] [--prefix DIR] [--profile x2_d1]
#                                  [--deb-cache DIR] [--git-cache DIR] [--jobs N]
# Layout: <prefix>/releases/<sha>/{install/,deps/,manifest.yaml}; deps = pinned runtime debs
# (ws/third_party/runtime_debs.lock), install = third-party (FAST-LIO, Smac, from ws/third_party.repos + patches)
# + ws/src. Sources come from a fresh clone checked out at exactly <sha> (never from a working tree); when --repo
# is a local working tree that has uncommitted changes, the build is refused so nobody believes those changes are
# in the release. Does NOT touch current/previous and does not start or stop anything (deploy.sh switches links).
# Defaults: --prefix $QAZBOT_PROD_PREFIX or $HOME/prod; --repo = the repository containing this script.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"
PREFIX="${QAZBOT_PROD_PREFIX:-$HOME/prod}"; SHA=""; PROFILE="x2_d1"; DEB_CACHE=""; GIT_CACHE=""; JOBS=2
while [ $# -gt 0 ]; do case "$1" in
  --sha) SHA="$2"; shift 2;; --repo) REPO="$2"; shift 2;; --prefix) PREFIX="$2"; shift 2;;
  --profile) PROFILE="$2"; shift 2;; --deb-cache) DEB_CACHE="$2"; shift 2;; --git-cache) GIT_CACHE="$2"; shift 2;; --jobs) JOBS="$2"; shift 2;;
  *) echo "unknown arg $1" >&2; exit 2;; esac; done
[ -n "$SHA" ] || { echo "--sha is required" >&2; exit 2; }
if [ -d "$REPO/.git" ] || git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
  if [ -n "$(git -C "$REPO" status --porcelain --untracked-files=no)" ]; then
    echo "REFUSED: $REPO has uncommitted changes; commit/push first (release = exact commit only)" >&2; exit 3
  fi
  SHA="$(git -C "$REPO" rev-parse --verify "$SHA^{commit}")"
fi
[[ "$SHA" =~ ^[0-9a-f]{40}$ ]] || { echo "--sha must resolve to a full commit id, got $SHA" >&2; exit 2; }
REL="$PREFIX/releases/$SHA"
[ -e "$REL" ] && { echo "REFUSED: $REL exists (releases are immutable)" >&2; exit 4; }
STAGE="$PREFIX/releases/.staging-$SHA"
rm -rf "$STAGE"; mkdir -p "$STAGE"
trap 'echo "build failed; staging kept for inspection: $STAGE" >&2' ERR
git clone -q --no-checkout "$REPO" "$STAGE/src"
git -C "$STAGE/src" checkout -q --detach "$SHA"
[ -z "$(git -C "$STAGE/src" status --porcelain)" ] || { echo "REFUSED: checkout of $SHA not clean" >&2; exit 3; }
WS="$STAGE/src/ws"

mkdir -p "$STAGE/out"
"$WS/third_party/fetch_runtime_debs.sh" "$STAGE/out/deps" ${DEB_CACHE:+--cache "$DEB_CACHE"}
nice -n 19 "$WS/third_party/build_third_party.sh" --src "$STAGE/tp_src" --build-base "$STAGE/build_tp" \
  --install-base "$STAGE/out/install" --deps "$STAGE/out/deps" --jobs "$JOBS" ${GIT_CACHE:+--git-cache "$GIT_CACHE"}
set +u; source /opt/ros/humble/setup.bash; set -u
MAKEFLAGS="-j$JOBS" nice -n 19 colcon --log-base "$STAGE/log" build --base-paths "$WS/src" --build-base "$STAGE/build" \
  --install-base "$STAGE/out/install" --parallel-workers "$JOBS" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct-
[ -z "$(git -C "$STAGE/src" status --porcelain)" ] || { echo "REFUSED: build modified the source checkout" >&2; exit 3; }

python3 - "$STAGE/src" "$STAGE/out" "$SHA" "$PROFILE" <<'PY'
import datetime, getpass, hashlib, json, os, socket, subprocess, sys, yaml
src, out, sha, profile = sys.argv[1:]
ws = os.path.join(src, 'ws')
fsha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
dpkg = subprocess.run(['dpkg-query', '-W', '-f', '${Package} ${Version}\n'], capture_output=True, text=True).stdout
ros = dict(l.split(' ', 1) for l in dpkg.splitlines() if l.startswith('ros-humble-'))
repos = yaml.safe_load(open(os.path.join(ws, 'third_party.repos')))['repositories']
pd = os.path.join(ws, 'third_party', 'patches')
patches = {f'{d}/{f}': fsha(os.path.join(pd, d, f)) for d in sorted(os.listdir(pd)) for f in sorted(os.listdir(os.path.join(pd, d)))}
inst = os.path.join(out, 'install')
def j(p):
    return json.load(open(p)) if os.path.isfile(p) else None
manifest = {
    'sha': sha,
    'date_utc': datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
    'profile': profile,
    'builder': f'{getpass.getuser()}@{socket.gethostname()}',
    'arch': os.uname().machine,
    'commit_subject': subprocess.run(['git', '-C', src, 'log', '-1', '--format=%s'], capture_output=True, text=True).stdout.strip(),
    'third_party': {k: {'url': v['url'], 'version': v['version']} for k, v in repos.items()},
    'third_party_patches_sha256': patches,
    'third_party_builds': {'fast_lio': j(os.path.join(inst, 'fast_lio', 'share', 'fast_lio', 'qazbot_build.json')),
                           'nav2_smac_planner': j(os.path.join(inst, 'nav2_smac_planner', 'qazbot_bounds_fix.json'))},
    'runtime_debs_lock_sha256': fsha(os.path.join(ws, 'third_party', 'runtime_debs.lock')),
    'dpkg_ros_humble': ros,
}
with open(os.path.join(out, 'manifest.yaml'), 'w') as f:
    f.write('# control_backend release manifest (docker/deploy/build_release.sh)\n')
    yaml.safe_dump(manifest, f, sort_keys=False)
PY
mv "$STAGE/out" "$REL"
rm -rf "$STAGE"
trap - ERR
chmod -R a-w "$REL"
echo "release built: $REL"
