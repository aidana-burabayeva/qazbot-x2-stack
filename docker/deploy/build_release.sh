#!/usr/bin/env bash
# Build an immutable, SHA-pinned release of control_backend into <prefix>/releases/<sha>/.
#   docker/deploy/build_release.sh --sha <commit> [--repo <url|path>] [--prefix DIR] [--profile x2_d1]
#                                  [--deb-cache DIR] [--git-cache DIR] [--jobs N]
# Layout: <prefix>/releases/<sha>/{install/,deps/,manifest.yaml}; deps = pinned runtime debs
# (ws/third_party/runtime_debs.lock), install = third-party (FAST-LIO, Smac, from ws/third_party.repos + patches)
# + ws/src. Sources come from a fresh clone checked out at exactly <sha> (never from a working tree); when --repo
# is a local working tree that has uncommitted changes, the build is refused so nobody believes those changes are
# in the release. Does NOT touch current/previous and does not start or stop anything (deploy.sh switches links).
# --prefix DIR (or an explicitly exported $QAZBOT_PROD_PREFIX) is REQUIRED, no default; --repo = the repository containing this script.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"
PREFIX="${QAZBOT_PROD_PREFIX:-}"; SHA=""; PROFILE="x2_d1"; DEB_CACHE=""; GIT_CACHE=""; JOBS=2
while [ $# -gt 0 ]; do case "$1" in
  --sha) SHA="$2"; shift 2;; --repo) REPO="$2"; shift 2;; --prefix) PREFIX="$2"; shift 2;;
  --profile) PROFILE="$2"; shift 2;; --deb-cache) DEB_CACHE="$2"; shift 2;; --git-cache) GIT_CACHE="$2"; shift 2;; --jobs) JOBS="$2"; shift 2;;
  *) echo "unknown arg $1" >&2; exit 2;; esac; done
[ -n "$PREFIX" ] || { echo "REFUSED: no release prefix; pass --prefix DIR or export QAZBOT_PROD_PREFIX explicitly (no default: the production tree is never implied)" >&2; exit 2; }
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
# One canonical build (docker/tools/build_workspace.sh, from the fresh checkout): pinned debs, third-party overlay,
# ws/src on top, manifest.yaml (sha, dirty=false because the checkout is exactly $SHA and was verified clean above).
mkdir -p "$STAGE/out"
"$STAGE/src/docker/tools/build_workspace.sh" --prefix "$STAGE/out" --build-root "$STAGE/work" --release --kind release --profile "$PROFILE" \
  --jobs "$JOBS" ${DEB_CACHE:+--deb-cache "$DEB_CACHE"} ${GIT_CACHE:+--git-cache "$GIT_CACHE"}
[ -z "$(git -C "$STAGE/src" status --porcelain)" ] || { echo "REFUSED: build modified the source checkout" >&2; exit 3; }
grep -q '^kind: release' "$STAGE/out/manifest.yaml" || { echo "REFUSED: manifest not kind: release" >&2; exit 3; }
grep -q '^dirty: false' "$STAGE/out/manifest.yaml" || { echo "REFUSED: manifest not dirty:false" >&2; exit 3; }

mv "$STAGE/out" "$REL"
rm -rf "$STAGE"
trap - ERR
chmod -R a-w "$REL"
echo "release built: $REL"
