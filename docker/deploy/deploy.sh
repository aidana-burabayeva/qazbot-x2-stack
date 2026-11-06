#!/usr/bin/env bash
# Point <prefix>/current at an already built release; the old target becomes <prefix>/previous.
#   docker/deploy/deploy.sh --sha <full sha> [--prefix DIR]
# Only switches symlinks (atomic rename). It never starts, stops or signals processes: restart the stack from
# <prefix>/current yourself (operator permission required on the robot), then check docker/deploy/status.sh.
set -euo pipefail
PREFIX="${QAZBOT_PROD_PREFIX:-}"; SHA=""
while [ $# -gt 0 ]; do case "$1" in
  --sha) SHA="$2"; shift 2;; --prefix) PREFIX="$2"; shift 2;; *) echo "unknown arg $1" >&2; exit 2;; esac; done
[ -n "$PREFIX" ] || { echo "REFUSED: no release prefix; pass --prefix DIR or export QAZBOT_PROD_PREFIX explicitly (no default: the production tree is never implied)" >&2; exit 2; }
[ -n "$SHA" ] || { echo "--sha is required" >&2; exit 2; }
REL="$PREFIX/releases/$SHA"
[ -f "$REL/manifest.yaml" ] || { echo "REFUSED: $REL/manifest.yaml missing (build_release.sh first)" >&2; exit 3; }
grep -q "^sha: $SHA\$" "$REL/manifest.yaml" || { echo "REFUSED: manifest sha differs from $SHA" >&2; exit 3; }
swap() {  # $1 link name, $2 target (relative to PREFIX)
  ln -sfn "$2" "$PREFIX/.$1.tmp" && mv -T "$PREFIX/.$1.tmp" "$PREFIX/$1"
}
CUR="$(readlink "$PREFIX/current" 2>/dev/null || true)"
if [ "$CUR" = "releases/$SHA" ]; then echo "already current: $SHA"; exit 0; fi
[ -n "$CUR" ] && swap previous "$CUR"
swap current "releases/$SHA"
echo "current -> releases/$SHA${CUR:+ ; previous -> $CUR}"
echo "next: restart the stack from $PREFIX/current (operator), then docker/deploy/status.sh"
