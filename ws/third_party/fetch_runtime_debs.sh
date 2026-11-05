#!/usr/bin/env bash
# Fetch + verify (sha256) + extract the pinned runtime debs of runtime_debs.lock into DEST/<group>/...
#   ws/third_party/fetch_runtime_debs.sh DEST [--cache DIR]
# --cache: directory with already downloaded .deb files (used when the sha256 matches; else downloaded).
# Result layout equals the legacy control_backend/runtime/{qazbot_nav_deps/root,nav2_bt_deps/root,user_pkgs}.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEST="${1:?usage: fetch_runtime_debs.sh DEST [--cache DIR]}"; shift
CACHE=""; [ "${1:-}" = "--cache" ] && CACHE="${2:?--cache DIR}"
mkdir -p "$DEST/.debs"
grep -v '^#' "$HERE/runtime_debs.lock" | while IFS=$'\t' read -r group file sha url; do
  [ -n "$group" ] || continue
  out="$DEST/.debs/$file"
  if [ -n "$CACHE" ] && [ -f "$CACHE/$file" ] && echo "$sha  $CACHE/$file" | sha256sum -c --quiet - 2>/dev/null; then
    cp "$CACHE/$file" "$out"
  elif ! { [ -f "$out" ] && echo "$sha  $out" | sha256sum -c --quiet - 2>/dev/null; }; then
    curl -fsSL -o "$out" "$url"
  fi
  echo "$sha  $out" | sha256sum -c --quiet - || { echo "sha256 mismatch: $file" >&2; exit 1; }
  case "$group" in user_pkgs) root="$DEST/user_pkgs";; *) root="$DEST/$group/root";; esac
  mkdir -p "$root"; dpkg -x "$out" "$root"
done
echo "runtime debs extracted to $DEST"
