#!/usr/bin/env bash
# Populate vendor/ from the AgiBot archives with sha256 verification. Idempotent.
#   fetch_vendor.sh [ZIP_DIR]      extract (default ZIP_DIR: $TWIN_ARCHIVES or ~/Downloads)
#   fetch_vendor.sh --verify       check the already extracted tree against vendor/manifest.sha256 (zip hashes) only
# Layout:  vendor/aimdk_sdk (SDK 1.1.0: src/, docs/)   vendor/mc (official mc x86)   vendor/sim (MuJoCo sim)   vendor/rl_deploy
# The archives are untrusted input: each goes into its own new directory and is never executed here.
set -euo pipefail
cd "$(dirname "$0")/../vendor"
MANIFEST=manifest.sha256
declare -A TARGET=(
  [aimdk-sdk-v1_1_0-aimdk-aarch64-faf4c945-artifacts.zip]="aimdk_sdk:aimdk-aarch64-faf4c945-artifacts"
  [mc-x86-v1.0.0-20260522.zip]="mc:mc-x86-v1.0.0-20260522"
  [sim_mujoco-x86-v1.0.0-20260522.zip]="sim:sim_mujoco-x86-v1.0.0-20260522"
  [aimdk-extra-x2_rl_deploy-v1.1.zip]="rl_deploy:"
)
if [ "${1:-}" = "--verify" ]; then
  [ -s "$MANIFEST" ] || { echo "no $MANIFEST" >&2; exit 1; }
  for z in "${!TARGET[@]}"; do d=${TARGET[$z]%%:*}; [ -d "$d" ] && echo "present: $d" || { echo "MISSING: $d" >&2; exit 1; }; done
  echo "vendor tree present; archive hashes are listed in $MANIFEST"; exit 0
fi
ZIPS="${1:-${TWIN_ARCHIVES:-$HOME/Downloads}}"
for z in "${!TARGET[@]}"; do
  [ -f "$ZIPS/$z" ] || { echo "missing archive $ZIPS/$z" >&2; exit 1; }
  want=$(awk -v f="$z" '$2==f {print $1}' "$MANIFEST" 2>/dev/null || true)
  have=$(sha256sum "$ZIPS/$z" | cut -d' ' -f1)
  if [ -n "$want" ] && [ "$want" != "$have" ]; then echo "HASH MISMATCH $z" >&2; exit 1; fi
  [ -n "$want" ] || echo "$have  $z" >> "$MANIFEST"
  d=${TARGET[$z]%%:*}; inner=${TARGET[$z]#*:}
  if [ -d "$d" ] && [ -n "$(ls -A "$d" 2>/dev/null)" ]; then echo "skip $d (exists)"; continue; fi
  tmp=$(mktemp -d); unzip -q "$ZIPS/$z" -d "$tmp"
  mkdir -p "$d"
  if [ -n "$inner" ] && [ -d "$tmp/$inner" ]; then cp -a "$tmp/$inner/." "$d/"; else cp -a "$tmp/." "$d/"; fi
  rm -rf "$tmp"; echo "extracted $d"
done
