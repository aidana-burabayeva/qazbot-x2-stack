#!/bin/bash
# Download (NO install) the Nav2 Humble 1.1.18 DWB debs from the SAME ROS snapshot as runtime/qazbot_nav_deps
# (snapshots.ros.org/humble/2025-08-15) and dpkg-deb -x them into PREFIX (scratch). sha256 checked against the index.
# Usage: fetch_dwb.sh PREFIX_DIR
set -eu
PREFIX="${1:?prefix dir}"; DL="$PREFIX/_debs"; mkdir -p "$DL"
B=http://snapshots.ros.org/humble/2025-08-15/ubuntu
curl -s "$B/dists/jammy/main/binary-arm64/Packages.gz" | gunzip > "$DL/Packages"
for p in ros-humble-dwb-core ros-humble-dwb-plugins ros-humble-dwb-critics ros-humble-dwb-msgs ros-humble-costmap-queue; do
  f=$(awk -v p="$p" '$1=="Package:"{x=($2==p)} x&&$1=="Filename:"{print $2}' "$DL/Packages")
  h=$(awk -v p="$p" '$1=="Package:"{x=($2==p)} x&&$1=="SHA256:"{print $2}' "$DL/Packages")
  [ -f "$DL/$(basename "$f")" ] || curl -s -o "$DL/$(basename "$f")" "$B/$f"
  echo "$h  $DL/$(basename "$f")" | sha256sum -c -
  dpkg-deb -x "$DL/$(basename "$f")" "$PREFIX"
done
echo "DWB prefix: $PREFIX/opt/ros/humble"
