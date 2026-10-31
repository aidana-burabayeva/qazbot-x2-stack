#!/usr/bin/env bash
FIXPKG=/tmp/labint/digital_twin/ws/src/x2_twin_sim/x2_twin_sim
A="-7.885339914512176,12.96582446398428,0.0"; AY="-7.885339914512176,12.96582446398428,1.57"; B="-8.535,-15.434,0.002"
for V in base fixed; do
  F=""; [ $V = fixed ] && F=$FIXPKG
  PROBE_SECS=35 ./frame_ab.sh ${V}_A_static1 "$A" "" $F
  PROBE_SECS=35 ./frame_ab.sh ${V}_A_static2 "$A" "" $F
  PROBE_SECS=90 ./frame_ab.sh ${V}_A_ccw "$A" "--only-ccw" $F
  PROBE_SECS=90 ./frame_ab.sh ${V}_A_cw "$A" "--only-cw" $F
  PROBE_SECS=35 ./frame_ab.sh ${V}_AY_static "$AY" "" $F
  PROBE_SECS=90 ./frame_ab.sh ${V}_AY_ccw "$AY" "--only-ccw" $F
  PROBE_SECS=35 ./frame_ab.sh ${V}_B_static "$B" "" $F
  PROBE_SECS=90 ./frame_ab.sh ${V}_B_ccw "$B" "--only-ccw" $F
done
echo ALL_AB_DONE
