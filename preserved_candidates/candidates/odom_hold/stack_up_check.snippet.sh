# Insert in tools/stack/stack_up.sh after the existing native_odom_bridge wait_log (line ~60), inside the native_imu if-block.
# Fails startup unless the bridge started with standing_hold=True (hold silently off = fix not active) and no ERROR on import.
  grep -q 'native_odom_bridge started: .*standing_hold=True' "$LOGS/localization_$TAG.log" \
    || die "native_odom_bridge standing_hold!=True (aimdk_msgs import failed? see 'standing hold ENABLED but' in $LOGS/localization_$TAG.log)"
