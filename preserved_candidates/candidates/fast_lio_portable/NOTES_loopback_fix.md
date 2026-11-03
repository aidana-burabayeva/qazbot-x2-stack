# FAST-LIO portable binary: loop-back time_buffer fix (2026-10-22)
- Patch applied to src/FAST_LIO: lidar_loopback_time_buffer.patch (clears time_buffer, lidar_pushed on "lidar loop back").
- Build: cd candidates/fast_lio_portable && source /opt/ros/humble/setup.bash && source ws/src/navigation/env.sh && colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release (about 2.5 min; log build.log, old log build.log.bak_20261022_pre_loopback).
- Old binary: install/fast_lio/lib/fast_lio/fastlio_mapping.bak_20261022_pre_loopback, sha256 4097314bba6df4e2a358625bdb06897cbd3cd50f1d14fb6f2986572101237064
- New binary sha256 63903a5c42e78438a97493d3a9cd3e8e26eb079932d4bf77746c342f093c17e8
- Offline replay (domain 88): 609/609 poses identical to original, odom latency p50 26 ms.
