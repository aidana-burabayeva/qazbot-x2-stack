# Offline P20 and native-odometry regression, 2026-10-29

The navigation branch's P20 motion adapter and native odometry guard were fetched via GitHub into an isolated local snapshot. Their Git blob IDs are respectively `726dbeb618a287d237bd102e50d51a3cbb07214d` and `d05e3bf044e97cd20c30af29ad3ccf4e319d1462`. Existing native-odometry tests and their trial06 fixture were also used; the local fixture matches its Git blob ID `25dcc4359ccde35c2a9977352b8135f9b769e79d`.

`python3 -m pytest -q -p no:cacheprovider tests/motion_adapter/test_yaw_rate_loop.py tests/lio/test_reset_scenario.py tests/lio/test_native_odom_clock_gaps.py tests/lio/test_native_odom.py`: **42 passed, 6 skipped**. The skips need ROS/level_bridge dependencies. No physical robot was contacted.

The new reset test covers 0.467 m and 1.563 m discontinuities after 0.1 s and 40 s. The existing source guard keeps the last good pose and latches on the jump. This is an offline component test, not a mode-change simulation or a test of Trust, recovery, reanchoring, or resumed motion. The current code has no authenticated reset evidence input; there is no basis to automatically infer a reset from a gap or jump.

The separate digital twin directory and its live-chain sources are present. The Docker daemon socket was inaccessible from the sandbox; MuJoCo Python bindings and ROS Python imports were unavailable on the host. Thus official-MC/MuJoCo or lite runs, S2/S3/S6 and the I1 A/B were not executed. This snapshot does not change production, MC, localization parameters, or the I1 candidate.
