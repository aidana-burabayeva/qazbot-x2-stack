# I1 bounded clearing: candidate for passive physical integration

Production base: c15dd6c. I1 is not yet installed on X2. No movement is authorized.

The old +inf proposal treated a nearby floor point as proof of free space to
3.5 m. The first bounded prototype instead chose the shortest ray in each bin;
on actual registered clouds it was too conservative (median 0.516 m).

The final candidate uses the longest **actually measured** in-slice free
segment, capped by the nearest current in-slice obstacle in that bin. A floor
point at (1,0,-1) alone still stops at 0.628019 m. A separate measured far beam
may extend observation; absent returns never provide evidence. There is no
angular dilation or invented coverage.

Marking and clearing use two PointCloud2 sources in the same actual point
geometry: resolved_scan_topic + `_obstacles` and `_clearing`. LaserScan remains
unchanged for AMCL and Trust. This prevents a reproduced integration defect:
float32 LaserScan angle at the nominal zero-bearing bin is -1.117587e-7 rad,
which can put its marking into row 79 while an axial clearing point is in row
80 of the same grid. Keeping actual point geometry for both sources fixes it.

Only two production files change: level_bridge.py and nav_control.launch.py,
69 insertions / 4 deletions against c15dd6c. Native odometry, TF ownership,
on_odom/on_imu, level rotation, footprint, RPP collision detection, thresholds,
and the D1/AMCL architecture remain unchanged.

## Measured validation, 2026-10-28

- Unit tests: 14 bounded-geometry/conversion tests + 5 deployment/rollback
  filesystem tests passed. The latter include partial install failure and
  refusal to overwrite changed baselines or corrupt backups.
- Native odometry/clock-gap regression: 29 passed; 5 ROS Node cases deselected.
  AST comparison confirms production odometry/TF callbacks are unchanged.
- Actual Nav2 ObstacleLayer 1.1.20 on PC: 21 assertions / 1 CTest passed.
- Actual X2 Nav2 1.1.18, isolated domain 88: two complete bridge→DDS→TF→lifecycle→
  ObservationBuffer→ObstacleLayer→costmap A/B runs, width 4 m and 8 m. Each has
  11 baseline + 11 candidate checks, all passed. Baseline reproduces stale marks;
  candidate clears them with a confirmed far floor ray, keeps true obstacles,
  near-floor/far-lethal, missing/empty/partial input and outside-FOV marks,
  and leaves unobserved cells unknown. No goals, cmd_vel or MC were used.
- Real bag: 100 same-stamp cloud/odometry pairs, cloud age p95 0.06652 s,
  max 0.09249 s. After real bridge level initialization/order filtering, 71
  clouds produced scans. All original finite scan returns and marking ranges
  were preserved; 25702 clearing rays had independent actual-ray witnesses;
  FOV/unsupported-clearing violations: zero. Existing IMU-origin/2D height-slice
  assumptions apply; this does not prove an entire vertical column is empty.
- Syntax and git diff --check passed.

The native A/B is source/costmap integration, not a MuJoCo motion run or a
NavigateToPose result. RPP actions/physical motion remain to be measured after
passive installation approval. The earlier failing prototype/harness runs are
retained as diagnostic evidence, not counted as final PASS.

Native standalone Costmap teardown is an outstanding baseline issue: plain
SIGINT aborts after the functional checks in both baseline and candidate;
explicit lifecycle deactivate/cleanup followed by SIGINT exits -11 in both.
The final harness preserves that failure (it does not turn it into PASS).
The 44 passing checks above refer to functional data/costmap assertions,
not clean standalone-process shutdown or full navigation acceptance. This
requires an explicit decision before promoting beyond passive integration.

## Reproduce

```sh
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/lio/test_i1_bounded_clearing.py tests/lio/test_i1_deployment.py
cmake -S tests/lio -B /tmp/i1_layer && cmake --build /tmp/i1_layer
ctest --test-dir /tmp/i1_layer --output-on-failure
python3 tools/analyze_i1_cloud.py BAG_PATH --baseline BASELINE_LEVEL_BRIDGE --output real_check.json
```

For the actual source/costmap A/B only: ROS_DOMAIN_ID=88, an explicit process-local
UDP-only DDS profile and test-only topics/frames are mandatory. Run
tools/i1_costmap_ab.py with --baseline, --candidate, --output and --width 4/8.
Do not replay these inputs or run the bench on the hardware domain.

## Next: separately approved passive deployment

Review this exact diff and tests, then obtain explicit operator approval.
tools/deploy_i1.py checks the two installed baseline SHA256 values, refuses a
running QazBot stack, saves/verifies backups, atomically installs only the two
files and writes a deployment manifest with exact source Git SHA. It starts no
ROS or motion process. --restore verifies backup and current file provenance.

Use tools/i1_nav_udp_only.xml per process (vendor + RViz interfaces), preserving
global/vendor DDS files. HAL was separately restored after a vendor SHM crash;
do not start a duplicate. First validate live scan, both clouds, native TF,
operator-seeded AMCL/Trust and obstacle insertion/removal without motion.
Rollback on failure. Only then request permission for MC/adapter and one short
RViz goal with RC/physical stop and operator present.
