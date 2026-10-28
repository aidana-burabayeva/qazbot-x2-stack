# I1: bounded observed rays — experimental candidate

Base: production c15dd6c. Not installed on X2. Not ready for deployment or final review.

The old azimuth-coverage candidate turns a 1 m floor return into +inf. This
authorizes Nav2 to clear to raytrace_max_range=3.5 m, beyond that return.
Its 10 existing unit tests pass despite this counterexample.

This candidate adds a separate clearing-only PointCloud2 source whose topic is
the resolved scan topic plus `_clearing`. The ordinary LaserScan still marks
real obstacles and feeds AMCL. It no longer clears the local ObstacleLayer;
bounded_rays clears and never marks. Scan +inf is not enabled for clearing.

Each endpoint follows an actual current-cloud bearing. Its distance is bounded
by the return, range_max and the first exit from the existing height slice:

```
ray(t): xy=t*xy_return, z=t*z_return, 0<=t<=1
lower boundary: t*(z_return - FLOOR_MARGIN_PER_M*r_return) = Z_MIN
upper boundary: t*z_return = Z_MAX
```

The shortest endpoint in each bin wins, including points inside range_min
which suppress that bin. No angular dilation, interpolation, infinity,
remembered coverage or absent-return inference is used. A floor point
(1, 0, -1) bounds the endpoint at 0.65/1.035 = 0.628019 m. A real obstacle
at 0.8 m stays in the marking scan. A measured horizontal 8 m ray can clear
the observed local area up to the unchanged Nav2 3.5 m limit.

Production native odometry and TF ownership code is preserved; the old
candidate file was not copied. Collision detection, robot radius, obstacle
range and controller behavior are unchanged. The new source must accompany
the bridge change: a scan-only simulator will not supply bounded clearing.

## New checks on PC, 2026-10-28

After `source /opt/ros/humble/setup.bash`, from this worktree:

```sh
python3 -m pytest -q -p no:cacheprovider tests/lio/test_i1_bounded_clearing.py
cmake -S tests/lio -B /tmp/qazbot_i1_observation_build
cmake --build /tmp/qazbot_i1_observation_build -j2
ctest --test-dir /tmp/qazbot_i1_observation_build --output-on-failure
```

- 14 Python tests passed: near floor/far lethal bound, competing beams,
  true return, height-slice exit, exact FOV edge, missing/invalid/partial
  cloud, near blind zone, yaw and the real on_cloud conversion callback.
- Actual installed Nav2 Humble ObstacleLayer 1.1.20: 21 assertions passed.
  Direct raytraceFreespace tests preserve a distant lethal for short/absent
  observations, retain neighbouring unknown cells and cells past 3.5 m,
  and reproduce the unsafe long-endpoint erase. Actual updateBounds with
  separate static marking and clearing observations preserves a real obstacle.
- py_compile and git diff --check passed.

The C++ harness uses the installed ObstacleLayer implementation. It does not
initialize a ROS participant, lifecycle node, TF message filter or DDS input
pipeline. It is a regression of raytracing and update ordering, not full
launch/TF/LaserScan integration and not proof of geometry on the robot.

## Remaining acceptance work

1. Repeat A/B with the new bounded source and actual bridge conversion.
   Docker socket access is currently denied on this PC. The old plant
   generated the previous candidate's LaserScan semantics; it must not be
   used to claim this implementation passed.
2. Validate endpoints against real /qazbot_lio/cloud_registered, after a
   separately authorized passive capture. Confirm the per-cloud IMU origin
   and deskewing assumptions; raw lidar_pointcloud is not a substitute.
3. Run full source/TF/lifecycle integration and, if versions differ, the
   matching production Nav2 implementation. Measure preservation of true
   obstacles and clearing of the original stale RPP cells.
4. Promote to review only after those checks. No X2 installation is authorized.

This is a conservative geometric projection within the existing 2D slice
model. It does not prove an entire vertical column is empty. Nearest-beam
selection can reduce clearing, so resolving the original RPP blockage remains
unconfirmed until the repeated A/B and real-cloud checks.
