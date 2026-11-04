# Python level_bridge -> C++ qazbot_lidar_projector switch (prepared, NOT deployed)

Base: 9208e21 (CloudPoseSync fix, verified). D-07 NOT included: `costmap_z_min -0.90` has no physical evidence.
Params: `qazbot_lidar_projector/config/lidar_projector_switch.yaml` (z_min -0.65 = production band).

Switch (operator-approved, robot standing, no active goal):
1. Build the package into a dedicated prefix from this commit.
2. Stop Nav2/GC/adapter/trust and localization (level_bridge exit shuts the launch down).
3. Start localization with `projector_prefix:=<prefix> projector_params:=<copy of the yaml>`.
4. Check TF odom->base_footprint (one owner, 10 Hz), /qazbot_lio/scan 5 Hz, scan_obstacles/scan_clearing, AMCL, fresh 2D Pose Estimate, Trust READY.

Rollback: restart localization without `projector_prefix` (default '' = unchanged python level_bridge.py), fresh pose, Trust READY.

Ownership: exactly one of python/C++ starts (IfCondition), same node name qazbot_level_bridge, same remaps; no tf_static publisher, no odom->camera_init.
