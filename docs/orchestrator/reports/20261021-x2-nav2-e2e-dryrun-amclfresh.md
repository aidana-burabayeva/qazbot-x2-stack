# amcl freshness fix (implementer)
Files: ws/src/navigation/loc_trust.py, localization_trust_node.py, tests/loc_trust/test_trust_gaps.py (backups *.bak_20261021_pre_amclfresh).
Rule: T1_amcl_pose_stale / T1_particle_cloud_stale only if odom moved >= amcl_update_min_d 0.25 m or >= amcl_update_min_a 0.2 rad since odom_at_amcl / odom_at_cloud for > pose_max_age_moving_s (3.0, unchanged). New stamp resets timer. odom_at_*=None -> not due. moving is status only.
Node: odom_src span = pose_max_age_moving_s + 1 s (was 1 s); odom_at_amcl interpolated at amcl_pose stamp (fallback latest odom); odom_at_cloud = odom at receipt; late-pin if odom absent at receipt. Side issue 3.4 (_tf stamp-0) and _moving source-stamp change NOT done (not requested).
Tests: loc_trust 46 -> 51 (test_trust_gaps 17 -> 22: 2 "while moving" tests replaced by 7). motion_adapter 43 / stop criterion 7 unchanged, all OK.
AMCL params: localization.launch.py:49-54 sets only frames/topics/max_beams/motion model, no update_min_*. Defaults 0.25/0.2: navigation2-1.1.18 nav2_amcl/src/amcl_node.cpp:201,205 (source copy under scratchpad snap/.../candidates/nav2_smac_bounds/src); runtime binary is runtime/qazbot_nav_deps/.../nav2_amcl/amcl (no source in runtime/). Not verified via param get (forbidden).
Not restarted: running trust node still has old code until orchestrator restarts it. e2e not run.

## Cycle 1 fixes (verifier FAIL)
1. loc_trust._amcl_overdue: due = |dx|>d or |dy|>d or |wrap(dyaw)|>a, strict, odom-frame deltas (nav2_amcl 1.1.18 amcl_node.cpp:775-777 shouldUpdateFilter, pose in odom_frame_id). Tests: 0.30 m diagonal not due, 0.36 m diagonal due, exact threshold not due.
2. node _amcl/_cloud: a message whose stamp SourceStamp rejects (duplicate/old/invalid) is ignored entirely (cov, stamp, cloud, odom_at_* kept; also avoids resetting the due timer with a -inf stamp). Behaviour change: previously cov/stamp were overwritten with -inf stamp. Tests NodeStampReject (2, node built via __new__, no rclpy init).
Results: loc_trust 16+27+13 = 56 OK; motion_adapter 33+6+1+3 OK; stop criterion 7 OK.
