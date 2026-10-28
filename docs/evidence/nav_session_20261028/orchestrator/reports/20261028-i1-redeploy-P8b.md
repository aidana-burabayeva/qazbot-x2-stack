# P8b I1 redeploy: NOT INSTALLED (stack running; deploy tool refuses by design)
- Current SHAs == baseline (level_bridge 86c434ce..., nav_control.launch 'ebea6637...'). Staged == I1 (c817a5ed..., c4e46506...).
- deploy_i1.py writes only the 2 BASE files (+backup dir); no_stack() aborts if fastlio/level_bridge/amcl/controller_server/goal_coordinator etc. run. They run now (pids 372200,372225,372271,387302,387399). I did not bypass the guard.
- nav_full.launch.py loads nav_control.launch.py via importlib and calls ctl.setup(...) => I1 costmap sources (scan_obstacles marking + scan_clearing PointCloud2 bounded_rays) WILL be active under nav_full. No change needed. Note: level_bridge must publish scan_obstacles/scan_clearing (the other file of I1).
- Install (after stack stopped):
  python3 /tmp/i1_stage_2d240dbd/tools/deploy_i1.py --source /tmp/i1_stage_2d240dbd --source-commit 2d240dbd09ca69de87e9ac4b1acde735293ad6b2 --backup /home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd_r2
- Rollback: python3 /tmp/i1_stage_2d240dbd/tools/deploy_i1.py --restore --backup /home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd_r2
