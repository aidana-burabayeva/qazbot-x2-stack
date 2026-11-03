# e2e simpletrust_01 (domain 87)
Result dir: /home/agi/control_backend/tests/integration_results/nav_e2e_simpletrust_01/results.json
rc=0; 167 checks, failed=[]; shutdown all 0 (nav2_restarted, adapter, trust, model, abnormal_children); known_defects 0.
Safety cases all ok: scan_loss, odom_loss, amcl_loss (incl. at_goal_request_clears_ready), takeover (A terminal, not revived, B succeeded), pose_jump and wrong_initial_pose (not_ready until reconfirm, no motion), driver_loss, stop criteria (nonzero_after_terminal empty), all model_pose within tol 0.1.
No ack/timeout failures; no env-vs-logic classification needed.
Load avg during run (1-min samples): 10.8-14.7 (pre-run 10.9/11.8/11.5), high but no failures.
Domain 0 untouched.
