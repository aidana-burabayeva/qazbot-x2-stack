# P8d vendor trim (2026-10-28 ~21:36-21:38)
Apps (aima em list-apps): atop auto_integrated camera_forward camera_occlusion_test cobridge data_collect_task_manager dc_master dc_slave dit_bridge dit_qt dm drp ec factory_calibration hal_camera hal_camera_data hal_sensor_orin hal_uwb hds_master hds_slave housekeeper lx_webrtc_app map_manager orbbec_camera ota_* perception pnc poweroff1 reboot1 robot_chores slam uwb_fusion.
Running (user run) at start: hal_sensor_orin, pnc(vectorflux), drp, hds_master/slave, housekeeper, ec, dm, slam, lx_webrtc_app, auto_integrated, robot_chores, map_manager (+ root soc1_ota).
STOPPED: drp (aima-drp-app; only SUBSCRIBES to /aima/mc/locomotion/velocity, data-recorder; ~18% CPU), lx_webrtc_app (video stream).
KEEP: hal_sensor_orin, ec, dm, housekeeper, robot_chores, hds_master/slave (uncertain-ish, ~3%), ota.
UNCERTAIN (kept): pnc/vectorflux (not a publisher of /aima/mc/locomotion/velocity, but runs static TF map->foot_contact child and is 12-18% CPU; may feed vendor RC/app_proxy; candidate for later), slam/map_manager/auto_integrated (vendor nav stack, 1-2% each), hds_*, dm.
Locomotion velocity publishers seen: motion_adapter (ours), soc0_rc3207 (RC), app_proxy, soc2_interaction_slave. mc state subscriber drp etc.
Verification after each stop: HAL 204720 alive; /aima/mc/common/state input_source=rc, action STAND_DEFAULT, fsm current_state 4 unchanged. No respawn of drp/webrtc after >30 s.
CPU: before idle 66.9%, load 13.2/8.7/12.2; after idle 79.5%, load 13.4/11.6/12.9 (1-min lagging).
Restart: aima em start-app drp; aima em start-app lx_webrtc_app
## HAL entry (read-only)
aima em get-app hal_sensor_orin: path utils/capsh_run.sh, args [], cgroup_group 'sensor', sudo True, uid/gid None, term_timeout 10000, env EM_APP_NAME=hal_sensor_orin. (hal_camera entry has args ['hal_sensor_orin']). No nice/affinity in em app config; em cfg/cgroup.yml is cgroup based. em_run.sh picks cfg/hal_sensor_orin_t2d5.yaml for EM_APP_NAME=hal_sensor_orin (soc index 1).
PID 204720: ppid 204714 (em_run.sh, ppid 1), NI 0, affinity ff, cgroup user-1001 session-241.scope => NOT tracked by em (em-server PID 2458 did not start it; not in em cgroup).
Vendor restart: stop manual PID 204720 first (kill via its owner), then `aima em start-app hal_sensor_orin` (or restart-app). Not executed.
lidar_pointcloud (/aima/hal/sensor/lidar_chest_front/lidar_pointcloud) 10.00 Hz after trim.
