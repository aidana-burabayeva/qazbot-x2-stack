# P8a camera chain stop (2026-10-28 ~21:21)
Method: vendor EM, no sudo: `aima em stop-app perception|orbbec_camera|hal_camera` (each "stopped, success").
Mapping: perception->PID4674, orbbec_camera->camera_container 2936 (parent ros2 launch orbbec_camera), hal_camera->3361 (em_run.sh, args hal_sensor_orin, cfg hal_camera_orin_t2d5.yaml).
Result: 3361/2936/4674 gone, not respawned after 15 s (and ~1 min later). 204720 (hal_sensor_orin) alive; lidar_pointcloud 10.02 Hz. nvargus-daemon (1196, root systemd service, needs sudo) left running.
Load avg before 10.87/13.18/18.65 (21:21:01) -> after 8.64/11.86/17.71 (21:22:28); CPU 71% idle after.
Restart: `aima em start-app hal_camera; aima em start-app orbbec_camera; aima em start-app perception` (or restart-app; order hal_camera, orbbec_camera, perception).
