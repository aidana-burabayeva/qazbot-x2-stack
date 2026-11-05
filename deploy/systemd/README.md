# qazbot-vendor-trim (2026-11-05)

Why: EM `default_apps` (firmware, `/agibot/software/entry/cfg/run_agibot.yaml`, root/run, no sudo) start
perception, orbbec_camera, hal_camera, drp, lx_webrtc_app on every boot. Navigation does not need them (operator decision 2026-10-28, P8a).
Firmware config is not edited. A user unit stops them after boot with `aima em stop-app` (6 passes x 20 s after EM is reachable).

Install (robot, no sudo): copy `qazbot_vendor_trim.sh` to `~/.config/qazbot/`, unit to `~/.config/systemd/user/`,
`systemctl --user daemon-reload && systemctl --user enable qazbot-vendor-trim.service` (user linger is on).
Log: `~/control_backend/logs/vendor_trim.log`. Undo: `systemctl --user disable qazbot-vendor-trim.service`.
Restart vendor app manually: `aima em start-app hal_camera; aima em start-app orbbec_camera; aima em start-app perception`.
Status: installed and enabled; test run with apps already stopped OK. Boot-time behaviour NOT yet verified (needs a reboot, operator permission).
