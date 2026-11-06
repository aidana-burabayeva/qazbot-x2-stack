# Host user units (production runs on the host, not in Docker)

`qazbot-vendor-trim.service` + `qazbot_vendor_trim.sh`: after every boot stops vendor apps not needed for navigation
(hal_camera orbbec_camera perception drp lx_webrtc_app) via `aima em stop-app` (6 passes x 20 s once EM is reachable).
Firmware config (EM `default_apps`, root-owned) is not edited. Log: `${XDG_STATE_HOME:-~/.local/state}/qazbot/vendor_trim.log`.

AgiBot docs: `~/aimdk/extra/mc-rl/README_en.md` uses `aima em stop-app/start-app`; docs are silent on trimming default_apps.

Install (no sudo, idempotent): `ws/tools/systemd/install_user_units.sh [--dry-run]`
Undo: `ws/tools/systemd/install_user_units.sh --uninstall` (= `systemctl --user disable qazbot-vendor-trim.service`).
Restore vendor apps manually: `aima em start-app hal_camera; aima em start-app orbbec_camera; aima em start-app perception`.
Boot-time behaviour must be verified on a real reboot (operator permission).
