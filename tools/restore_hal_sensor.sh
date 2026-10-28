#!/usr/bin/env bash
# Operator-approved recovery of the missing vendor LiDAR HAL only.
# Run as root so capsh can restore the original run user's NET_RAW/SYS_NICE.
# No vendor files are changed; camera/MC paths are not executed.
set -euo pipefail
if [[ ${EUID} != 0 || $# != 1 ]]; then
  echo 'Usage: sudo bash restore_hal_sensor.sh /home/agi/control_backend/docs/evidence/HAL_LOG_DIR' >&2
  exit 2
fi
case "$1" in
  /home/agi/control_backend/docs/evidence/*) ;;
  *) echo 'Log path must stay under the approved docs/evidence directory' >&2; exit 2 ;;
esac
if pgrep -f 'aima-hal_sensor-app-main.*--process_name=hal_sensor_orin' >/dev/null; then
  echo 'Existing HAL sensor owner found; refusing duplicate startup' >&2
  exit 3
fi
mkdir -p "$1"
chown run:run "$1"
chmod 0755 "$1"
python3 - "$1" <<'PY'
import os
from pathlib import Path
import subprocess
import sys

# Use the existing camera component's vendor environment, without printing it
# or assuming a run-user home layout. Both components share this installation.
cameras = []
for path in Path('/proc').iterdir():
    if not path.name.isdigit():
        continue
    try:
        argv = (path / 'cmdline').read_bytes().split(b'\0')
        if b'--process_name=hal_camera' in argv and any(b'aima-hal_sensor-app-main' in x for x in argv):
            cameras.append(path)
    except (FileNotFoundError, PermissionError):
        continue
if len(cameras) != 1:
    raise SystemExit('Expected one existing camera environment; refusing guessed startup')
env = dict(x.split(b'=', 1) for x in (cameras[0] / 'environ').read_bytes().split(b'\0') if b'=' in x)
env = {k.decode(): v.decode() for k, v in env.items()}
env.update(AGIBOT_SOC_INDEX='1', EM_APP_NAME='hal_sensor_orin', LOG_PATH=sys.argv[1])
with open(Path(sys.argv[1]) / 'startup.log', 'ab', buffering=0) as log:
    p = subprocess.Popen([
        '/usr/sbin/capsh', '--caps=cap_net_raw=eip cap_sys_nice=eip cap_setpcap,cap_setuid,cap_setgid+ep',
        '--user=run', '--keep=1', '--addamb=cap_net_raw,cap_sys_nice', '--', '-c',
        'exec /agibot/software/hal_sensor_orin/bin/em_run.sh'], env=env,
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
print('HAL_LAUNCH_PID=' + str(p.pid))
PY
