#!/usr/bin/env bash
# Terminal entry point; lifecycle and execution remain in lab.py / nq.py.
set -euo pipefail
LAB_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
export QAZBOT_ROLE=LAB QAZBOT_LAB=1 TWIN_ENVFILE=.env.lab
export NIGHTLY_CODEX_MODE=exec NIGHTLY_ROBOT_REF=origin/robot/stable
exec python3 "$LAB_ROOT/tools/nightly/labctl.py" "$@"
