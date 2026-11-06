#!/usr/bin/env bash
# (container, no network) canonical workspace build (docker/tools/build_workspace.sh): patched FAST-LIO + Smac from the
# sources fetched by fetch.sh (/lab/tp_src) + ws/src into /lab/install, manifest /lab/manifest.yaml.
set -euo pipefail
exec /workspace/control_backend/docker/tools/build_workspace.sh --prefix /lab --build-root /lab --deps /lab/deps \
  --no-fetch --jobs "${QAZBOT_LAB_JOBS:-2}"
