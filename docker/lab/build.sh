#!/usr/bin/env bash
# (container) the canonical in-place build ws/tools/build.sh. NOT PORTED: it writes ws/build + ws/install inside the
# source tree, which compose.yaml mounts read-only (docker/README.md "Not ported yet"), so it fails loudly there.
set -euo pipefail
exec /workspace/control_backend/ws/tools/build.sh --jobs "${QAZBOT_LAB_JOBS:-2}"
