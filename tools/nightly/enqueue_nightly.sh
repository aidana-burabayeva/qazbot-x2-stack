#!/usr/bin/env bash
# Existing systemd timer -> existing queue, capability-aware and idempotent.
set -euo pipefail
cd "$(dirname "$0")/../.."
exec python3 tools/nightly/workflow.py enqueue
