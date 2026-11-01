#!/usr/bin/env bash
# One entry point for the operator simulation session.   tools/twin/operator_sim.sh {start|status|cancel|stop|build-rviz} [options]
# Simulation only: no SSH, no physical robot, no host-network container. See docs/OPERATOR_SIM.md
set -Eeuo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$HERE/operator_sim.py" "$@"
