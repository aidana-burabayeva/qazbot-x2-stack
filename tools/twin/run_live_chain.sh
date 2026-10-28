#!/usr/bin/env bash
# Live-adapter chain against the Virtual MC (isolated SIM). Usage: run_live_chain.sh NEW_RESULT_DIR
set -e
TWIN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TWIN_DIR/env_sim.sh"
exec python3 "$TWIN_ROOT/tests/twin/test_live_chain.py" "${1:?Provide a new result directory name}"
