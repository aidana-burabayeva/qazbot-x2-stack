#!/usr/bin/env bash
# x86 equivalent of tests/nav_e2e/run_model.sh: same suite, same isolation, twin environment.
set -e
TWIN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TWIN_DIR/env_sim.sh"
exec python3 "$TWIN_ROOT/tests/nav_e2e/run_suite_real.py" "${1:?Provide a new result directory name}"
