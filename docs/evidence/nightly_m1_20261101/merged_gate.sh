#!/usr/bin/env bash
set -euo pipefail
python3 -m pytest -q tests/nightly/test_engineering_lead.py tests/nightly/test_execution_modes.py tests/nightly/test_labctl.py tests/nightly/test_lab.py tests/nightly/test_mergeq.py tests/nightly/test_stable_branches.py tests/nightly/test_runtime_qualification.py tests/motion_adapter
python3 tools/twin/verify_robot_baseline.py
