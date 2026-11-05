#!/usr/bin/env bash
# Writes the manifest nav_full.launch.py checks (sha256 of the locally built, bounds-patched Smac library).
set -e
TWIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
P="$TWIN_ROOT/runtime_x86/build_ws/install/nav2_smac_planner"
python3 - "$P" "$TWIN_ROOT/candidates/nav2_smac_bounds/bounds.patch" <<'PY'
import hashlib, json, sys
p, patch = sys.argv[1], sys.argv[2]
sha = lambda f: hashlib.sha256(open(f, 'rb').read()).hexdigest()
json.dump({'library_sha256': sha(p + '/lib/libnav2_smac_planner_2d.so'), 'patch_sha256': sha(patch),
           'base': 'navigation2 1.1.20 (x86 twin build)'}, open(p + '/qazbot_bounds_fix.json', 'w'), indent=1)
PY
