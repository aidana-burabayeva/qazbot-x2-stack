#!/usr/bin/env bash
# (container, network) third-party sources at the pinned commits + pinned runtime debs into /lab.
set -euo pipefail
WS=/workspace/control_backend/ws
python3 - "$WS/third_party.repos" /lab/tp_src <<'PY'
import os, subprocess, sys, yaml
for path, r in yaml.safe_load(open(sys.argv[1]))['repositories'].items():
    d = os.path.join(sys.argv[2], path); os.makedirs(d, exist_ok=True)
    run = lambda *a: subprocess.run(['git', *a], cwd=d, check=True)
    if not os.path.isdir(os.path.join(d, '.git')):
        run('init', '-q'); run('remote', 'add', 'origin', r['url'])
    run('fetch', '-q', '--depth', '1', 'origin', r['version']); run('checkout', '-q', '--force', 'FETCH_HEAD')
PY
"$WS/third_party/fetch_runtime_debs.sh" /lab/deps
