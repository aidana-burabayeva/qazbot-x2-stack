#!/usr/bin/env bash
# Re-create the mirror links from the research root to the navigation stack (control_backend) for entries added there later.
# The research root is a complete ROOT for the tools (ws/, tests/, logs/, candidates/, docs/, tools/): its OWN content is real,
# everything that belongs to the stack is a relative link. Safe to re-run; never overwrites real files.
set -euo pipefail
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; CB="$(cd "$R/../../control_backend" && pwd)"
[ -e "$R/ws" ] || ln -s ../../control_backend/ws "$R/ws"
for D in tests logs candidates docs tools; do
  for E in $(ls -A "$CB/$D"); do
    [ -e "$R/$D/$E" ] || [ -L "$R/$D/$E" ] || { ln -s "../../../control_backend/$D/$E" "$R/$D/$E"; echo "linked $D/$E"; }
  done
done
