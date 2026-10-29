#!/usr/bin/env bash
# Preflight for the host. Prints a checklist; exit 1 if something blocks `make up`.
cd "$(dirname "$0")/.."
fail=0
ok()  { printf '  [ ok ] %s\n' "$1"; }
bad() { printf '  [FAIL] %s\n' "$1"; fail=1; }
command -v docker >/dev/null && ok "docker $(docker version --format '{{.Server.Version}}' 2>/dev/null)" || bad "docker not found"
docker compose version >/dev/null 2>&1 && ok "$(docker compose version)" || bad "docker compose v2 not found"
docker buildx version >/dev/null 2>&1 && ok "buildx present (BuildKit, named contexts)" || bad "buildx missing"
[ -x vendor/mc/bin/aima-mc-app-main ] && ok "vendor/mc (official mc)" || bad "vendor/mc missing: make vendor"
[ -x vendor/sim/bin/aima-sim-app ] && ok "vendor/sim (official MuJoCo sim)" || bad "vendor/sim missing: make vendor"
[ -d vendor/aimdk_sdk/src/aimdk_msgs ] && ok "vendor/aimdk_sdk (SDK source)" || bad "vendor/aimdk_sdk missing: make vendor"
free_gb=$(df --output=avail -BG . | tail -1 | tr -dc 0-9)
[ "${free_gb:-0}" -ge 15 ] && ok "disk: ${free_gb} GB free" || bad "disk: only ${free_gb} GB free (images ~3 GB, volumes ~1 GB, build cache)"
[ "$(nproc)" -ge 8 ] && ok "cpu: $(nproc) cores" || echo "  [warn] $(nproc) cores: the real-time loops of mc may miss deadlines"
[ -n "${DISPLAY:-}" ] && ok "DISPLAY=$DISPLAY (GUI mode possible: TWIN_GUI=1)" || echo "  [info] no DISPLAY: headless only"
exit $fail
