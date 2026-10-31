#!/usr/bin/env bash
# Nightly regression runner. tier lite: every scenario in its own compose project, N workers in parallel.
# tier i1: I1 A/B (MuJoCo twin) one after another in its own project, verdict = existing scripts/i1/summarize.py GO/NO-GO.
#   scripts/nightly/run_nightly.sh            env: CAMPAIGN=campaigns/nightly.tsv WORKERS=2 MIN_FREE_GB=5 TIMEOUT=900
# Output: results/nightly_<stamp>/{meta.env,<name>.log,<name>.rc,summary.json}. No retries: a failure stays a failure.
set -uo pipefail
cd "$(dirname "$0")/../.."
CAMPAIGN=${CAMPAIGN:-campaigns/nightly.tsv}
WORKERS=${WORKERS:-2}
MIN_FREE_GB=${MIN_FREE_GB:-5}
TIMEOUT=${TIMEOUT:-900}
I1_TIMEOUT=${I1_TIMEOUT:-2400}
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=results/nightly_$STAMP
BACKEND=$([ -d ../control_backend/ws ] && echo ../control_backend || echo ..)
ENVFILE=${TWIN_ENVFILE:-$(test -f .env && echo .env || echo .env.example)}   # lab: .env.lab (robot map), set by labctl/systemd

# The twin runs the stack copy baked into the image, not the live tree: bind the verdict to the exact files tested.
# Image tag `nightly` is separate from `dev`, so rebuilding never changes other twin experiments.
export TWIN_VERSION=${TWIN_VERSION:-nightly}
REBUILD=${REBUILD:-1}
STACK_LIST=$(grep -v '^#' scripts/contract/stack_list.sh)   # one definition of the contract file list (also used on the robot)
tree_hash() { (cd "$BACKEND/ws/src" && bash -c "$STACK_LIST" | sha256sum | cut -c1-16); }
image_hash() { docker run --rm --entrypoint bash -w /opt/x2twin/stack "x2-twin-stack:$TWIN_VERSION" -c "$STACK_LIST | sha256sum | cut -c1-16" 2>/dev/null; }
for img in sim test; do  # same binaries as :dev, only the tag differs (no extra disk)
  docker image inspect "x2-twin-$img:$TWIN_VERSION" >/dev/null 2>&1 || docker tag "x2-twin-$img:dev" "x2-twin-$img:$TWIN_VERSION" || exit 5
done
TREE=$(tree_hash)
if [ "$REBUILD" = 1 ]; then  # always: twin code (ws/src of digital_twin) is baked into the image too; cached = no-op
  docker compose -f docker/compose.yaml --env-file "$ENVFILE" build stack >"$OUT.build.log" 2>&1 || { echo "build failed: $OUT.build.log"; exit 4; }
fi
IMG=$(image_hash)
[ "$TREE" = "$IMG" ] || { echo "refuse: image stack $IMG != tree $TREE (REBUILD=$REBUILD)"; exit 4; }

. scripts/contract/robot.env
MAP=${TWIN_MAP:-$(sed -n 's/^TWIN_MAP=//p' "$ENVFILE" | tail -1)}; MAP=${MAP:-$ROBOT_MAP}
MAP_SHA=$(sha256sum "$BACKEND/ws/src/maps/$MAP/map.pgm" | cut -d' ' -f1)
[ "$MAP_SHA" = "$ROBOT_MAP_SHA256" ] || echo "WARNING: twin map $MAP ($MAP_SHA) is not the robot's $ROBOT_MAP"

free=$(df --output=avail -BG . | tail -1 | tr -dc 0-9)
[ "$free" -ge "$MIN_FREE_GB" ] || { echo "refuse: ${free}G free < ${MIN_FREE_GB}G"; exit 3; }
mkdir -p "$OUT"
{
  echo "stamp=$STAMP"
  echo "backend_sha=$(git -C "$BACKEND" rev-parse --short HEAD)"
  echo "backend_dirty=$(git -C "$BACKEND" status --porcelain | wc -l)"
  echo "stack_image=$(docker image inspect -f '{{.Id}} {{.Created}}' x2-twin-stack:dev)"
  echo "stack_tree_sha=$TREE"
  echo "stack_image_sha=$IMG"
  echo "image_tag=$TWIN_VERSION"
  echo "map=$MAP"
  echo "map_sha256=$MAP_SHA"
  echo "map_matches_robot_trust=$([ "$MAP_SHA" = "$ROBOT_MAP_SHA256" ] && echo yes || echo "NO (robot: $ROBOT_MAP $ROBOT_MAP_SHA256)")"
  echo "workers=$WORKERS"
  # Git only: the lab never contacts the physical X2 (the operator runs check_robot.sh by hand). Published baseline = origin/robot/stable.
  echo "robot_contract=$(PUB_OUT=$(scripts/contract/check_published.sh 2>&1); rc=$?; case $rc in 0) echo "in_sync_with_published_${ROBOT_REF}" ;; 1) echo "DRIFT: $(echo "$PUB_OUT" | sed -n '/^DRIFT/,$p' | tr -d ' \n' | cut -c1-200)" ;; *) echo "published_ref_unavailable" ;; esac)"
} > "$OUT/meta.env"

run_one() {  # name envs
  local name=$1 envs=$2 proj="x2tn-${STAMP//_/}-${1//_/-}"
  [ "$envs" = "-" ] && envs=""
  local C="docker compose -p $proj -f docker/compose.yaml --env-file $ENVFILE"
  env $envs RUN_NAME="nightly_${STAMP}_$name" timeout "$TIMEOUT" $C --profile lite run --rm scenario-lite >"$OUT/$name.log" 2>&1
  echo $? >"$OUT/$name.rc"
  $C --profile lite down -v >/dev/null 2>&1
}

while IFS=$'\t' read -r name tier envs soft; do
  case "$name" in ''|\#*) continue ;; esac
  case "$tier" in lite) ;; i1) continue ;; *) echo "skip $name: tier $tier not implemented"; continue ;; esac
  while [ "$(jobs -rp | wc -l)" -ge "$WORKERS" ]; do wait -n; done
  echo "start $name"; run_one "$name" "$envs" &
done < "$CAMPAIGN"
wait

run_i1() {  # name envs
  local name=$1 envs=$2 proj="x2tn-i1-${STAMP//_/}" tag="nightly${STAMP}_$1"
  [ "$envs" = "-" ] && envs=""
  env $envs TWIN_PROJECT="$proj" timeout "$I1_TIMEOUT" scripts/i1/run_ab.sh "$tag" >"$OUT/$name.log" 2>&1
  echo "ab_rc=$?" >>"$OUT/$name.log"
  docker compose -f docker/compose.yaml --env-file "$ENVFILE" --profile dev run --rm --no-deps --user "$(id -u):$(id -g)" dev \
    python3 /work/scripts/i1/summarize.py "$tag" >"$OUT/$name.verdict" 2>&1
  echo $? >"$OUT/$name.rc"
  docker compose -p "$proj" -f docker/compose.yaml --env-file "$ENVFILE" --profile test --profile lite down -v >/dev/null 2>&1
}
while IFS=$'\t' read -r name tier envs soft; do
  case "$name" in ''|\#*) continue ;; esac
  [ "$tier" = i1 ] || continue
  echo "start $name (serial)"; run_i1 "$name" "$envs" </dev/null  # docker would eat the rest of $CAMPAIGN from stdin
done < "$CAMPAIGN"

docker compose -f docker/compose.yaml --env-file "$ENVFILE" --profile dev run --rm --no-deps --user "$(id -u):$(id -g)" dev \
  python3 /work/scripts/nightly/summarize_nightly.py "/results/nightly_$STAMP" "/work/campaigns/nightly.tsv"
rc=$?
echo "summary: $OUT/summary.json (rc=$rc)"
exit $rc
