# docker/ — lab environment, release/rollback scripts, cleanup tooling

| Path | What |
|---|---|
| `Dockerfile`, `compose.yaml`, `lab/` | LAB build / test / offline replay environment (ROS 2 Humble) |
| `deploy/` | SHA-pinned releases on the robot host: `build_release.sh`, `deploy.sh`, `rollback.sh`, `status.sh` |
| `tools/` | task debris registry and cleanup: `task_register.sh`, `task_cleanup.sh`, `cleanup_candidates.sh` (read-only) |
| `twin/` | digital-twin container scripts (official AimDK MuJoCo sim + MC, x86); moved from `tools/twin/docker` |

## Where things run: production on the host, lab in a container
AgiBot documentation (AimDK 1.1.0 local docs, `~/aimdk/docs/cn/_build/html/zh-cn/dev`):
* `quick_start/prerequisites.html` 4.2 and `container_dev.html` 4.4: on the development compute unit, **from AimDK
  v1.2.0 secondary-development programs must be built and run in a container**; **v1.1.0 is the transition version where
  both host and container mode are allowed**. This X2 runs platform v1.1.5 / aimdk_msgs v1.1.0.4 (docs/X2_AIMDK_VERSIONS.md).
* `changelog.html` v1.0: user programs run in the agi rootfs sandbox; "暂不支持 docker" at v1.0; `systemctl --user` only.
* `common/warnings.html` "二次开发边界与声明": never use PC1 (10.0.1.40, MC) as dev/run environment; do not change vendor
  DDS/system configuration; integrate third-party libraries via the container mode; start at most 2 ROS nodes per second
  while the robot stands or walks.

Decision (P1S 6, operator-approved): **production stays on the host** (dev unit 10.0.1.41) for now: it is what runs and
is qualified (v1.1.x permits it), the Docker daemon is not running on the robot, and the stack needs the vendor host
environment (`~/.aima/env/bashrc`, `/agibot/software/common` aimdk_msgs, host DDS). **Risk / next step:** before an
AimDK v1.2.0 upgrade the production stack must move into the official container (4.4 Dockerfile/run.sh); the release
layout (`<prefix>/releases/<sha>/{install,deps}`) is relocatable and can be bind-mounted into that container unchanged.

## LAB isolation (this robot is a temporary lab; the same files work on a lab PC)
`compose.yaml` services `build`, `test`, `replay` (and `fetch`, the only one with network, no ROS):
* **no access to MC / robot graph**: `network_mode: none` (no DDS discovery, no `/aima/*` traffic at all), `ipc: private`
  (no FastDDS shared-memory segments shared with host processes), `ROS_DOMAIN_ID=88`, `ROS_LOCALHOST_ONLY=1`; the
  bringup refuses `mode:=lab` on domain 0 or a non-local graph, and lab mode runs the motion adapter dry
  (`live:=false`: no `/aima/*` publisher, no SetMcInputSource client) or not at all (`start_adapter:=false` in replay);
* **no devices**, `cap_drop: ALL`, `no-new-privileges`, unprivileged user, sources mounted **read-only** (outputs in the
  `lab-out` volume, so a container cannot dirty the repository);
* **CPU/memory**: `cpuset` `QAZBOT_LAB_CPUSET` (default `0-3`; FAST-LIO is pinned to 5,6 on the X2), `cpus`
  `QAZBOT_LAB_CPUS` (default 2.0), `mem_limit` `QAZBOT_LAB_MEM` (default 6g), `pids_limit`; builds use `nice -n 19` and
  2 jobs. On a lab PC override these in `docker/.env` — no architecture-specific step (ros:humble is multi-arch).
* Host-side lab runs without Docker (as done for P2 verification) use the same guards: `ROS_DOMAIN_ID=88`,
  `ROS_LOCALHOST_ONLY=1`, `nice -n 19`, `--parallel-workers 2`, build/install outside the repository.

**Do not start the Docker daemon on the production robot without operator permission** (it changes host networking
/iptables). The images were therefore not built on the X2 during P2 (the compose plugin is not installed there either); `compose.yaml` was checked as YAML only. Build/run them on a lab PC first.

## Lab usage
```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm fetch     # network: pinned third-party sources + debs into lab-out
docker compose -f docker/compose.yaml run --rm build
docker compose -f docker/compose.yaml run --rm test
QAZBOT_BAG=/abs/path/to/bag docker compose -f docker/compose.yaml run --rm replay
```
The twin (`twin/`) has its own image (`twin/Dockerfile`, x86, network none, domain 87) and needs the vendor sim/MC
runtime and the research root of the lab lineage (`lab/stable`); not runnable from this tree alone (unchanged).

## Releases (robot host)
See the root README "LAB → PRODUCTION" and `docs/RELEASE.md`. Prefix default `$QAZBOT_PROD_PREFIX` or `$HOME/prod`.
```bash
docker/deploy/build_release.sh --sha <sha> [--deb-cache /home/agi/qazbot_archive/third_party_debs]
docker/deploy/deploy.sh --sha <sha>      # switches current/previous only
docker/deploy/status.sh                  # selected and running SHA
docker/deploy/rollback.sh                # current <-> previous
```

## Cleanup tooling
```bash
docker/tools/task_register.sh <task_id> <path>...   # every temp path a task creates
docker/tools/task_cleanup.sh <task_id> [--dry-run]  # deletes only registered paths after checks
docker/tools/cleanup_candidates.sh [root...]        # read-only report
```
`task_cleanup.sh` deletes a registered path only if it is inside `QAZBOT_CLEANUP_ROOTS`, not protected
(`QAZBOT_PROTECTED`, release `current`/`previous`, legacy production, worktrees, aimdk, archive), not open/used by
any process (lsof, /proc cwd/exe), and holds no unique git work (uncommitted/untracked, stash, unpushed commits).
