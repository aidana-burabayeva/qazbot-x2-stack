# Third-party sources and runtime dependencies (pinned)

Nothing in this directory is a binary. Everything the X2 navigation stack runs that is not built from `ws/src` is
reproduced from pinned upstream sources + patches (`../third_party.repos`, `patches/`) or pinned Debian packages
(`runtime_debs.lock`, sha256). Proven 2026-11-05 against the live production files (details: report
`.orchestrator/reports/20261105-cb-consolidation/P2-implementer.md`).

| Component | Upstream pin | Patches | Build flags (= live CMakeCache) | Live artefact replaced |
|---|---|---|---|---|
| FAST-LIO (`fast_lio`, `fastlio_mapping`) | hku-mars/FAST_LIO `a4743b0` (ROS2 branch, merge PR #381, 2025-01-15) + ikd-Tree `e2e3f4e` (its submodule pin) | `patches/fast_lio/0001-x2-portable.patch` | Release, `USE_LIVOX=OFF` | `candidates/fast_lio_portable/install` |
| Smac 2D bounds fix (`nav2_smac_planner`) | ros-navigation/navigation2 tag `1.1.18` = `1c68c21` | `patches/nav2_smac_planner/0001-smac2d-world-to-map-bounds.patch` | RelWithDebInfo | `candidates/nav2_smac_bounds/install` (`QAZBOT_SMAC_PREFIX`) |
| Nav2 1.1.18 runtime (amcl, controller, planner, RPP, smoother, lifecycle, core, navfn, smac deb, ompl, nav_2d) | snapshots.ros.org humble 2025-08-15 | – | debs | `runtime/qazbot_nav_deps` (12 debs) |
| bt_navigator, nav2_behavior_tree 1.1.18, behaviortree_cpp_v3 3.8.7 | snapshots.ros.org humble 2025-08-15 | – | debs | `runtime/nav2_bt_deps` (3 debs) |
| nav2_map_server, slam_toolbox, pointcloud_to_laserscan + Ceres/GraphicsMagick/lapack/lcms2/libexif/libwmf | snapshots.ros.org + ports.ubuntu.com jammy | – | debs | `runtime/user_pkgs` (19 debs) |

## How the live artefacts were produced (provenance)
* **FAST-LIO**: `candidates/fast_lio_portable/src/FAST_LIO` was an untracked copy (no `.git`). Search over the last 60
  upstream ROS2 commits: `a4743b0` has the smallest diff; with ikd-Tree `e2e3f4e` all other files are byte-identical.
  The remaining diff (CMake C++17 + optional Livox, `preprocess.*`, `laserMapping.cpp` = the two earlier patches
  `lidar_loopback_time_buffer.patch` + `runtime_logging.patch` and portability edits, `config/x2.yaml`) is
  `0001-x2-portable.patch`. Applying it to the pin reproduces the live source tree byte-for-byte (`diff -r`: identical).
  Live build: `colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release`
  (NOTES_loopback_fix.md, CMakeCache: Release, USE_LIVOX=OFF).
* **Smac**: `bounds.patch` against navigation2 1.1.18; `smac_planner_2d.original.cpp` == upstream tag file;
  patched file sha256 `79beb929…6f1c` == `source_sha256` in the live `qazbot_bounds_fix.json`. Live build
  RelWithDebInfo, CMAKE_BUILD_PARALLEL_LEVEL=1 (colcon events log). Note: the live `src/navigation2-1.1.18` copy is
  truncated (432 zero-byte files after the 2026-10-27 restore); the pinned upstream replaces it.
* **Runtime debs**: `runtime/qazbot_nav_deps/debs` and `runtime/user_pkgs/debs` hold the original debs; `dpkg -x` of
  them is byte-identical to the live trees (940 and 877 files). `runtime/nav2_bt_deps` had no debs: the three debs from
  snapshots.ros.org/humble/2025-08-15 extract byte-identical to all 261 live files. All 34 debs are also cached in
  `/home/agi/qazbot_archive/third_party_debs/` (not in git).

## Use
```bash
ws/third_party/fetch_runtime_debs.sh <deps-dir> [--cache /home/agi/qazbot_archive/third_party_debs]
nice -n 19 ws/third_party/build_third_party.sh --src <dir> --build-base <dir> --install-base <install> --deps <deps-dir> --jobs 2
```
`build_third_party.sh` writes `nav2_smac_planner/qazbot_bounds_fix.json` (checked by `nav_full.launch.py`) and
`fast_lio/share/fast_lio/qazbot_build.json`. Release builds (`docker/deploy/build_release.sh`) call both.

## Rebuild vs live (2026-11-05, lab rebuild on the X2, nice 19, 2 jobs)
* fastlio_mapping: same size (1 717 112 B), same dynamic symbols, same NEEDED libraries, same function symbol set,
  **identical instruction stream** (171 485 instructions, mnemonic-level); bytes differ only in addresses, build-id and
  the embedded absolute source paths.
* libnav2_smac_planner_2d.so: same dynamic symbols and NEEDED; instruction stream identical except 2 `adr`/`adrp`
  relaxations and 2 linker branch veneers (layout shift from different embedded build paths, RelWithDebInfo).
  Both contain the patch's "outside the costmap" exceptions.
Bit-identical binaries are not expected (paths, build-id); parameters are unchanged (FAST-LIO reads
`qazbot_localization/config/x2.yaml`, Smac params come from `nav_full.launch.py`).
