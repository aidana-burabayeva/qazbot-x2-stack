# P0 preservation restore drill — 2026-11-03

**RESTORE_BLOCKED. Classification: OFFLINE_VERIFIED only.**
This is a fresh, bounded restore drill on the operator PC, using existing
`/home/adn/QazBot/x2_preservation_20261103/{SHA256SUMS,inventory/files.json,final_deployment_validation.json}`.
No new robot inventory, network call, hardware access, ROS graph, replay,
installation into the host environment, model-router change or Nightly run.
Only Codex ran; Claude remains reserved for a later independently authorized
review. Time of day is not provider availability or review authorization.

## Fresh checks and evidence

[Machine evidence](verification.json) records commands, exit codes, hashes,
restored paths and individual BLOCKED checks. [Local preflight](preflight.json)
records checkout state, resource availability and external worktrees left alone.
[Pytest transcript](pytest.txt) records the focused verification result.

| Check | Fresh result / exact scope |
|---|---|
| Resource check | About 5 GiB free on workspace filesystem before drill. No large archive or raw bag copies. Git tree size 105,187,901 bytes checked before checkout. Scratch stays under this isolated attempt and is removed after each drill. |
| Four archive SHA256 values | All match the existing SHA256SUMS, including the absolute-path preserved-bundle entry. Private archives are readable in this sandbox; no permission bypass was needed. |
| Git restore | Local `git clone --no-checkout --no-hardlinks`, both bundles verified, `git fsck --full` succeeds, then detached checkout of `dd1d0af5bfb30733db33811a8cb0b3d8d632f13e`. Hardware `1212d7e…` and feedback `617999a…` commits resolve. A dangling `55ad923…` commit is informational, not corruption. |
| Current deployed selected sources | 1,974 restored `ws/src`, tools and tests files match original inventory SHA256, reproducing the scope/count of the existing final deployment validation. 73 omitted files are runtime `.lock` files, explicitly listed. Within this set, 374 files outside historical backups/results match with executable-bit parity. Nothing was copied into the working production tree. |
| Private tar | All 21,314 members streamed: 18,431 regular files, 2,529 directories, 354 symlinks. All regular sizes/SHA256/modes match inventory; 268 recorded link targets match. All 354 links recreated in isolated scratch and checked with lstat/readlink, never dereferenced on the host. Four bounded source/map samples restored from tar with exact SHA256 and modes. No unsafe member paths or unknown regular files. |
| Historical tar | All 106 members readable (91 regular files, 15 directories). Original `history.tar.zst.partial` remains incomplete: 1,142,755,328 bytes, SHA256 `b5fb2fb2459daaf442d2e0109ca2fbf019ef62d098b0168f6113bdb780ed8299`. Readable outer tar does not complete this inner backup. |
| D1 map | Restored PGM/YAML decode, 673 × 903. PGM `72ee4094…`, YAML `c86aa2aa…`; 413,052 unknown, 180,955 free, 13,712 occupied pixels. YAML trinary classification preserves all 413,052 unknown cells. No map_server execution. |
| Restored backend imports | Trust, geometry, native odom/bridge, level_bridge, coordinator and adapter import; D04 libraries and CM evaluator import. Real host ROS Python dependencies import after sourcing existing Humble setup. No node construction or SDK stubs. Adapter import checks its pure core, not its delayed SDK-dependent main. |
| LAB backend imports | Current attempt's source_state/lab/labctl/preflight import separately; they are not claimed to be restored robot files. |
| C++ projector | Restored original package configures, compiles and installs into scratch against installed real ROS Humble dependencies, one worker, `BUILD_TESTING=OFF`. No executable, launch test, DDS participant or replay is run. This verifies a PC build, not restored ARM runtime compatibility. |
| Independent D05 work | Existing 11 CM decision-emulator tests run alongside the 11 restore checks. Missing D04 recordings do not gate these tests. Real-node fail-open/stopping evidence remains unresolved. |

Git stores executable bits, not original group-write bits or uid/gid. The tar
comparison checks regular-file permission bits; the drill does not certify an
ownership-restoring full system recovery. Historical logs/test results restored
from Git are not new runtime results. Previously established defects were not
reinvestigated or changed.

## Exact partial BLOCKED findings

1. **Three D04 bags: independent copies and actual MCAP SHA256 BLOCKED.**
   Each registered path was checked in this attempt's evidence directory,
   the preservation directory and the original local control_backend evidence
   directory (plus any explicitly configured roots). Each has zero readable
   copies in this scope. The original backup report explicitly excludes bulk
   recordings. Expected hashes are not actual verified file hashes:

   | Bag | Expected bytes | Expected SHA256 |
   |---|---:|---|
   | 204153 | 3,700,720,716 | `2b7f7adf5204f0f47391095d85b8da94e80718e65b465abb0addd951ba76d4b7` |
   | 211835 | 3,698,473,010 | `6c953804ad1c27804933aca31c781d8e60367d4de775c3e79dfe245e8d4c86bf` |
   | 183015 | 9,039,573,139 | `bb90e41b65874adb14e01636e5f1cd18d75d130ba7b1d2a01eb930eadbc1546f` |

   Total 16,438,766,865 bytes per complete set. No bulk copying is justified on
   the current workspace filesystem. Absence in these locations is not a claim
   that no other offline storage copy exists. Bag 183015 has no box claim.

2. **Runtime dependency closure BLOCKED.** 81 symlink chains end outside the
   archive membership: 68 under original `/home/agi/ws_lio/{src/FAST_LIO,build/fast_lio}`,
   10 under `control_backend/runtime/user_pkgs`, and 3 under `/opt/ros/humble`.
   The complete original link paths, targets and missing terminal paths are in
   `dependencies.missing_archive_link_targets`. They include FAST-LIO executable,
   generated interfaces and configs, and missing libwmflite/liblapack targets.
   The original inventory has 86 read errors (68 FAST-LIO, 13 user_pkgs,
   2 nav_ext_ws, 3 AIMDK paths); this is retained as original evidence, not
   reclassified into 86 newly missing files. Five original errors have targets
   available after archive-local resolution; 81 chains remain unclosed.

3. **Host SDK import BLOCKED.** `aimdk_msgs` is not importable in this PC
   environment. AIMDK itself is present in the private archive (6,729 regular
   files in the existing inventory; no missing AIMDK link chains in this drill).
   This is a host build/overlay/architecture dependency, not a claim that the SDK
   source was lost. No install, replacement messages or stubs were introduced.

4. **Complete historical/system recovery BLOCKED.** The inner partial historical
   archive is incomplete. Existing REPORT.md does not image `/opt`, `/agibot` or
   firmware. Installed host ROS dependencies allow an offline PC build but do
   not prove those dependencies can be reconstructed solely from Git/backup.

[Minimal offline host follow-up](HOST_VERIFY.md) closes only the missing-copy
and inaccessible-archive checks; it does not authorize robot access. Independent
D05 source/emulator safety work may continue. Runtime evidence needs separately
authorized **L3_QUALIFICATION**; hardware facts remain **NEEDS_HARDWARE**.

## Reproduce

```bash
python3 -m pytest -q -p no:cacheprovider tests/preservation/test_restore.py tests/collision_monitor/test/test_cm_eval_lib.py
```

The tests perform a new local restore and compile, not a read of this saved PASS.
They require the original backup and currently installed dependencies. To save a
new machine report, set `QAZBOT_RESTORE_EVIDENCE` to an authorized output file.
Expected partial blockers are asserted explicitly; a missing/corrupt archive or
failed source/import/build check fails the tests. No production fix is made.
The orchestrator owns commit/hash/publication; this worker commits nothing.
