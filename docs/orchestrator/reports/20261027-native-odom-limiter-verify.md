# Verification: step_cmd.py `--limit-source native_base` (20261027-native-odom-limiter)

Reviewer: independent verifier, 2026-10-27, on robot `agi`. The robot did not move. No MC sources were registered, nothing was armed, no nonzero velocity went to any non-isolated graph, and no live adapter was started. Everything ran offline or in isolated domains 87/88 with ROS_LOCALHOST_ONLY=1. No passive domain-0 subscriptions were made. No project files were edited; scratch roots and scripts are in
`/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/{verify,verify_cur_root,verify_base_root}`.

## VERDICT: FAIL (one fix needed: non-finite input silently disables the limiter). Every other check passed.

## Per-check results

| # | Check | Result | Evidence |
|---|---|---|---|
| 1 | Transform direction/composition | PASS | `lookup_transform_core('base_link','lidar_imu_chest_front',t)` returns T_base_imu (target=base_link). Code (step_cmd.py:190-197): q_ob = q_oi*conj(q_bi), p_ob = p_oi - R_ob*t_bi. This equals T_odom_imu*T_base_imu^-1. Independent numpy matrix check (verify/indep_math.py): real trial06 static chain, random non-identity waist joints with translations, roll/pitch up to 10 deg, 400 samples. Max xy error 3.6e-12 m, max yaw error 4.4e-11 rad. The reverse order inv(Tbi)*Toi gives 4.6 m error, so the check is sensitive to composition order. qmul is the correct Hamilton product (x,y,z,w). |
| 2 | Yaw wrap | PASS | Unwrap per evaluated sample (step_cmd.py:198-200). The independent sweep went from -3.0 rad through ±pi several times to 20.94 rad with error <1e-10, and displacement() reports 1371 deg without wrapping. Yaw = atan2(R10,R00) of the full base orientation, which is the ZYX yaw and is correct under tilt. |
| 3 | No extrapolation | PASS | Only odom samples with stamp <= min(latest stamp of each of the 3 dynamic edges) are evaluated (step_cmd.py:181-188). Float-to-ns rounding is monotonic, so the gate and tf2 agree. Tested: odom newer than TF waits in pending, and the pose keeps the older stamp. If one edge stalls, tf_stamp = min and the result is native_tf_stale. |
| 4 | Staleness >0.2 s -> zeros + motion_adapter/stop in every phase | PASS | native fault is checked inside Tool.fault() (step_cmd.py:383-387). That covers call() during the arm service, the post-arm trust wait, the arm-report wait, every step tick before publish, and post-pause. Preconditions refuse with exit 4 before any arm. Reran: step-phase odom cut and TF cut (implementer tests) both gave exit 2, then zeros, then stop with no resume. Extra verifier checks through the real Tool.run(): (a) arm phase with the arm service held pending for 1 s and data cut: exit 2 native_odom_stale, 0 nonzero rows, 21 end_zeros, last call motion_adapter/stop. (b) Data cut at the start of the pause: exit 2, step 1 never started, last call stop. Expected behaviour seen in (a'): if data stops <0.2 s before the step, up to about 0.15 s of nonzero commands still go out before the abort. That is how the 0.2 s threshold works, not a bug. |
| 5 | Default lio path behaves the same | PASS (output not byte-identical) | Diff review: every new branch is gated on `self.native is not None`. The `elif self.limits and ... ODOM_MAX_AGE` checks are unchanged. limit_displacement() calls displacement(). CSV header is unchanged. Model runs of the current file and the backup: C1-C5 CSV headers identical, summary keys identical, same exit codes and abort reasons. Only difference: `summary.json` `args` gains 3 keys (limit_source, native_odom_topic, native_max_age). This does not change behaviour. |
| 6 | Tests exercise the real code | PASS (with caveats) | NativeLimiterReplay runs the real NativeBase methods from step_cmd. Only the 20 Hz tick driver is reimplemented in the test (test_native_limiter.py:72-91). ToolReplay runs the real Tool.run(), callbacks, fault() and close(). It replaces spin(), status inputs, graph queries and Trigger clients. Caveats: the adapter status is always motion_armed=True and futures complete immediately, so the arm-phase abort was not covered (verifier covered it, row 4). There is no test for non-finite inputs. The composition unit test uses step_cmd's own qmul/qrot to build its inputs; the verifier's numpy check covers that independently. |
| 7 | New tests | PASS 17/17 | `source ws/src/navigation/env.sh && python3 -m unittest -v tests/s3/test_native_limiter.py`: Ran 17 in 36.3 s, OK. Observed: native dist does not trip (final 0.0058 m, 67.4 deg). Yaw 30 deg trips 3.697 s after the recorded onset. Odom cut gives native_odom_stale and TF cut gives native_tf_stale, both exit 2. Default LIO trips at `dist 0.301 m` 5.824 s after onset. The verifier subclass reran the 5 ToolReplay tests again: OK. |
| 8 | tests/motion_adapter | PASS 43/43 | Run from tests/motion_adapter: `python3 -m unittest test_adapter_core test_dry_run_shadow test_early_takeover test_new_faults`: Ran 43, OK. |
| 9 | 35/38 model failures existed before this change | CONFIRMED | Two separate scratch roots (copies of harness + step_cmd, symlinked ws/tools/runtime/nav_e2e; nothing written to the repo). Current file: RESULT 35/38. Backup `.bak_20261027_pre_nativebase` (sha256 2767cd3d...): RESULT 35/38. The same 3 fail in both: c1_adapter_zero_burst_then_silence_at_end, c2_adapter_silent_after_limit, c5_adapter_zero_then_silence. The tool side is correct in both: the first zero row comes about 50 ms after the last step row, and c5 has 1 step row then end_zeros. The probable cause is in the harness/adapter, not the tool: the adapter re-publishes the last command at 50 Hz (x2_motion_adapter.py publish_hz=50, cmd_timeout 0.15 s) until a zero arrives, and the harness timestamps callbacks when processed, against a t_off+0.01 window. This was not changed here, but the c5 (+0.2 s) failure in a safety check should be looked at separately. |

## Defects

### D1 (needs fixing, safety): non-finite leg odometry silently disables the limiter for the rest of the run
- step_cmd.py:176-179 (`add_odom`), 194-200 (`update`), 206-216 (`fault`). Nothing checks whether the pose or quaternion is finite.
- Evidence (verify/edge_cases.py, real NativeBase):
  - One sample with NaN qw, then good samples: displacement = (0.0, **nan**) and fault() = '' (fresh). The unwrapped yaw stays NaN for good, because `prev + atan2(...)` with prev=NaN is NaN. `abs(nan) >= limit` is False, so **the yaw limit never trips, and no fault is raised.**
  - NaN x: dist is NaN for that tick (limit is skipped) and recovers on the next sample.
  - Zero quaternion: accepted, gives a false 10 deg yaw / 2.19 m jump. This is on the safe side because it trips early.
  - NaN TF: tf2 rejects it (TF_NAN_INPUT), but `tf_edge` still advances (step_cmd.py:172). The lookup then raises an exception, which leads to native_base_stale after 0.2 s. That is safe, but the cause is reported as staleness.
- In the trial06 bag, 0 of 3598 leg-odom samples were non-finite, so this is unlikely. But this is the stop limiter being reviewed, and the failure is silent.
- Required fix: reject non-finite odom (and non-unit or zero quaternions) in `add_odom` / `_on_native_odom` and record an error. Also make `fault()` return e.g. `native_non_finite` if the pose or start pose is non-finite. Optionally skip the tf_edge stamp update when the transform is non-finite. Add a unit test for this.
- The LIO path has the same latent problem (step_cmd.py:279-289, `_on_odom` unwrap). It existed before this change and is not counted against this task, but it should be noted.

### Non-blocking observations
- O1: `_on_native_odom` (step_cmd.py:291) checks child_frame_id but not header.frame_id (`leg_odom`). In the bag, all 3598 samples are ('leg_odom','lidar_imu_chest_front').
- O2: A leg-odom reset or jump with continuous stamps (after the reboot the origin was reset) is not detected. A jump usually trips the limit early (safe side), but it could in theory hide real motion.
- O3: Stamps are compared to time.time() (unix). This relies on vendor header stamps being PTP/unix-synchronised (implementer reports age p50 6 ms). I did not re-check this, because domain 0 is off-limits for this review. Do a passive check before live use, as the implementer's next step 3 says.
- O4: An exception in a native callback, raised inside spin_once, would leave run() without zeros or stop. This is the same pattern as before the change. The adapter's cmd_timeout of 0.15 s still stops output. tf2_py logged errors but did not raise for NaN, self-parent or empty frame_id (tested).
- O5: Native and LIO final yaw on trial06 differ by about 5.8 deg (67.4 vs 61.6). Neither is ground truth. Hit times for the yaw limit were checked only against the earlier offline audit, which used the same composition method on the same data. That shows consistency, not accuracy.
- O6: The run uses a single-threaded executor with leg odom at about 168 Hz plus /tf at about 46 Hz in Python callbacks. This was not load-tested on the live graph.

## About the trial06 "truth"
Trial06 was judged by the operator watching the robot (mostly in-place rotation, 20261023 report lines 141/156). The 0.03 m pelvis translation and 67.4 deg native yaw come from the vendor's own leg odometry plus TF. They match the earlier audit and the operator's account, but they are not independent ground truth. LIO's 1.68 m was rejected because it contradicted that observation.

## Commands run
1. `cd /home/agi/control_backend; bash -c 'source ws/src/navigation/env.sh && python3 -m unittest -v tests/s3/test_native_limiter.py'`: 17/17 OK (36.3 s).
2. `cd tests/motion_adapter; source env.sh + aimdk_env.sh; python3 -m unittest test_adapter_core test_dry_run_shadow test_early_takeover test_new_faults`: 43/43 OK.
3. `verify_cur_root: bash tests/s3/run_model_s3.sh verify_cur_01`: 35/38 (domain 87).
4. `verify_base_root (backup step_cmd): bash tests/s3/run_model_s3.sh verify_base_01`: 35/38, same 3 failures.
5. `verify/indep_math.py`: composition and unwrap verified (errors around 1e-11).
6. `verify/edge_cases.py`: D1 reproduced; no extrapolation; one-edge stall gives tf_stale.
7. `verify/arm_pause_abort.py`, `verify/arm_abort2.py` (domain 88): pause and arm-pending aborts give zeros + stop. The arm_pause_abort "arm" case failed my own wrong assumption (data cut <0.2 s before step start), which is expected behaviour.
8. Read-only sqlite check of the trial06 bag: chain edges have one parent each, stamps are monotonic, leg odom is all finite.
9. Diff: `diff -u step_cmd.py.bak_20261027_pre_nativebase step_cmd.py` reviewed in full.

---

## Cycle 2: re-check of the D1 fix (non-finite inputs). VERDICT: PASS

Files checked: step_cmd.py sha256 548fdf48… (mtime 16:08:52), test_native_limiter.py sha256 8dc33940… (16:09:30). Backup: `step_cmd.py.bak_20261027_pre_nonfinite`. The diff (about 44 changed lines) was read in full. No motion. Only isolated domain 88/localhost was used; domain 0 and the DDS XML files were not touched.

| Check | Result | Evidence |
|---|---|---|
| Non-finite / zero / non-unit quaternion rejected | PASS | `finite_pose` step_cmd.py:138-142 requires a finite stamp and t7, and \|q\| within 1±0.01. It is applied in add_static :176-181 (latches only for chain links pelvis/lidar_chest_front/lidar_imu_chest_front; ignores others), add_tf :183-192, add_odom :194-199, and on the composed pose in update :219-221. Re-probes: NaN x, NaN qw, zero q, \|q\|=1.02, NaN stamp all give `native_non_finite:odom`. NaN TF gives `native_non_finite:tf …`. NaN chain static gives `native_non_finite:tf_static lidar_chest_front->lidar_imu_chest_front`. Off-chain zero q is ignored. |
| Rejected TF does not move the edge stamp | PASS | step_cmd.py:186-188 returns before the `tf_edge` update. Covered by test_nan_tf_rejected_and_edge_stamp_not_advanced (asserts tf_edge is unchanged). |
| Latch is permanent and causes zero + stop in every phase | PASS | `fault()` :230-238 checks `non_finite` first, then start and pose finiteness, then staleness. The latch is never cleared. Tool.fault :420 passes native_start. Tool.fault is the single check used in the arm call, trust wait, arm wait, every step tick and post-pause (unchanged since cycle 1, verified there through the real run()). Preconditions :509 refuses with exit 4. Real Tool.run() with NaN qw mid-step: exit 2 `native_non_finite:odom`, last nonzero 0.959 s after the recorded onset with the NaN at 1.0 s, then ≥1 s of zeros, then motion_adapter/stop. Arm-pending test: exit 2, 0 step rows, all rows zero, last call is stop. |
| LIO default unchanged except dropping NaN | PASS | :312-316 drops only when x, y or yaw is non-finite, before the unwrap, and does not refresh odom_at (persistent NaN still gives odom_stale). :642-643 adds the summary key only when the count is non-zero. Tool test lio_nan: exit 3 `dist 0.301 m` (same as without NaN), dropped=1, no nan in CSV. lio_default: same as cycle 1 (0.301 m, 5.82 s). |
| New tests use the real code | PASS | NaN, zero-q and TF tests call the real NativeBase.add_*/update/fault through the existing replay. The 3 new ToolReplay tests run the real Tool.run(), callbacks and close(). Only spin, status inputs, graph queries and Trigger clients are mocked; the arm_delay future is a mock. |

Tests rerun (after the 16:08/16:09 edits):
- `tests/s3/test_native_limiter.py`: **25/25 OK** (52.3 s, domain 88 localhost).
- `tests/motion_adapter` (4 modules, run from that directory): **43/43 OK**.
- Model S3 was not rerun this cycle. The implementer reports 35/38 with the same 3 failures that existed before; the default-path change is limited to dropping NaN samples.

Remaining non-blocking observation (new):
- O7: step_cmd.py:469. If the latch fires before the first pose exists, `preconditions()` keeps logging `waiting for: native_base` and never shows the non_finite reason. This is safe (no arm, no motion), but the operator has to Ctrl-C. Suggestion: include `self.native.non_finite` in the wait log, or refuse.

O1–O6 from cycle 1 are still open and do not block. The c5 harness failure (+0.2 s) also still deserves a separate look.
