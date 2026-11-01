# AMCL initialpose — attempt 4, 2026-11-01

Isolated branch `attempt/engineering_amcl_initialpose_v3-4-5deaca`, initial HEAD
`0c652f824773cf85d70040fd1b25c08a6ecae00c`, initially clean. This is an offline
candidate pending orchestrator commit/hash/test rerun and required review.
No network, robot, Docker, MuJoCo, MC or ROS graph was contacted or started.
No agent commit was made, as required by this attempt's task instruction.

## Saved episodes and preserved FAIL

`saved_episodes.json` contains five exact archived operator initialposes: four
from `t5live1_observer_20261028.jsonl.gz`, one from
`i1_deploy_2d240dbd/passive_observer.jsonl.gz`. Original compressed files remain
unchanged and are bound by SHA256 in `manifest.json`.

Re-evaluating their original localization logs preserves the historical strict
warning-free criterion **FAIL**: five initialpose future-TF warnings, missing
coverage of 8.475–34.303 ms at the requested lookup time. This is a new offline
check of historical records, not a new physical or simulation run. The logs also
show AMCL setting all five poses. They do not prove why TF coverage was late.
Post-seed AMCL XY differs from the operator input by 0.0056–0.0547 m; an output
pose cannot stand in for the original operator seed. Neither is ground truth.

The available observer datasets lack complete scan/odom input streams. Original
`First_RViz_goal_C3_1205` and `RViz_goal2_1216` full bags are absent from this
attempt. The lost `First_RViz_goal_1135` tail remains irretrievable. No attempt
was made to rebuild missing measurements or obtain files from another worktree.

## Reproduction, causal evidence and minimal fix

Both existing replay entrypoints ignored the recorded initialpose topic,
constructed one seed from recorded AMCL map→odom output plus odom, replaced its
covariance and stamped/published it before playback. Thus operator input and
reseed epochs disappeared. Separately, clock alignment to the first source
header removed its recorded input latency.

Before any source fix, actual original reader statements were executed against
temporary SQLite fixtures containing the exact saved initialposes. Scan/odom
and transport encoding are explicitly synthetic scaffolding, using a
message-shaped JSON decoder rather than CDR. All **14 tests failed**
(`before_fix.txt`), including 0/4 and 0/1 seeds read, output dependence, missing
seed acceptance and lost truncation semantics. Original reader blocks are frozen
in `baseline_readers.json`, with original file hashes and Git provenance.

Before changing clock alignment, two additional tests executed the actual
playback loops with a declared 300 ms first-input delay: both failed because
the old formula erased exactly 300 ms (`source_age_before_fix.txt`). The original
alignment statements are retained with the original readers.

The shared ROS-independent reader now retains every recorded initialpose with
its pose, covariance and header, filters replay TF to odom→base_footprint,
and refuses selected windows without an operator seed before ROS initialization.
Both entrypoints dispatch the seed at its recorded relative receipt time and
shift nonzero source stamps by one receive-clock offset. Zero initialpose stamp
keeps its ROS “now” semantics. Recorded AMCL output cannot construct a seed.
Production localization, AMCL parameters, Trust, T5, safety limits and tolerances
are unchanged.

## Original FAIL retest and regressions

The verification command in the result JSON runs **107 passed, 2 deselected**;
the two excluded cases require ROS node imports. Two existing NumPy warnings
in NaN/gap safety tests remain (`after_fix.txt`). No skipped tests in this command.

Tests re-execute both frozen original readers and reproduce their lost-seed
FAIL, then verify the fixed readers and actual dispatch loops. They also repeat
the original source-age error; cover both archived datasets, full covariance,
repeated seeds, no-seed refusal, output independence, truncation, TF filtering,
timestamp shifting and invalid/zero-stamp payloads. Recorded initialpose and AMCL
messages are additionally replayed through the actual Trust callback bodies.
That callback test bypasses receipt-freshness validation because observer receipt
time is not the original node receipt; it makes no READY/MOTION_OK claim.
Existing Trust tests cover seed refresh, source rejection, gaps, reseed history
and motion admission separately. Synthetic Trust tests are safety regressions,
never AMCL accuracy evidence. No truth localization was used as AMCL evidence.

An independent Codex read-only review reran the then-current focused suite
(21 PASS), matched both frozen readers to original Git HEAD, and found no blocking
defect. Claude is unavailable through this session's tools; this is not the
required cross-provider review. Later additions preserve archived warning FAIL
and replay callbacks, and repeat the original clock formula in regressions.

## Remaining qualification

**Request L3_QUALIFICATION** on the orchestrator's committed candidate SHA before
any authorized runtime experiment. First obtain a complete approved saved
scan/odom/initialpose/map recording to replay with actual CDR/AMCL and measure
the original warning/failure, changed behavior, and regressions. That replay,
DDS delivery, AMCL stochastic behavior and original runtime root cause remain
**UNVERIFIED**. Physical timing/calibration and robot behavior remain
**NEEDS_HARDWARE**. No live experiment is authorized by this report.

Known independent follow-up: both legacy replay summaries still label transform
translation changes as T5. Those figures are excluded from this diagnosis; T5
qualification requires robot displacement at the same odom pose. The reader fix
does not close that separate metric defect. Pacing still uses inherited float
seconds with submicrosecond quantization. Nothing here is **SIM_VERIFIED**.
