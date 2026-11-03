# 20261103-controller-compare — RPP vs DWB vs MPPI (offline, Nav2 Humble 1.1.18 binaries, domain 78)
STATUS: DONE — main matrix 24/24 runs + 12 extra runs (D-04 candidate C clearing; constant-gain MC variant), seed 1, all completed.
No hardware, no deploy, nothing installed system-wide; nice 19 / ionice idle; one sim at a time; ROS_DOMAIN_ID=78 localhost unicast.

## 1. Setup (facts)
- Binaries: controller_server/RPP/velocity_smoother/lifecycle = runtime/qazbot_nav_deps (1.1.18); MPPI = runtime/nav_ext_ws
  (ros-humble-nav2-mppi-controller 1.1.18-1jammy.20250729.010514); DWB NOT on host -> downloaded only (same snapshot 2025-08-15,
  sha256 = index) and dpkg-deb -x into scratch: dwb_core 1.1.18-1jammy.20250729.010621, dwb_plugins .20250729.011434,
  dwb_critics .20250730.130109, dwb_msgs .20250719.103844, costmap_queue .20250730.115147 (tests/controllers/fetch_dwb.sh).
- RMW rmw_fastrtps_dynamic_cpp for ALL runs (dwb_msgs/nav_2d_msgs fastrtps typesupport need libfastcdr.so.1; host has 2.x).
- Chain: FollowPath -> controller_server (+local costmap = LIVE footprint/padding 0.01/inflation 0.5,3.0/ObstacleLayer 2 sources)
  -> /cmd_vel_nav -> velocity_smoother (LIVE, OPEN_LOOP 0.3/0.6, acc 0.4/1.0, dec -0.6/-1.5) -> /cmd_vel -> sim.
- Path: first /qazbot_d1_map/nav2/plan of LIVE-2 nav_bag_183029 (5.31 m, map frame, ~100 deg left curve then straight), identical
  start pose / seed for all; world = D1 map walls (d1_qazbot_20261029) + 0.4x0.4 m boxes.
- Sensor (D-04b): marking FOV ±54 deg, 0.45..3.0 m; clearing = level_bridge.clearing_endpoints (FOV -46..+44.5, ray ends at return,
  no ray for a return inside the 0.45 m blind range); z=0 points (Humble per-source max_obstacle_height default 0.0).
- Robot: ideal (cmd = motion) and mc_hyp = HYPOTHESIS from D-05: dead time 0.4 s, v = 0.34+0.53*cmd for |cmd|>=0.02
  (=1.66x at 0.30; offset/minimum-gait-speed shape — report says "NOT a model"), tau_v 0.25 s, wz 1:1, tau_w 0.15; cmd >0.15 s old -> 0.
  Native-odom under-read (D-06) and low obstacles (D-07) NOT modelled.
- Params verified against what the installed plugins actually DECLARE (ros2 param list after configure): RPP 20/20 = live;
  DWB: `stateful` is not a 1.1.18 DWB param -> removed; MPPI: `ObstaclesCritic.inflation_layer_name` absent in 1.1.18 -> removed,
  1.1.18 ObstaclesCritic has its own inflation_radius/cost_scaling_factor -> set to live 0.5/3.0. MPPI 1.1.18 has NO accel params
  (ax_max is Iron+): accel limiting = live velocity_smoother (MC accel unmeasured; DWB acc_lim = smoother values, stated).
- DWB: vx 0..0.3, wz ±0.6, 20x1x20 samples, sim_time 1.7, ObstacleFootprint critic. MPPI: DiffDrive, vx 0..0.3, wz ±0.6,
  batch 1000, 56 steps x 0.05 s, ObstaclesCritic consider_footprint true, default Humble critic set/weights.
- FollowPath only (no BT): "replans/recoveries" = ABORTED result where the live BT would replan / ClearCostmap / Wait.

## 2. Results (all runs, seed 1; raw JSON incl. cmd/pose series in tests/controllers/results/)
Columns: min clr = min footprint clearance to walls+boxes / to the box only (negative = contact/penetration); P23 = fractions of
/cmd_vel ticks the adapter would treat as pure rotation (|vx|<0.02,|wz|>0.15) / low vx (0.02..0.15) / pure-rot mode switches;
period = /cmd_vel_nav inter-arrival (20 Hz loop; stretch = compute overrun); CPU = controller_server CPU % of one core / CPU ms
per control cycle (incl. costmap thread) / RSS. log = RPP "collision ahead" / progress-checker fail / MPPI optimizer fail / missed rate.

| model | scenario | ctrl | result | t[s] | prog[m] | dev mean/max | min clr (all/box) | stop/resume | wz flips/s | vx jerk rms | P23 pure-rot / low-vx / switches | period p50/p95/p99 ms | CPU % / ms per cycle / RSS MB | log: coll / fail-prog / mppi-fail / missed-rate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ideal | nominal | rpp | SUCCEEDED | 18.63 | 4.99 | 0.014/0.046 | 0.493/- | - | 0.108 | 5.16 | 0.035/0.032/1 | 50.0/50.5/50.8 | 4.2/2.09/69.0 | 0/0/0/0 |
| ideal | nominal | dwb | SUCCEEDED | 19.38 | 5.0 | 0.015/0.042 | 0.492/- | - | 0.104 | 7.47 | 0.057/0.034/2 | 50.0/51.2/52.4 | 22.3/11.22/72.0 | 0/0/0/0 |
| ideal | nominal | mppi | SUCCEEDED | 18.38 | 5.15 | 0.022/0.056 | 0.492/- | - | 0.11 | 2.6 | 0.0/0.038/0 | 50.0/50.8/51.5 | 17.6/8.83/77.0 | 0/0/0/0 |
| ideal | static_block | rpp | ABORTED | 18.37 | 2.25 | 0.019/0.047 | 0.376/0.376 | stopped_before=True passed=False | 0.0 | 8.92 | 0.0/0.03/0 | 50.0/50.5/50.8 | 4.4/2.2/69.0 | 200/0/0/0 |
| ideal | static_block | dwb | ABORTED | 26.93 | 2.56 | 0.006/0.039 | 0.033/0.033 | stopped_before=True passed=False | 0.0 | 5.15 | 0.022/0.132/2 | 50.0/50.7/51.3 | 22.1/11.06/72.3 | 0/1/0/0 |
| ideal | static_block | mppi | ABORTED | 31.71 | 2.5 | 0.016/0.057 | 0.113/0.113 | stopped_before=True passed=False | 0.032 | 1.15 | 0.0/0.315/0 | 50.1/51.4/54.3 | 57.9/28.99/72.9 | 0/1/0/0 |
| ideal | transient | rpp | SUCCEEDED | 23.16 | 4.99 | 0.013/0.046 | 0.493/0.297 | stop=True resume=0.36s | 0.087 | 18.32 | 0.037/0.052/1 | 50.0/50.4/50.9 | 4.2/2.1/67.0 | 85/0/0/0 |
| ideal | transient | dwb | SUCCEEDED | 22.9 | 5.0 | 0.018/0.043 | 0.492/0.083 | stop=False resume=0.32s | 0.088 | 9.98 | 0.048/0.177/2 | 50.0/50.7/51.3 | 22.6/11.38/72.1 | 0/0/0/0 |
| ideal | transient | mppi | SUCCEEDED | 22.4 | 5.13 | 0.031/0.068 | 0.492/0.305 | stop=False resume=0.36s | 0.179 | 2.68 | 0.0/0.23/0 | 50.0/51.5/54.0 | 24.6/12.25/75.1 | 0/0/0/0 |
| ideal | stale_cell | rpp | ABORTED | 10.08 | 0.0 | 0.0/0.0 | 0.492/- | - | 0.0 | 0.0 | 0.0/0.0/0 | 50.0/50.9/51.4 | 4.0/1.99/68.9 | 201/0/0/0 |
| ideal | stale_cell | dwb | ABORTED | 15.1 | 0.02 | 0.001/0.001 | 0.492/- | - | 0.0 | 0.52 | 0.0/0.0/0 | 50.0/50.6/51.6 | 20.7/10.37/73.8 | 0/1/0/0 |
| ideal | stale_cell | mppi | ABORTED | 15.11 | 0.01 | 0.001/0.001 | 0.492/- | - | 0.134 | 0.3 | 0.0/0.0/0 | 50.0/50.5/50.8 | 67.8/34.05/74.9 | 0/1/0/0 |
| mc_hyp | nominal | rpp | SUCCEEDED | 14.85 | 5.31 | 0.064/0.172 | 0.492/- | - | 0.407 | 11.55 | 0.199/0.111/5 | 50.0/50.4/50.9 | 3.4/1.72/67.0 | 0/0/0/0 |
| mc_hyp | nominal | dwb | SUCCEEDED | 13.59 | 5.31 | 0.052/0.182 | 0.492/- | - | 0.298 | 11.68 | 0.132/0.048/2 | 50.0/50.3/50.7 | 20.6/10.33/74.1 | 0/0/0/0 |
| mc_hyp | nominal | mppi | SUCCEEDED | 13.09 | 5.31 | 0.074/0.29 | 0.425/- | - | 0.309 | 0.65 | 0.065/0.053/2 | 50.0/50.4/50.8 | 17.0/8.54/77.1 | 0/0/0/0 |
| mc_hyp | static_block | rpp | SUCCEEDED | 15.11 | 5.31 | 0.062/0.182 | -0.236/-0.236 | stopped_before=False passed=True | 0.267 | 15.33 | 0.156/0.109/3 | 50.0/50.3/50.4 | 3.6/1.79/71.0 | 25/0/0/0 |
| mc_hyp | static_block | dwb | SUCCEEDED | 14.1 | 5.31 | 0.048/0.181 | -0.236/-0.236 | stopped_before=False passed=True | 0.287 | 14.61 | 0.124/0.103/2 | 50.0/53.0/53.5 | 20.6/10.36/70.2 | 0/0/0/0 |
| mc_hyp | static_block | mppi | ABORTED | 7.56 | 2.58 | 0.084/0.265 | 0.022/0.022 | stopped_before=True passed=False | 0.4 | 0.99 | 0.073/0.119/8 | 50.1/51.6/54.1 | 26.6/13.31/77.5 | 0/0/1/0 |
| mc_hyp | transient | rpp | ABORTED | 16.12 | 2.12 | 0.13/0.18 | 0.492/-0.095 | stop=True resume=Nones | 0.125 | 8.21 | 0.084/0.068/4 | 50.0/50.4/50.5 | 3.8/1.93/71.0 | 200/0/0/0 |
| mc_hyp | transient | dwb | SUCCEEDED | 14.61 | 5.31 | 0.046/0.182 | 0.492/-0.236 | stop=False resume=0.08s | 0.278 | 14.51 | 0.127/0.096/4 | 50.0/50.6/51.1 | 21.1/10.66/74.2 | 0/0/0/0 |
| mc_hyp | transient | mppi | ABORTED | 5.81 | 2.03 | 0.104/0.264 | 0.002/0.002 | stop=False resume=Nones | 0.0 | 1.25 | 0.0/0.19/0 | 50.0/52.6/55.1 | 25.8/13.27/79.5 | 0/0/1/0 |
| mc_hyp | stale_cell | rpp | ABORTED | 10.08 | 0.0 | 0.0/0.0 | 0.492/- | - | 0.0 | 0.0 | 0.0/0.0/0 | 50.0/50.4/50.5 | 4.1/2.04/69.0 | 201/0/0/0 |
| mc_hyp | stale_cell | dwb | ABORTED | 15.1 | 0.0 | 0.0/0.0 | 0.492/- | - | 0.0 | 0.37 | 0.0/0.0/0 | 50.0/50.3/50.4 | 22.2/11.13/71.8 | 0/1/0/0 |
| mc_hyp | stale_cell | mppi | ABORTED | 15.11 | 0.0 | 0.0/0.0 | 0.492/- | - | 0.267 | 0.76 | 0.0/0.0/0 | 50.0/50.8/52.0 | 69.0/34.62/74.9 | 0/1/0/0 |

## 3. Condensed comparison
| criterion | RPP (live) | DWB | MPPI |
|---|---|---|---|
| nominal, ideal | OK 18.6 s, dev 1.4/4.6 cm | OK 19.4 s, dev 1.5/4.2 cm | OK 18.4 s, dev 2.2/5.6 cm, goal 0.17 m (others 0.31) |
| nominal, mc_hyp | OK 14.9 s, dev max 17 cm, wz flips 0.41/s, 5 pure-rot switches | OK 13.6 s, dev max 18 cm | OK 13.1 s, dev max 29 cm, min wall clr 0.43 (others 0.49) |
| real obstacle, ideal | stops 0.38 m before, abort 10 s (collision ahead) | stops 0.03 m before (!), abort 15 s (progress) | stops 0.11 m before, abort ~15 s (progress) |
| real obstacle, mc_hyp | CONTACT, then drives through | CONTACT, drives through | stops 0.02 m, optimizer fail abort |
| transient, ideal (stop/resume) | full stop, resume 0.36 s, clr 0.30 | slows to 0.016 m/s, clr 0.08, resume 0.32 s | slows to 0.02 m/s, clr 0.31, resume 0.36 s |
| transient, mc_hyp | CONTACT (-0.10), abort | drives THROUGH (-0.24), "succeeds" | 0.002 m clr, optimizer fail abort |
| stale blind cell (D-04b geometry) | blocked, abort 10 s (collision ahead x200) | blocked (no motion), abort 15 s | blocked (small sub-deadband wz only), abort 15 s |
| detours around obstacle / stale cell | no (by design) | no | no (tight path critics; 0.4 m box centred on path) |
| oscillation (wz flips/s nominal ideal / mc_hyp) | 0.11 / 0.41 | 0.10 / 0.30 | 0.11 / 0.31 (0.18 transient) |
| vx jerk RMS nominal ideal / mc_hyp [m/s^3, per 20 Hz cycle] | 5.2 / 11.6 | 7.5 / 11.7 | 2.6 / 0.65 (smoothest) |
| CPU (nominal / obstacle / stale) ms per cycle | 2.1 / 2.2 / 2.0 | 11.2 / 11.1 / 10.4 | 8.8 / 29.0 / 34 (68 % of a core) |
| period p99 (worst run) | 50.9 ms | 53.5 ms | 55.1 ms (no "missed rate" warnings in any run) |
| RSS (VmHWM) | 67-71 MB | 70-74 MB | 73-80 MB |
| P23 adapter view | pure-rot 3.5 % (ideal) / 20 % with 5 switches (mc_hyp); no sub-deadband rot | 6 % / 13 %; sub-deadband rot 1.5-9 % | 0-7 % pure-rot, ≤2 switches; low-vx 4-31 % near obstacles; sub-deadband rot up to 17.5 % when blocked |
| cmd gaps >150 ms (adapter timeout) | 0 | 0 | 0 |

## 4. Interpretation
- Controller choice does NOT fix the two observed field problems:
  (a) stale blind marks (D-04/D-04b): all three are blocked by the same cluster (it lies on the forward/right-turn swept footprint);
      none detours. Fix stays in the costmap/clearing/odom domain (D-04 C, D-06), not in the controller.
  (b) RPP "collision ahead" repeats are the correct reaction to LETHAL cells on the swept footprint; DWB/MPPI fail the same goals
      via the progress checker instead (silent stall 15 s) — less diagnosable, not safer.
- Real-obstacle stopping with IDEAL dynamics: RPP keeps the largest margin (0.38 m); MPPI 0.11 m; DWB 0.03 m (ObstacleFootprint
  only rejects collisions, cost scale 0.02 barely repels) -> DWB rejected as primary candidate.
- With the HYPOTHESIS MC response (1.66x overspeed + 0.4 s dead time + offset gait speed): RPP and DWB physically CONTACT the box
  (commanded-speed collision horizon 1.5 s / 1.7 s is ~1.6x optimistic and the robot coasts 0.2-0.3 m); after the overshoot the box
  cells in front of the robot were lost from the costmap (inside the 0.45 m blind range nothing re-marks; mechanism of the loss not
  isolated — HYPOTHESIS: grazing clearing rays / footprint clearing) and both drove through. MPPI did not touch (0.02 m / 0.002 m)
  but only by aborting (Optimizer fail). => On this MC NO controller is safe near obstacles until the MC speed map is calibrated.
- MPPI pros: smoothest commands (lowest jerk by 2-18x), best goal accuracy, fewest pure-rotation mode switches (good for the P23
  yaw loop, which RPP's rotate-to-heading interleaving triggers 5x per goal under mc_hyp), no "collision ahead" spam, intermediate
  clearance. MPPI cons: CPU 4x RPP nominally and up to 15x (29-34 ms CPU per 50 ms cycle, ~70 % of a core) when obstacles/blocked
  — still no rate misses here, but the robot also runs FAST-LIO/level_bridge; issues many LOW-vx commands (0.02..0.15 m/s: up to 31 %
  of ticks near obstacles) which the MC (per D-05 hypothesis: offset ~0.34 m/s) would NOT execute as slow motion -> MPPI's slow
  approach is exactly the regime the MC distorts most; sub-deadband wz (≤0.15 rad/s) passed by the adapter unchanged (MC ignores it)
  up to 17.5 % when blocked; no accel limits in 1.1.18 (relies on smoother); failure mode = optimizer fail / progress stall.
- MC arbitration: unchanged by any controller (all output through the same smoother -> adapter path; 0 gaps > 150 ms).

## 5. Verdict
- Moving to MPPI: CONDITIONAL YES as the main candidate (vs DWB: NO). It beats RPP on smoothness, P23 compatibility, stop margin
  under both MC overspeed hypotheses (0.02-0.11 m vs RPP contact/0.002-0.03 m) and, with D-04 C clearing, it recovered from the stale
  blind cell; it does not beat RPP on stop margin with ideal dynamics (0.11 vs 0.38 m) and costs 4-15x CPU.
  Do NOT switch on hardware before:
  1. D-05 calibration (straight walks at fixed cmd 0.05..0.30 + stops; MC speed map incl. low-speed region, dead time, coast).
     Then either an inverse speed map in the adapter or limits that keep cmds in the near-unity region; re-run this harness with
     the measured map (model plug-in in sim_model.RobotModel). Without it, all controllers' collision horizons are optimistic.
  2. D-07: obstacles below ~0.34 m are invisible to every controller (sensor issue; this harness cannot help) — operator rule or
     a second sensor before unattended MPPI runs.
  3. D-04/D-04b clearing fix (C or equivalent) + D-06 odom creep: stale cells block all three controllers identically.
  4. MPPI tuning pass in this harness: wz_std/PathAlign weights for sub-deadband wz, VelocityDeadbandCritic (present in 1.1.18)
     with the MC deadbands, CPU at batch 1000 vs 2000 on the robot under FAST-LIO load; then a dry run (controller_dry) before LIVE.
- Keep RPP live until then; RPP's repeated "collision ahead" is correct behaviour, not a fault.

## 6. Extra runs (results_C/, results_mclin/)
D-04 candidate C clearing (clearing FOV ±54 = marking band, CMP_CLEAR_FOV=-54,54), ideal dynamics:
| scenario | RPP | DWB | MPPI |
|---|---|---|---|
| stale_cell | blocked, abort 10 s (same as A) | blocked, abort 15 s (same) | SUCCEEDED 20.1 s (A: blocked) — its CCW turn brought the cluster into the ±54 clearing band -> cleared by real rays |
| transient | as A (stop, resume 0.32 s, clr 0.31) | as A (clr 0.08) | as A (clr 0.30, resume 0.22 s) |
-> C changes nothing for RPP/DWB here; with MPPI it turns the D-04b stall into a success because MPPI tries small left turns that
   expose the blind cell to clearing. Single seed: indicative only.
mc_lin = constant gain 1.61 (D-05 measured R at 0.30) + same lag (alternative MC hypothesis without the offset):
| scenario | RPP | DWB | MPPI |
|---|---|---|---|
| static_block | stops 0.034 m, abort | CONTACT -0.024 m, abort | stops 0.085 m, abort |
| transient | stop, box clr 0.002 m, resume 0.94 s, OK | stop, clr 0.028, OK | stop, clr 0.114, resume 0.82 s, OK |
-> Under BOTH MC hypotheses MPPI keeps the largest obstacle margin; RPP's 0.38 m ideal margin collapses to ~0 m; DWB touches.

## Files
/home/agi/control_backend/tests/controllers/{README.md, run.sh, run_compare.py, sim_model.py, test_sim_model.py (8 passed),
fetch_dwb.sh, dds_domain78_unicast.xml, config/{common,rpp,dwb,mppi}.yaml, launch/controller_harness.launch.xml,
data/live2_plan_183029.json, results/ (24 JSON+logs, summary.json), results/_invalid_v1/ (first matrix, wrong clearing model)}.
Standard compliance: XML launch, params only in config/, app logic in sim_model.py (unit tested), ROS node only for comms.
Deviation: harness node uses Python (tooling) and wall time (no /clock) to keep compute-time measurements real.
