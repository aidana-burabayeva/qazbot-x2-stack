# 20261103-reuse-first / R5: D-05 B1 (cancel mid-cruise) and B2 (T_nav), offline
Agent: robotics-debugger (read-only). No production change, no domain-0 traffic, no goals on the robot, no motion, no installs, no coefficient change, no commit.
Standard: ~/.claude/standards/henki_ros2_best_practices.md (deviations in §7).
Coordinator correction applied: bag d04_static_bag_183015 contains NO boxes, only the operator. The person events are valid for T_nav.
Process check at write time: no ROS_DOMAIN_ID=84 process running (checked /proc/*/environ of controller_server, velocity_smoother, goal_coordinator, x2_motion_adapter). The dry harness was written but NOT RUN (see §5).
Host note: this host is the X2 compute (housekeeper, vendor stack). The live QazBot stack is running in domain 0 in dry-run mode (adapter pid 242319 live:=false, GC allow_dry_run_driver). Load average was 16 on 8 cores, from another agent's bag extraction at 537 % CPU. Any latency measured on this host now is contention-inflated.

STATUS: B1 ROOT_CAUSE_FOUND from code, dynamic dry run NOT RUN. B2 PARTIAL: the sensing stage is measured from live bag stamps, the costmap→RPP→smoother stage is config-derived (NOT MEASURED).

## 0. Decisions first
| Item | Finding | Grade | Decision |
|---|---|---|---|
| Cancel at the GC public action | `goal_coordinator.py:187-189` cancel() → trip('goal_cancelled') `:119-125`. In that one callback: gate=False, latched=True, ONE zero Twist on `cmd_vel_authorized`, child Nav2 cancel requested. The smoother decel ramp that follows is NOT forwarded (gate closed, `:142`). | code (deterministic) | The cancel is a STEP to zero, not a ramp |
| Adapter on that zero | `x2_motion_adapter.py:282-286`: zero cmd → wants_motion False → ACTIVE→IDLE, burst_left=5. `_drain :374-376` adds one zero per 50 Hz tick, so 5 zeros over ~100 ms, then silence. If the zero is lost: cmd_timeout 0.15 s (`:88-90`) gives the same path. No latch. | code | first zero at MC ≤ 1 tick (20 ms) after GC zero, worst ≤ 0.17 s |
| "release() DISABLE vs DELETE (:376)" | Neither exists. `:376` is the zero burst in `_drain`. The only SetMcInputSource actions in the file are ADD/ENABLE (`:610-611`). `OwnerLock.release` (`:409`) is a file flock. `destroy_node` (`:700-709`) sends a zero burst and drops the flock. The SDK source stays registered (prio 30, lease 1000 ms). Per the vendor doc, zeros do not renew the lease and the data is cleared after the lease. | code + vendor doc (R1) | no deregistration exists. This is an open design question, not a D-05 blocker |
| State after cancel | GC: latched=True, fault='goal_cancelled', busy→False when the child is terminal. It needs `goal_coordinator/resume` (`:236-240`: not busy, final smoother zero seen, trust good) and then a NEW goal with a fresh arm. Adapter: idle, reason no_command, NOT latched. motion_armed stays True until the next latch/arm (`:581-593`; only GC can feed it and GC's gate is closed). Trust node: untouched by cancel. | code | consistent and safe |
| Equivalence | `goal_coordinator/stop` (operator_stop) runs the identical trip() path. Live logs: 14× `TRIP operator_stop`, 4× localization_untrusted, 1× localization_quality_prolonged, **0× `TRIP goal_cancelled`** (grep ~/.ros/log, control_backend/log{,s}). | logs | the cancel path was never exercised live, the stop path was |
| RViz cancel | `rviz/localization.rviz:139-182`: only SetGoal on `/qazbot_d1_map/goal_pose`, no Nav2 panel. `GC.pose_goal :77-82` forwards via its own pose_client and discards the handle. So **RViz cannot cancel**. A cancel needs an action client (e.g. `ros2 action send_goal` + Ctrl-C), the cancel-all service `/qazbot_d1_map/navigate_to_pose/_action/cancel_goal` (zero uuid/stamp), or `goal_coordinator/stop`. | config | protocol must name the exact method |
| P23 location | `/home/agi/control_backend/ws/src/navigation/x2_motion_adapter.py` :71-74 (yaw_w_min 0.15), :83-85 (yaw_hold_s 1.0), :153-155, :166-173 YawRateLoop.pause, :178-184, :353-355, :368-373. Candidate copy `x2_motion_adapter.py.p23_candidate`. NOT in the repo `qazbot-x2-stack` HEAD 982f618, whose copy differs (yaw_w_min 0.05, no pause/hold). The running adapter executes `ws/src/navigation/x2_motion_adapter.py`, so the deployed copy has P23. | files | the repo/deploy divergence must be fixed before any C++ port |
| Cancel as P-D05 cruise-stop | ADMISSIBLE for the **abrupt stop-from-cruise** class: deterministic step at GC, adapter zero burst, GC latch, same path as the live-used operator_stop. It does NOT produce the **obstacle-stop class**: RPP collision → controller zero → smoother −0.6 m/s² ramp (0.30→0 in 0.5 s) forwarded by an open gate. So cancel/stop cannot stand in for the obstacle-stop coast in C1/C2. | derived | USE `goal_coordinator/stop` (preferred, already live-proven) or the cancel-all service for C3-abrupt. Obstacle-ramp class is still BLOCKED (§3) |
| T_nav sensing stage | measured from live bag stamps (below) | measured | replaces the 0.1 s scan-period assumption |
| T_nav costmap→adapter | 5 Hz costmap + 20 Hz controller + 20 Hz smoother + 50 Hz adapter, plus a 0.5 s smoother ramp to zero | config | NOT MEASURED (harness ready, not run) |

## 1. System reconstruction (relevant slice)
- Live params: `/home/agi/.ros/log/qazbot_ctl_t9htg14z.yaml` (2026-11-02, the same file the d05 dry harness uses).
  - RPP: desired 0.30, lookahead 0.5 (0.3-0.9, not velocity-scaled), use_collision_detection true, max_allowed_time_to_collision_up_to_carrot 1.5, cost-regulated scaling false, controller 20 Hz, **failure_tolerance 10.0**, progress_checker 15 s.
  - local_costmap: update 5 Hz, publish 2 Hz, odom rolling 4×4 m/0.05; marking only from `/qazbot_lio/scan_obstacles`, clearing from `/qazbot_lio/scan_clearing`; footprint front x=+0.17.
  - Smoother: 20 Hz OPEN_LOOP, max 0.30, decel −0.6, timeout 0.5.
- BT (`ws/src/navigation/bt/navigate_to_pose.xml`, identical to the repo working copy): RecoveryNode(4){PipelineSequence{RateController 1 Hz ComputePathToPose|AlwaysSuccess, FollowPath}, ClearLocal+ClearGlobal}. There are no motion recoveries. When FollowPath fails, both costmaps are cleared and FollowPath is retried, up to 4 times.
- Chain: controller_server(cmd_vel_nav) → velocity_smoother → `/qazbot_d1_map/cmd_vel` → GC (sole-publisher check, gate) → `cmd_vel_authorized` → adapter 50 Hz → `/aima/mc/locomotion/velocity` (live) or `~/dry_run_cmd` (dry).
- Bag topics (metadata.yaml) include raw `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` (10 Hz), `/qazbot_lio/cloud_registered` (10 Hz), `scan_obstacles`/`scan_clearing` (5 Hz), `/qazbot_d1_map/tf` (map→odom, odom→base_footprint), live `costmap_raw` (~1.7 Hz). There is no tf_static. scan_obstacles frame = base_footprint, raw = lidar_chest_front, LIO = camera_init.

## 2. B2 evidence: sensing stage (measured, live bag, 6 windows, 395 scan_obstacles / 790 raw frames)
Script: scratchpad `r5/ext_win.py` (rosbag2 mcap seek, read-only) and `r5/an1.py`, `r5/an2.py`. Windows (bag s): 43-57, 73-87, 224-238, 365-377, 393-404, 493-507.
| Quantity | p50 | p99 | max |
|---|---|---|---|
| raw cloud: recorder receive − header stamp | 101.8 ms | 103.1 | 104.3 |
| cloud_registered: receive − stamp | 30.0 | 33.5 | 36.2 |
| scan_obstacles: receive − stamp | 37.9 | 43.4 | 52.9 |
| scan_obstacles stamp − cloud_registered stamp | 0.0 | 0.0 | 0.0 (stamp propagated from LIO) |
| scan_obstacles stamp − preceding raw stamp | 95.2 | 95.2 | (min 88.5) |
| **scan_obstacles available − raw scan stamp** | **133.1** | **138.6** | **148.1** |
| scan_obstacles period | 200.0 | 206.9 | 216.4 (5 Hz: every 2nd scan dropped) |

Interpretation:
- Raw stamp ≈ scan start; the sweep takes 100 ms; LIO stamps at about scan end.
- A newly appearing object becomes visible in a raw scan that spans 100 ms. Every 2nd scan is not converted, because level_bridge runs at 5 Hz.
- Appearance → scan_obstacles containing it therefore lies in **[~0.03, ~0.24] s**: up to 100 ms waiting in a dropped scan, up to 100 ms sweep, then +133-148 ms.
- Sensing worst case for the budget: **0.24 s**. This is the measured stamp chain plus the 5 Hz decimation bound. Raw-cloud segmentation of the exact first scan per event was NOT DONE.

Obstacle-in-corridor events in the bag (scan_obstacles points with 0<x<0.65, |y|<0.40 in base_footprint, all the operator):

| bag t (s) | first corridor frame | xmin |
|---|---|---|
| +50.51 | n=52 | 0.31 m |
| +79.31 | n=63 | 0.29 m |
| +229.91 | 13 pts | 0.36 m (person-front segment) |
| +371.11 | 8 pts | 0.54 m (crossing) |
| +503.71 | 14 pts | 0.59 m, then 0.30 |

- Not in the corridor: +398.9 (xmin 0.83) and +501.9 (≥0.74).
- These are the stimuli for the costmap→RPP stage. All are inside the RPP projection: the footprint front 0.17 + 1.5 s × 0.30 = 0.62 m.

## 3. B2: downstream stage (NOT MEASURED, config-derived)
| Stage | Bound | Source |
|---|---|---|
| scan_obstacles → costmap update | ≤ 0.20 s (5 Hz) + update cost | params |
| costmap → RPP collision check | ≤ 0.05 s (20 Hz) | params |
| RPP collision → controller output | HYPOTHESIS: with failure_tolerance 10 s, Humble 1.1.x controller_server publishes ZERO each cycle and keeps the goal for up to 10 s, so it auto-resumes if the projection clears; FollowPath only aborts after 10 s → BT clear-costmaps + retry | NOT CHECKED (no source/log verification) |
| smoother | first reduced cmd ≤ 0.05 s; zero only after the 0.30/0.6 = **0.5 s ramp** | params, R1 |
| GC forward + adapter tick | ≤ 0.02 s | code |

T_nav, composed (HYPOTHESIS for the downstream part):

| Definition | p50 (approx) | worst |
|---|---|---|
| appearance → first reduced cmd at adapter | 0.13+0.10+0.025+0.025+0.01 ≈ **0.29 s** | 0.24+0.20+0.05+0.05+0.02 = **0.56 s** |
| appearance → zero cmd at adapter | ≈ 0.8 s | **≈ 1.06 s** |

- R1's 0.42 s assumed a 0.1 s sensing stage. The measured sensing stage (up to 0.24 s worst) makes the worst case larger.
- p99 of the end-to-end event latency is NOT available: 0 replay trials were run, and the bag has only 5 events, so even a run would support max rather than p99.

## 4. C1/C2 recomputed (inputs as R1 §5.1; MC coast at cruise is still UNMEASURED and needs hardware)
- v_real(0.30) = 0.482 m/s (measured, D1).
- d_coast_max = 0.30 m: abrupt-step samples, 2-3 only. For the smoother-ramp obstacle stop it is UNKNOWN and plausibly larger: the MC 0.4 s dead time at 0.48 plus the ramp plus the post-ramp coast. The ~0.4 m figure is a HYPOTHESIS.
- M = 0.025 m; D_new = 0.525 m.

| Criterion | Inputs | Result | Margin | Verdict |
|---|---|---|---|---|
| C1 worst | T_nav 0.56 → 0.482×0.56 + 0.30 + 0.025 | 0.595 m > 0.525 | −0.07 | **NOT MET** (R1 had −0.005) |
| C1 p50 | T_nav 0.29 → 0.14 + 0.30 + 0.025 | 0.465 ≤ 0.525 | — | MET at p50 only. Not acceptable as a safety criterion |
| C2 as R1 | same T_nav 0.56 | 0.595 > 0.45 | −0.145 | NOT MET |
| C2 (known obstacle) | a known obstacle is already in the costmap, so the sensing and costmap stages drop out: T_nav_known ≈ controller 0.05 + smoother 0.05 + adapter 0.02 = 0.12 s → 0.058 + 0.30 + 0.025 | 0.383 ≤ 0.45 | +0.067 | MET only IF d_coast_max(ramp stop) ≤ 0.30 m, which is UNMEASURED. With the ~0.4 m hypothesis it is NOT MET. odom/TF lag NOT CHECKED |

Conclusion: C1 stays NOT MET at cmd 0.30 with the measured sensing stage. C2 is undecidable until the ramp-stop coast is measured on hardware. No coefficient change is proposed.

## 5. Dry harness (prepared, NOT RUN)
- Files are in the scratchpad: `/tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/r5/{harness.py,run.sh,ext_win.py,an1.py,an2.py,win.pkl}`. They are not placed under tests/d05: this agent is read-only, and an implementer may promote them.
- Real components: controller_server and velocity_smoother 1.1.18 (env.sh runtime) with the live params, goal_coordinator.py, and x2_motion_adapter.py with live:=false.
- Stubs: bt_navigator get_state=3, a private NavigateToPose→FollowPath forwarder, trust Bool/status, arm_check, McCommonState (rc, stand, fsm 4), map→odom identity, odom.
- run.sh guards: ROS_DOMAIN_ID=84, ROS_LOCALHOST_ONLY=1, vendor DDS profile unset.
- B1 trials: cancel via handle ×REPS, cancel-all service, GC stop.
- B2 trials: the 5 events × REPS, bag replay restamped with the original receive-vs-stamp offset preserved.
- Recorded topics: cmd_vel_nav, smoother out, cmd_vel_authorized, dry_run_cmd, GC/adapter status, costmap_raw corridor lethal count.
- Caveat: run it only when the host load is low; the measured latencies are contention-sensitive.

## 6. Symptom / proximate / root cause (B1/B2 view)
- Symptom: P-D05 has no admitted cruise-stop, and T_nav was a hypothesis.
- Proximate (B1): the stop path exists (GC trip). It was never exercised by cancel live, RViz has no cancel, and the protocol does not name a method.
- Proximate (B2): there was no instrumented replay.
- Root cause (safety budget): two stop classes with different command shapes reach MC:
  - GC trip: a step.
  - RPP collision: a smoother ramp through an open gate, plus failure_tolerance hold/auto-resume (hypothesis).

  Only the step class can be produced on demand, so the obstacle-stop coast cannot be measured with cancel/stop.

## 7. Deviations / Henki
- No production files touched.
- Harness = test tooling in the scratchpad. It uses print and a polled single-threaded spin and has no README; it needs a README and tests/d05 placement if promoted.
- GC/adapter legacy style observed: ERROR-level TRIP log on a normal operator cancel (log-level misuse candidate).

## 8. Open questions / NOT CHECKED
- Downstream T_nav measured values (run harness: `REPS=3 bash /tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/r5/run.sh` at low load).
- controller_server failure_tolerance behaviour on RPP collision (zero-and-hold vs abort; auto-resume when the person leaves). Check `ctl.log` in the harness run, or the 1.1.18 source.
- Exact first raw scan per event (raw segmentation not done).
- How to produce an obstacle-class (ramp) stop from cruise on hardware without a real obstacle. Candidate: a protocol step with a soft obstacle/person at the calibrated distance. That is an operator/safety decision; not proposed here.
- Whether an SDK source should DISABLE/DELETE on shutdown (vendor semantics of a stale registered prio-30 source after a restart).
- odom→base_footprint TF latency (affects C2 known-obstacle).
