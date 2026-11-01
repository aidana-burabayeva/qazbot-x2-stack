# Operator interactive simulation (digital twin X2) — simulation only

One command starts the official AgiBot MC + MuJoCo, the unmodified X2 navigation stack (AMCL + Localization Trust + Goal Coordinator +
Nav2 + motion adapter) and RViz on the D1 map. **You** place the pose estimate and the goal. Nothing touches the physical robot.

## Start / stop

```bash
cd /home/adn/QazBot/control_backend_p20_twin02          # the control checkout
tools/twin/operator_sim.sh start                         # asks you to type the authorization phrase
tools/twin/operator_sim.sh start --start-pose=-8.344,-1.213,-2.145 --mujoco-gui   # options
tools/twin/operator_sim.sh status                        # containers, Trust/seed diagnostics
tools/twin/operator_sim.sh cancel                        # cancel the active goal (gate closes, coordinator latches)
tools/twin/operator_sim.sh register                      # (re)register the adapter in MC; `start` already does it
tools/twin/operator_sim.sh resume                        # clear the adapter + coordinator latches before the next goal
tools/twin/operator_sim.sh stop                          # SAFE STOP: cancel goal, close the bag, collect logs, tear down, hash, hand off
```
Non-interactive start (only an operator may supply it): `--authorize "I AUTHORIZE ONE OPERATOR SIMULATION SESSION"`.
`start` runs a preflight and refuses (nothing is started) if: another official MC runs or holds `official-mc.lock`; a session is running; RAM < 24 GiB,
disk < Nightly floor 5 GiB + 1 GiB evidence (+1.5 GiB first-time volume seed), CPU load too high; images / D1 map / vendor binaries missing; no DISPLAY.
Build the RViz image once: `tools/twin/operator_sim.sh build-rviz`. Start-up takes ~4–5 min (MC + MuJoCo stand-up).

## What you do in RViz
1. **2D Pose Estimate** → click and drag the robot's pose on the D1 map (`/qazbot_d1_map/initialpose`). Nothing is seeded for you.
2. Read the panel (text above the robot): `TRUST: READY / MOTION_OK`, sigma / scan_score, and the **seed fit**: Trust's own scan score of your pose against
   the real scan and the best nearby pose. `SEED DOES NOT FIT THE SCAN` means the pose is off the map-consistent pose. **The go2 Trust policy does not gate READY on this score**
   (observed: READY with score 0.07 < 0.35), so heed the warning.
3. **2D Goal Pose** → click a goal (`/qazbot_d1_map/goal_pose`; the Goal Coordinator handles it exactly as on the X2, behind Trust). The tool is the Humble "Nav2 Goal" equivalent
   for this stack; the nav2 panel/GoalTool is deliberately not loaded because it would bypass the coordinator and Trust.
4. Watch the X2 move (RViz: map, costmaps, scan, particles, Nav2 plan, footprint, ground truth in blue; MuJoCo window with `--mujoco-gui`).
After `cancel` or any safety stop run `resume`; after an MC/Trust/TF fault re-place the pose estimate. If the stack container is recreated, restart the whole session
(the MC keeps the old `sdk_qazbot_nav2` registration: re-registration fails with SDK response code=1). `start` registers the motion adapter in the official MC once (the stack never does it by itself; this is not a localization step); `operator_sim.sh register` repeats it manually.

## Isolation and safety model
* Own capsule: `mode=OPERATOR_SIM`, `scope=interactive_operator_mc`, `qualifies=false`, ≤ 8 h, written by `operator_sim.py` after your confirmation. The sim container refuses to start without a valid
  capsule (`docker/sim/check_authorization.py`, enforced by `sim_supervisor.sh`) and takes the same `official-mc.lock` as the Nightly L3 worker: **two official MCs cannot run together** (verified: an L3 worker is refused while a session runs).
* Own compose project `x2-operator`, private `internal` network `x2-operator_twin` (no route to LAN/robot/internet, nothing published, no host network, no privileged, caps dropped, no docker.sock).
  ROS domain stays 87 because the twin plant refuses any other (`require_sim_environment`); DDS isolation = separate network (verified: a probe on another network sees 0 operator topics, same-network control sees 46).
* Nightly queue authorization is never read or written; ENGINEERING/L3/FULL_NIGHTLY are unchanged; a session never yields SIM_VERIFIED. The only contact with Nightly is the evidence hand-off after `stop`
  through the existing `workflow.py feedback` (clusters, dedup, producer/review tasks). No SSH, no robot address anywhere in the tool (tested).
* RViz, panel and recorder are separate containers. Writable host mount: the session evidence directory only.

## Evidence (`run/operator/<session>/`)
`snapshot/` (read-only: manifest with git SHA, map/params/compose hashes, image IDs, vendor binary hashes, stack-source-in-image hash, operator-script tree hash),
`worker.json` (capsule, deleted at stop), `evidence/` (rosbag2 of tf/scan/odom/amcl/particles/Trust/coordinator/adapter/plan/costmaps/cmd_vel/MC velocity/ground truth;
`events.jsonl`, `trace.jsonl` 10 Hz, `mc.log`, `sim.log`, `containers.log`, `analysis.json`, `SHA256SUMS.json`; all files read-only after stop).
`analysis.json` reports per goal: outcome, duration, path length, final pose error (after coasting), minimum clearance from ground truth vs the map, stalls (linear and in-place turn), Trust/seed events.
Nightly receives objective signatures only: stall, Nav2 ABORT of a reachable goal (a goal unreachable for a 0.454 m disc is a SAFE_REJECT), T5 jump, Trust READY for a misfitting seed. Operator cancels and wrong seeds as such are not defects.

## Known limitations
* Real MuJoCo contact forces and the real-time factor are **not published** by the vendor sim: clearance is computed from ground truth against the map; missed MC wakeups are in `mc.log` (hundreds/min under load).
* `--mujoco-gui` renders in software (no GPU passthrough) and competes with MC for CPU; not qualified here. Default is headless MuJoCo + RViz.
* The plant builds the scan from the pose assuming map == odom at start, while MuJoCo settles a few cm / several degrees from `--start-pose`; a "correct" seed can therefore score below the Trust floor (fidelity issue sent to Nightly).
* A malformed `x2_twin/scenario_cmd` kills `hal_plant` (no input validation); the stack healthcheck does not notice until nodes are missing.
* Tooling verified end-to-end with a scripted operator that publishes exactly what the RViz tools publish; the RViz tool → topic wiring was verified from the ROS graph (publisher node `rviz`), the physical mouse step was **not** exercised by automation.
* L3/SIM_VERIFIED/HW_VERIFIED are not claimed. Nothing here qualifies the twin.
