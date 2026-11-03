# 20261103-odom-standing-hold -- implementer report
STATUS: DONE (candidate, offline-verified; NOT deployed, running bridge untouched, mtime 2026-10-27). Needs independent verification (not self-certified).

## Files (/home/agi/control_backend/candidates/odom_hold/)
- odom_hold.py (new, pure, no ROS): StandingHold + is_standing + measured constants.
- native_odom_bridge.py (candidate) + native_odom_bridge.diff (94 lines vs ws/src/lio): param `standing_hold` (default true), `state_topic`,
  subscribes /aima/mc/common/state (aimdk_msgs imported lazily; ImportError -> WARN + hold off = today's pass-through), hold applied to raw
  x,y AFTER the guards (JumpGuard/SourceGuard see the raw leg pose), twist (0,0,0) while held, status JSON gets `hold{held,standing,offset_m,counts}`.
- test_odom_hold.py (11 pure pytest tests, no sleeps), test_odom_hold_bridge.py (4 in-process node tests, domain 88, injected clock), replay/ (ext2.py, replay.py, combo/quiet/cmd analysis, replay_results.json).
- Deploy = copy odom_hold.py + native_odom_bridge.py into ws/src/lio and restart the bridge (needs aimdk env in the bridge process, as x2_motion_adapter).

## Standing signal (data-verified, 13 bags, ~1.1 M leg samples, 4 h; extraction ext2.py, full-rate leg + full common/state)
common/state fields that vary: fsm_state.current_state, action_info.action_desc, action_info.status, input_source (irrelevant). body_status=1, speed_mode, player_state never vary.
Walk start: action_desc STAND_DEFAULT->LOCOMOTION_DEFAULT (status 100->200->100) while fsm still 4 (~0.1 s), then fsm 5. Stop: fsm 5 + STAND_DEFAULT, then fsm 4 ~0.1 s later.
| state (fsm, action, status) | samples | leg 0.1 s speed p50 / p99 / max (m/s) |
|---|---|---|
| 4 STAND_DEFAULT 100 | 1,101,589 | 0.0005 / 0.0013 / 0.151 |
| 5 LOCOMOTION_DEFAULT 100/200 | 7.3 k / 10.2 k | 0.45 / 0.85 / 0.91 ; 0.71 / 0.88 / 0.94 |
| 4 LOCOMOTION_DEFAULT 100 (start) | 135 | 0.17 / 0.34 |
| 5 STAND_DEFAULT 100 (stop) ; 4 STAND_DEFAULT 200 | 85 ; 34 | 0.07 / 0.16 ; 0.001 / 0.115 |
Per command class (/aima/mc/locomotion/velocity, cmd.py): turn-in-place (14+166+165 sub-samples) -> fsm 5 LOCOMOTION_DEFAULT (yaw rate p50 1.0-1.2 rad/s); zero command after walking stays 5/LOCOMOTION_DEFAULT (v p50 0.02-0.1, decelerating); zero command at rest = 4/STAND_DEFAULT/100 (53 k samples, v p99 0.001). Commanded "translating" while still 4/STAND_DEFAULT/100: leg v = 0.0005 (MC has not acted yet): state follows physical motion, not the command.
Hence is_standing = fsm==4 AND action=='STAND_DEFAULT' AND status==100 AND body_status==1 (state age <= 1.0 s; observed 10 Hz, max gap 0.50 s).
FINDING (why the state alone is not enough): inside 4/STAND_DEFAULT/100 the pose still moves in two cases without any state change: (a) ~1 s settling after a walk (v up to 0.15 m/s after fsm->4), (b) external shove of the standing robot (nav_bag_183504 t=164 s: 25 mm and 46 mrad in 1 s, no field changes; vendor twist reacts). So a second, pose-based gate is part of the rule.

## Hold rule (odom_hold.py)
held = standing(state, fresh) AND quiet(raw pose over trailing 0.5 s: every window sample within D_POS=0.45 mm and D_YAW=0.2 mrad of the newest).
Constants are MEASURED maxima, not tuned: over 1.09 M quiet standing samples (standing >=5 s, excluding +-5 s around any 1 s excursion >1.5 mm/1.5 mrad), 0.5 s window excursion: p50 0.12 mm, p99 0.29 mm, max 0.450 mm; yaw p99 9 urad, max 199 urad (quiet.py output; W=1 s max 0.60 mm, W=2 s 0.80 mm). Vendor drift contributes only 0.085 mm per 0.5 s. A noise outlier only RELEASES the hold (= today's behaviour), never invents motion. Worst case of hidden real motion while held: < D/W = 0.9 mm/s (bounded; replay max held-window excursion 0.449 mm).
Mechanics: while held, published = frozen p_hold, offset = raw - published is refreshed each sample (absorbs the vendor drift); otherwise offset is constant, published = raw - offset => continuous at entry/exit, raw increments outside holds. Yaw is never held (raw MC yaw always); a yaw change > D_YAW over 0.5 s releases the translation hold. Window restarts on stamp-back or source gap > 0.2 s (offset kept). Pose gate releases in 1-3 samples (<=20 ms) of a locomotion start, before the 10 Hz state arrives.

## Offline replay (replay.py: real SourceGuard + StandingHold, bag receive order; 13 bags; 185835 and 185926 contain the same recording, count once)
Standing intervals >= 30 s (state-standing, first 3 s trimmed): raw -> published drift, cm/min (held fraction):
oc_175920 7.8 min 1.00 -> 0.00 (1.0) | 1.3 min 1.00 -> 0.00 (0.99) | oc_180934 8.0 min 1.02 -> 0.00 | 181810 1.02 -> 0.00 | 182106 1.01 -> 0.01 | 182226 1.03 -> 0.00 | 182355 1.02 -> 0.00 | 183029 1.02 -> 0.00 | 184039 14.4 min 1.04 -> 0.00 | 185510 2.4 min 1.04 -> 0.00 | 185926 28.5 min 1.04 -> 0.00 (net 29.5 cm -> 0.1 cm).
183504 (contains the 25 mm shove): raw 0.77 -> published 0.28 cm/min: the real 0.9 cm shove is passed through, the drift is removed.
Held fraction of all state-standing samples: 0.81 (183911, much walking) .. 1.00; typical 0.98-1.00; samples held while state not standing: 0 in all bags.
Transitions (all entries/exits, 11-17 per long bag): published step at entry/exit == raw step (max 0.17 mm, i.e. one 6.7 ms sample; no jump; the max raw step of any sample is 26 mm in walking).
Walking (state not standing, 0 held samples; 4.4-18 m path per bag; oc_175920 12.6 m, 185926 18.4 m, LIVE-1 183911 3.9 m, LIVE-2 185510 3.5 m): offset never changes (0 changes); published step - raw step max 1.3e-15 m (float rounding of x - offset; exactly 0 in most bags); bit-identical x,y before the first hold in all bags (pre-hold samples 46-85); yaw is the raw yaw by construction. Net displacement over a whole bag: unchanged to the removed drift (183911: 3.0956 m raw vs 3.0959 m published).
Independent check with FAST-LIO (4 bags with LIO): LIO 1 s displacement over whole-window-held intervals max 1.6 / 1.6 / 1.9 / 3.7 cm (p50 0.3-0.6 cm = LIO wander), vs up to 73 cm in other intervals: the robot really stood whenever held.
Existing 28 tests/lio/test_native_odom_bridge.py (copy pointed at the candidate): OK. New: 15 passed (pytest, 1.7 s).

## Limits / notes
- Held pose is frozen in leg_odom; the vendor lateral drift of 1.0 cm/min (1.7e-4 m/s) remains in the leg estimate and is simply not published. Drift during LOCOMOTION_DEFAULT with zero command was not measurable (no standing there; no hold applied there by design).
- Release at motion onset loses <= D = 0.45 mm of motion; entry after a stop needs 0.5 s of quiet pose (settling ~1 s after fsm->4 is therefore never held).
- Gates can only be validated on 11-02 data (one robot, speed_mode 1, STAND_DEFAULT). Other states (sit, other body_pose/stand height, MC restart) untested: then is_standing is false -> pass-through.
- Standard deviations: legacy bridge keeps in-code parameter declarations (no YAML/launch change); new logic separated into pure odom_hold.py with unit tests; no README update (legacy package).
- Replay results: /home/agi/control_backend/candidates/odom_hold/replay/replay_results.json; pickles in scratchpad .../scratchpad/hold.
