# .89 reloc/trust logic — user-provided description (principles only, no code)
- State persisted atomically (~/.x2/lio/reloc_state.json: map, FAST-LIO run id, time, T(map<-odom), last map pose) every 5 s in tracking/board and on each board calibration.
- Restore: same LIO run -> reuse correction (seed); new LIO run -> seed so pose = last saved pose (assumes robot not moved), real fix via table board; no file/other map -> global search every 15 s, rejected if ambiguous/low score.
- Scan matching every 5 s; in board mode not applied, discrepancy reported (drift signal). Tracking: misses (score<thr or >0.3 m/5 deg) accumulate; 15 consecutive -> coasting (keep correction, LIO drift ~cm/tens of min). Local check ±0.6 m/25 deg; far global result accepted only score>=0.55, unambiguous, twice same place. Correction slewed slowly (median of 5, <=1 cm/0.2 deg per match); frozen near goal (reloc_hold).
- Jump detector: >2 m/s by message stamps -> 15 s quarantine; >=2 m jump marks session bad. pose_trusted = map frame + fresh<=2 s + no recent jump. Slow drift not detected.
- Source failsafe: no fresh pose <=1 s -> stop publishing map; readers fall back after 6 s.
- Gaps: no trust gate stopping the walker; scan matching made 9 m/32 deg error (score 0.45) on 20.10 -> board mode introduced; park_at pose send untested live; thresholds uncalibrated.
