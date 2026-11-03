# R3b: floor noise / FP on "walking" bags (offline)
TASK_ID 20261103-reuse-first / R3b. No commits, bags not modified, python -I, ROS only sourced for rosbag2_py (reader only, no node, no graph use).
Scripts: /tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/r3b/ (w5.py extract; an5.py, clean.py analysis; *.pkl, an_*.out outputs).

## 0. KEY CAVEAT: these bags are NOT walking
All three bags contain the raw E1R topic `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` (1011 / 798 / 615 frames) plus lidar IMU, /qazbot_lio/odometry, odom_level, /aima/mc/leg_odometry, tf. Nothing missing.
But the robot is almost stationary (leg_odometry and LIO agree):
| bag | duration | frames with speed >0.1 m/s | leg_odom speed max | path / net disp | |yaw rate| max (lio) |
|---|---|---|---|---|---|
| walk_02 | 100 s | 18-19 / 1011 | 0.31 | 1.62 / 0.97 m | 4.7 deg/s |
| turn_01 | 79 s | 25 / 798 (leg), 2 (lio) | 0.25 | 1.85 / 0.02 m | 4.5 deg/s |
| reverse_01 | 60 s | 20 / 615 | 0.52 | 2.16 / 0.89 m | 7.2 deg/s |
So ~2-3 % of frames are in motion and only a few steps are recorded. "Sustained walking" sway is NOT measured. The moving subset (n=15-19 frames) is reported but is thin. A real continuous walk recording is still needed. (LIO odom roll values are huge, -174..178 deg, because LIO frame is the rolled sensor frame; roll/pitch from it are only usable for pitch.)

## 1. Floor-plane residual (|h|<8 cm band around plane, region |b|<=56 deg; median over frames; cm)
Per-frame fit sd: 0.98 cm median (p1-p99 0.9-1.1 cm) in walk_02; turn_01 p99 2.2 cm (object contamination); moving-frame median 0.99 (walk), 1.27 (turn, n=2).
| range | sd (still) | p99 | p99.9 | sd (moving, walk_02 n=19) | p99 | p99.9 |
|---|---|---|---|---|---|---|
| 0.45-0.8 | 1.38 | 3.55 | 4.3 | 1.37 | 3.7 | 4.4 |
| 0.8-1.25 | 1.09 | 2.1 | 3.0 | 1.12 | 4.5 | 7.4 |
| 1.25-2.0 | 0.82 | 1.7 | 5.2 | 0.81 | 2.0 | 6.6 |
| 2.0-3.0 | 1.04 | 4.8 | 7.5 | 0.88 | 2.7 | 7.3 |
reverse_01/turn_01 still: same within 0.1-0.3 cm (sd 1.1-1.4, p99.9 2.7-7.7 cm). Matches the R3 static numbers (sd 0.9-1.4 cm). The band is inflated at p99.9 mainly at the far bin (the |h|<8 cm selection also admits real objects/edges), and at 0.45-0.8 m (near-range bias +0.6..0.7 cm).
Per-frame plane vs fixed static plane: for still frames nearly identical (sd diff <=0.15 cm). For moving frames the fixed plane shows offset -2.2/-4.3/-2.5 cm at 0.8-2.0 m and p99 up to 5.1 cm (vs 4.5 at best per-frame) in walk_02 (n=19): the fixed plane starts to matter when the body moves.

## 2. Tilt (plane normal vs static n0) and its dependence
- Plane-normal tilt vs static: p1/p50/p99 = 0.30/0.37/2.6 deg (walk), 0.09/0.36/2.4 (turn), 0.28/0.53/3.3 (reverse). Fwd(pitch) sd 0.47-0.62 deg, lateral(roll) sd 0.28-0.35 deg, d (height) sd 0.7-0.9 cm. Max seen ~3.3 deg.
- Correlates with LIO pitch (r -0.3..-0.67) and weaker with roll; IMU-accel vs plane-normal angle is noisy (sd 0.7-1.1 deg, corr with tilt ~0): IMU acceleration (±0.1 s mean) is not a usable proxy while stepping.
- 3 deg tilt changes height at r=2 m by 2*tan(3 deg)... = ~10 cm worst-case for a fixed plane. Observed p99.9 band extremes (7-8 cm) fit this.
- Tilt rises in frames where objects are present (turn: 1.46 vs 0.46 deg clean; reverse 1.00 vs 0.31). Cause unknown: either the per-frame RANSAC fit is contaminated by objects, or the body leans; not separated.

## 3. FP rate: points above threshold in the floor area (r 0.45-2.0, |b|<=56, h in (T, 0.5), per-frame plane; fixed plane in brackets are nearly equal)
All frames (includes real objects/people):
| bag | T | frames >=1 pt | >=3 pts in one 5 cm cell |
|---|---|---|---|
| walk_02 | 0.05/0.07/0.10 | 58.6/51.8/51.1 % | 50.7/50.7/50.7 % |
| turn_01 | 0.07 | 15.3 % | 12.7 % |
| reverse_01 | 0.07 | 72.7 % | 72.0 % |
The >=3/cell rule does not remove these: they are dense clusters (median 8-18 pts per cell, max 119-142, cell mean height median 0.27-0.30 m, up to 0.5 m, spread x 0.25-1.95 m, y +-1.5 m, in 2-3 contiguous runs per bag, 513/101/443 frames). That is real structure above the floor in the first ~50 % of walk_02 etc., not floor noise; I did not identify the objects (people/furniture/box not checked, no map/video). Reported only as "points above threshold in the floor area".
Object-free frames (frames with no >=3-pt cell of mean height >0.12 m; 498/1011, 697/798, 172/615 frames) = floor-noise estimate:
| bag | T | >=1 pt (frame plane / fixed) | >=3 pts per cell | pts/frame |
|---|---|---|---|---|
| walk_02 | 0.05 | 15.9 / 6.8 % | 0 | 0.19 / 0.07 |
| walk_02 | 0.07 | 2.2 / 1.2 % | 0 | 0.02 |
| walk_02 | 0.10 | 0.8 / 0.6 % | 0 | 0.01 |
| turn_01 | 0.05/0.07/0.10 | 17.9/3.0/1.3 % | 0.4 (3 frames)/0/0 | 0.25/0.03/0.01 |
| reverse_01 | 0.05/0.07/0.10 | 11.0/2.3/0.0 % | 0.6 (1 frame)/0/0 | 0.19/0.05/0 |
- At T=0.07 and 0.10: zero frames with >=3 pts per cell in the object-free frames of all three bags (total 1367 frames, ~137 s), including the moving frames inside them. At T=0.05: 4 frames of 1367 (0.3 %) have >=3 in a cell (turn/reverse); a 2-frame persistence rule was not re-tested here.
- Per-frame plane fit is WORSE than fixed plane for single-point FP at T=0.05 (15.9 vs 6.8 %, 17.9 vs 8.9 %); equal at T>=0.07 (still frames). Hence per-frame refit is not needed for still robots, and it is not shown to be beneficial. For moving frames (n~19) fixed plane shows -2..-4 cm offset (§1), so the fixed plane may need margin; not enough data to decide.
- Tilt dependence: in object-free frames corr(pts above 0.07, tilt) 0.0-0.19; pts/frame at tilt>1 deg 0.09-0.12 vs 0.02-0.05 at tilt<0.5 deg (n=32/78 frames) - 3-5x higher but still <0.12 pts/frame. In all frames correlation is 0.48-0.67 only because tilt and objects co-occur (objects raise tilt estimate), not evidence of a tilt effect.

## 4. Conclusions for the candidate (floor-relative low band 0.07 m (0.05), >=3 pts per 5 cm cell)
- Supported on these data for near-static robot with small steps (<=0.5 m/s, <=7 deg/s, tilt <=3.3 deg): T=0.07 / >=3 per cell had 0 floor-noise FP frames in 1367 object-free frames; T=0.05 had 0.3 %.
- NOT established: sustained walking gait, dynamic sway at 0.3-0.5+ m/s, turn in place at speed; motion subset is 15-25 frames per bag. Not validated against any real low object (R3 caveat stands).
- Margin: worst-case fixed-plane height error for 3 deg tilt at r=2 m ~10 cm, > 0.07 m threshold. Therefore 0.07 m fixed-plane is not safe at 2 m under 3 deg tilt; per-frame plane (or LIO-levelled up) needed beyond ~1.3 m, or limit range, or T=0.10 at far range. This is an extrapolation from observed tilt, not an observed FP.
- Open: a real walking recording (empty room, known path) is needed; separate objects from floor via map/cluster inspection.
