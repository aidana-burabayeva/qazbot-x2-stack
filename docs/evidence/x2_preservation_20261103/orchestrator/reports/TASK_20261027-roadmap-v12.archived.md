# TASK

## Task ID
20261027-roadmap-v12

## Objective
Execute QazBot X2 roadmap v1.2 (reports/QazBot_X2_Autonomous_Navigation_Roadmap_v1.2_2026-10-27.md),
first two parallel jobs per §8: A/P2-T trace-first-delayed-message, B/V0–V2 vendor capability + D1-map shadow.

## Robot context
- This host = physical X2 PC2 (soc1), IPs 10.0.1.41 / 192.168.1.72. Identity vs historic .66 UNVERIFIED (V0).
- Jetson tegra 5.15, ROS 2 humble, Nav2 1.1.18, AimDK v1.1.x; MC on PC1 10.0.1.40.
- Repo /home/agi/control_backend (NOT git). Map: ws/src/maps/first_map_nav2_v3.
- Live at 16:33 (started by earlier agent today): localization.launch (portable FAST-LIO + map_server + AMCL),
  localization_trust_node, x2_motion_adapter live=false, nav_full (dry-run). No MC publisher expected.
- Digital twin lives on laptop adn@192.168.1.64 (~/x2_twin) — UNREACHABLE from here (no route) on 27.10 16:33.
- Known live anomaly (23.10): trust T1_stale while sources fresh — same symptom class as twin P2-T.

## Scope (this session)
1. B/V0: host, serial/profile, SDK/firmware, ROS domain, DDS profile, graph; TF owners; map publishers; MC sources.
2. B/V1: TaskMasterState (slam_state, loc_map_id), vendor map inventory, Localization/Map/PNC license — read-only queries only.
3. B/V2: passive verification of D1 map hash/origin vs live map_server; shadow FAST-LIO+AMCL+Trust health; short bags.
4. A/P2-T (live shadow analogue, since twin unreachable): T0..T6 timeline of scan/odom source→Trust; independent observer; locate first delay. No threshold change, no fake stamps.
5. P0 light inventory of control_backend vs roadmap (reuse/adapt/missing).

## Non-goals / forbidden
Any robot motion; MC source registration; publishing to /aima/mc/*; start_relocalization / vendor nav tasks / mapping;
second map→odom publisher; changing T1/trust thresholds; editing vendor configs; restarting vendor services.
Physical motion → STOP and ask user.

## Acceptance criteria
- V0–V2 report with evidence (commands + outputs) and explicit GO/NO-GO/NEEDS_DATA for vendor-localization branch.
- P2-T report: first delayed segment identified with trace, or honest NEEDS_DATA; trace overhead A/B.
- Verifier PASS on the reports' factual claims (re-run of key read-only checks).
