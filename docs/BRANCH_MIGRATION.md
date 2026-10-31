# Branch migration (audit)

Two permanent branches: **`robot/stable`** (installed X2 state) and **`lab/stable`** (SIM_VERIFIED lab state). Nothing was deleted, reset or force-pushed; every tip below is still reachable and tagged `archive/<branch>`.

| branch | tip | date | class | reason | commits ahead of robot/stable, lab/stable |
|---|---|---|---|---|---|
| `attempt/nav_frame_fix-1-e4bce3` | `fdd692a7d2c3` | 2026-10-30 16:43 | **INTEGRATED** | tip is contained in lab/stable | 41, 0 |
| `attempt/nav_frame_fix-2-3cfc8f` | `ba40df5c8772` | 2026-10-31 18:57 | **INTEGRATED** | tip is contained in lab/stable | 42, 0 |
| `attempt/nav_frame_repro-1-0e90a7` | `fdd692a7d2c3` | 2026-10-30 16:43 | **INTEGRATED** | tip is contained in lab/stable | 41, 0 |
| `attempt/nav_frame_review-1-11f5b0` | `c47ec77d25a1` | 2026-10-31 18:58 | **INTEGRATED** | tip is contained in lab/stable | 43, 0 |
| `baseline/x2-robot-20261030` | `fac640fd3e4a` | 2026-10-30 17:47 | **INTEGRATED** | tip is contained in robot/stable | 0, 3 |
| `docs/repo-deploy-alignment-20261030` | `d3db63a75f21` | 2026-10-30 13:28 | **ACTIVE** | recent work not yet in a stable branch | 2, 3 |
| `docs/x2-twin-evidence-20261030` | `c9b376ed5ac7` | 2026-10-30 17:38 | **INTEGRATED** | tip is contained in robot/stable | 0, 2 |
| `fix/i1-rpp-collision` | `2d240dbd09ca` | 2026-10-28 15:30 | **NEEDS_REVIEW** | not contained in a stable branch, no newer branch contains it, not recent: a human/agent must decide | 4, 4 |
| `handoff/first-rviz-goal-20261028` | `2b0f9a77fe7f` | 2026-10-28 16:02 | **INTEGRATED** | tip is contained in robot/stable, lab/stable | 0, 0 |
| `integration/operator-skills-robot-20261030` | `e2f44be3b3f6` | 2026-10-30 12:27 | **ACTIVE** | recent work not yet in a stable branch | 1, 2 |
| `lab/integration` | `3cf878a494ae` | 2026-10-31 18:59 | **INTEGRATED** | tip is contained in lab/stable | 46, 0 |
| `lab/stable` | `5ff0cfa1f2fa` | 2026-10-31 19:46 | **ACTIVE** | permanent: SIM_VERIFIED lab state | 52, - |
| `main` | `6e70c0f90665` | 2026-10-28 13:35 | **ARCHIVED** | untouched by operator decision (not the installed X2 state, not the lab state) | 0, 0 |
| `nav/first-goals-20261028` | `982f61802035` | 2026-10-30 11:59 | **INTEGRATED** | tip is contained in robot/stable | 0, 1 |
| `nightly/codex-stall-20261030` | `e54067fe48f9` | 2026-10-30 10:07 | **INTEGRATED** | tip is contained in lab/stable | 20, 0 |
| `robot/stable` | `fac640fd3e4a` | 2026-10-30 17:47 | **ACTIVE** | permanent: installed X2 state | -, 3 |
| `test/p20-odom-regression-20261029` | `ab837253d6cf` | 2026-10-29 21:26 | **INTEGRATED** | tip is contained in lab/stable | 3, 0 |
| `test/p20-twin02-sim-20261029` | `f4116e4b0354` | 2026-10-30 15:42 | **INTEGRATED** | tip is contained in lab/stable | 37, 0 |
| `tooling/operator-skills-20261030` | `d3bd4f14ba38` | 2026-10-31 19:46 | **INTEGRATED** | tip is contained in lab/stable | 44, 0 |
| `tooling/robot-sync-validate-skill-20261030` | `ce4537e72bd6` | 2026-10-30 11:53 | **INTEGRATED** | tip is contained in robot/stable, lab/stable | 0, 0 |
| `tooling/robot-sync-validate-skill-main` | `a77161dae0b7` | 2026-10-30 12:00 | **ACTIVE** | recent work not yet in a stable branch | 1, 1 |

Counts: ACTIVE=5, INTEGRATED=14, SUPERSEDED=0, NEEDS_REVIEW=1, ARCHIVED=1.
