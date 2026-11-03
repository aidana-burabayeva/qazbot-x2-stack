# 20261103-pd04-correction (F-PD04)
Operator fact: NO boxes during recording of bag d04_static_bag_183015 (operator only stood/walked; 21 cm and 45 cm boxes placed later, unrecorded; now 21 cm RIGHT, 45 cm LEFT).

## Files changed
- /home/agi/.orchestrator/reports/20261103-pd04-analysis.md (correction section on top + edits)
- /home/agi/.orchestrator/reports/20261103-pd04-cpp.md (correction section on top + segment labels)
- /home/agi/.orchestrator/reports/PLAN_20261102-d04-costmap-source.archived.md (correction note appended only)
- Worktree wt_feedback, branch robot/feedback-d04-d05-20261102, commit f41dc85f1a72e07c515f24bd27d4ceb37ca0bcdf (not pushed): docs/engineering/pd04_20261103/20261103-pd04-analysis.md and ...-cpp.md (identical to the reports copies). tests/d04/BAGS.md has no box claims: unchanged. D04_D05_CPP_REPORT.md, CANDIDATE.md, README: no box claims about 183015: unchanged.
- Not touched: raw logs. pd04_steps_20261103.txt line "operator correction: boxes were 21 cm and 45 cm (step4 R=45, step5 F=21 then 45)" contradicts the operator statement; discrepancy recorded in the analysis correction section. Operator statement prevails.

## Before / after
- Box 21 cm: "NOT EVALUABLE (absent)" -> "NOT PERFORMED (no boxes during recording)".
- Box 45 cm (R, 1.05 m/-55): "FAIL = D-07/FOV" -> NOT PERFORMED; cluster is unidentified, not scored.
- Box "45 cm" +48: "FAIL = D-07" -> NOT PERFORMED; unidentified cluster.
- Timeline/metric rows "45 cm box R", "box L-edge", "low object": now "unidentified cluster".
- "D-07: confirmed (both boxes present in raw cloud...)" -> "D-07 OPEN, NOT VERIFIED on real LiDAR; code fact only: level_bridge.py:235 band floor 0.329+0.035*r m".
- Section 5 box geometry: marked as calculation, not measurement. Repeat-test item 3 marked NOT YET PERFORMED.
- Unchanged: Person F PASS (A=C); person leaves C PASS / A FAIL (39 edge cells); empty PASS; no real obstacle wrongly cleared by C.
- cpp.md: "box R / box F/+48" parity labels -> unidentified clusters; parity numbers unchanged.
