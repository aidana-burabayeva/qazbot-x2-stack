# NIGHTLY-M1 preparation — 2026-11-01

Base: `e0bfe491ca58c8e915f556bae94b0271a54317eb` on local lab/control. Integration remains `57615780c90e8068e270cf7aa723dd84646cc8cb`. This patch is prepared in a separate sparse worktree, `lab/m1-exec-environment-20261101`; no shared queue, STOP or integration ref is changed.

## New host facts

The operator's commits `1dae63c` and `e0bfe49` address hidden DNS resolver state and volatile Claude state-file fingerprinting. Both real host CLI challenges have successful receipts and raw logs in this directory. [Observation snapshot](host_observations.json) preserves their hashes and model identities. Codex has a completed turn/assistant output, Claude the successful challenge output. These are **CLI capability proofs, not verified engineering results**. Earlier restricted-session failures are not projected onto the ordinary host.

AMCL still has no completed accepted result. At inspection it was parked NEEDS_APPROVAL by LAB_STOP. The shared queue was frozen/STOPPING. No operator STOP was removed and no second orchestrator or daemon was started.

## Additional execution defect and minimal fix

`select_agent()` calculates the execution fingerprint in the daemon's environment. Previously the trusted capability helper was launched by `bash -lc`, which can change PATH/proxy environment through login profiles. [Actual local reproduction](environment_reproduction.json) uses a service-style PATH and the same committed code/filesystem: parent-style and login-shell launches produce different fingerprints for both providers. This is a deterministic environment-boundary defect independent of network/API availability. It does not assert that every historical repeated probe had this sole cause; the earlier volatile Claude state-file cause remains separate.

The trusted helper is now launched directly via the current Python executable and the canonical ROOT workflow.py, preserving the daemon environment. The helper continues to invoke the agent through the existing sandbox. Other tasks retain their established execution path. No weakening of preflight, source/tier gates, filesystem isolation or production safety.

Changed source: `tools/nightly/nq.py` (one dispatch branch); regression: `tests/nightly/test_engineering_lead.py`. No planning redesign or new orchestrator.

## Validation

- [Before](regression_before.txt): new regression FAIL on the old login-shell invocation.
- [After](regression_after_56.txt): **56 PASS**, command: `python3 -m pytest -q tests/nightly/test_engineering_lead.py tests/nightly/test_execution_modes.py tests/nightly/test_labctl.py` in the isolated worktree.
- [Existing host-control source scopes](base_focused_80.txt): **80 PASS** before this isolated patch, command additionally covers test_lab.py and test_stable_branches.py. Overlapping counts, not additive.
- An initial broad focused run in the sparse worktree lacked tracked `digital_twin/.env.lab`; materializing that tracked file resolved the fixture error. It was not a product defect and no tests/gates were disabled.

## Remaining M1 acceptance

Not achieved: integrated fix, independent review, permitted STOP release, current positive preflight, live systemd PID/held lease, actual AMCL task → test rerun → stored engineering result → independent review → next task. Full regression/merged-tree gate must run before promotion. Current session cannot reach the user systemd bus; operator-host execution is required. No run or PASS is manufactured to complete the milestone.

Use the existing Merge Queue to integrate the entire reviewed dependency chain from this branch, preserving published and local history. Do not cherry-pick the remote tree-snapshot commit as if its history equalled local integration. After successful gates and explicit operator resume, start only ENGINEERING through labctl. Keep L3/Full closed until their own evidence exists. Physical X2 forbidden.
