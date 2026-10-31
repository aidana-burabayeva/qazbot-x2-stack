# Lab bootstrap continuation — 2026-10-31

Implementation continues Claude's d970733 on lab/control, based on lab/stable a467c16. The control checkout is /home/adn/QazBot/control_backend_p20_twin02. No physical X2 access, deployment or motion was performed.

Qualification is INCOMPLETE. Current environment denies Docker socket and systemd user bus access; GitHub hostname does not resolve. Therefore real service start, first real Claude/Codex task, fresh MuJoCo smoke, real stop/restart acceptance and push cannot be claimed.

The previously saved `20261031T211313_sim` report was produced by Claude before this continuation: its reported live AMCL/Trust/map/P23 checks passed, but TF uniqueness and Nav2 version compatibility were UNVERIFIED. It is historical evidence, not a new run of the changed ownership probe or bootstrap. The modified TF probe still needs live qualification.

Source checks in this continuation: 73/73 captured X2 source/map hashes match the working tree and separately the lab/stable Git blobs. Automatic lab chain scan found no SSH/robot invocation. New bootstrap tests exercise idempotent campaign preparation, failure-before-start, integration checkout refusal, stale ref refusal, exact parity exceptions, owned Compose cleanup and unknown cleanup failure. Existing lifecycle and regression results are saved alongside this file after completion.

The production queue was not started, unparked or populated during blocked qualification. Its original frozen STOPPED_OK checkpoint was preserved; that stored state is not a new independent observation of host systemd or Docker state. Historical failures remain classified in code for the first successful start: P20 stall superseded by explicit P23 reproduction; worker-kill is expected fault injection; two old nightly build failures require the new preflight, not blind retries.

Validation: full infrastructure + motion adapter regression: **235 passed in 311.72 s**; focused bootstrap including the later duplicate-daemon lease test: **18 passed**; lifecycle + bootstrap at the preceding revision: **30 passed**. The initial broad run exposed an incorrect publication failure classification for fixtures without lab/stable; that compatibility bug was corrected before the passing full regression. Real doctor/start both refused safely with diagnostic reports; no campaign was enqueued.

The original infrastructure commit was integrated locally through the existing Merge Queue as 7e75ab0d2eb4 after **166 nightly tests PASS** on the merged tree. Final follow-up pins the preflight map mount against inherited shell overrides and uses systemd KillMode=control-group so owned verifier subprocesses also stop; focused bootstrap/lifecycle/motion-adapter checks: **102 PASS**. Live systemd behavior still needs host qualification. Git CLI push fails on DNS; GitHub connector read access was separately confirmed, including remote lab/stable still at a467c16.
