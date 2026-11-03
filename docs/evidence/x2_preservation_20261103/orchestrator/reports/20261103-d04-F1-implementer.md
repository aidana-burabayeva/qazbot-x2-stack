# 20261103 d04 F1 implementer
R1 DONE: replay_d04.py already had fit_levelling/load_rlv/prepare(level=auto|fit|bag|json) (default fits when bag has scan_obstacles); tests/d04/test_levelling_fit.py had 2 failing tests (fixture lacked an initial odom so on_imu never levelled) -> fixed in fixture. pytest tests/d04: 59 passed (no sleeps, synthetic fixture only).
R2 DONE: candidates/d04/CANDIDATE.md, docs/engineering/D04_D05_CPP_REPORT.md created; evening section appended to docs/hardware/PHYSICAL_NAV_SESSION_20261102.md; control_backend/ws/src/lio/__pycache__/level_bridge.cpython-310.pyc (mtime 20:44:46) removed.
Not touched: running files, ws/src/qazbot_level_bridge. Note: real bags 204153/211835 not registered in BAGS (no obstacles key), so default fit is exercised only via fixture.
