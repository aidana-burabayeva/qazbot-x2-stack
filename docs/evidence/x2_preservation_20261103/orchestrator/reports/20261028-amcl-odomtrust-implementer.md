# AMCL odom-trust (P2a)
Before SHA256: 8ccde5f2eef749dbc7373de7d45433b87c96c37bdcf971b320c33ee40d82aa62
After SHA256:  feebadbb875b240d40f56b14619dbe282772ce8172e667d8120212ed5b6364b2
Change: max_beams 1440->60; alpha1..5=0.1; recovery_alpha_slow/fast=0.0; one comment. Nothing else.
Verify: py_compile OK; generate_launch_description() builds (humble sourced); grep shows new values.
Git mirror: checkout file was byte-identical to pre-change; mirrored, same after-SHA; not committed.
No ROS process touched (running nodes keep old params until relaunch).
Rollback:
cd /home/agi/control_backend/ws/src/navigation && cp -p localization.launch.py.bak_20261028_pre_amcl_odomtrust localization.launch.py
(git mirror: cp the same file to /home/agi/control_backend/qazbot-x2-stack/ws/src/navigation/localization.launch.py)
