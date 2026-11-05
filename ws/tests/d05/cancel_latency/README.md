# D-05 cancel / latency dry harness - NOT YET RUN

Harness (isolated ROS_DOMAIN_ID=84, localhost only) running real controller_server + velocity_smoother + goal_coordinator + x2_motion_adapter (live:=false) with stubs. B1: cancel mid-cruise; B2: bag replay for T_nav. See docs/engineering/reuse_first_20261103/20261103-reuse-R5-d05-cancel-latency.md.
Usage: `REPS=3 ./run.sh` (hardcodes /home/agi/control_backend/ws paths and a params yaml in ~/.ros/log). `win.pkl` (4 MB replay window) is not committed; generate it with `ext_win.py` from the bag. an1.py/an2.py analyse the output.
Run only when no load from other agents; latency numbers are contention-sensitive.
