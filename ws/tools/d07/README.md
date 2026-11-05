# D-07 offline low-obstacle analysis scripts

Offline, read-only analysis of the raw E1R LiDAR cloud in a recorded mcap bag (see docs/engineering/reuse_first_20261103/20261103-reuse-R3-d07.md). Not part of the runtime. Needs a ROS 2 environment (rosbag2_py, rclpy) and numpy.
Usage: `python3 ext.py BAG OUT.pkl [TOPIC] [WINDOWS]` extracts per-frame floor-plane fits; the other scripts (an, box*, fp*, order, peek, plane0, rings, theory) analyse them.
Caveat: scripts hardcode absolute paths (bag path under /home/agi/control_backend/docs/evidence/..., and ext.py loads plane0.npy from the original scratch dir). Adjust before reuse; logic was left unchanged. Intermediate .pkl/.npy data are not committed (plane0.npy is produced by plane0.py).
