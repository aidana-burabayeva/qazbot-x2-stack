# The contract file list: sha256 of every navigation-stack source file. Run from a ws/src directory (repo, image, or robot).
# nav_control.launch.py is excluded: the twin Dockerfile patches the image copy on purpose (SIM-only hook, off by default).
find navigation lio maps -type f ! -name "*.bak*" ! -path "*/__pycache__/*" ! -path navigation/nav_control.launch.py | LC_ALL=C sort | xargs sha256sum
