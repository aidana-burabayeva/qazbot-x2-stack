#!/bin/bash
# Official: "unset AGIBOT_SOFTWARE_HOME" selects the SIM config (mc_sim.yaml); REAL would use mc.yaml.
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset AGIBOT_SOFTWARE_HOME
cd /workspace/mc/bin && echo 0 | ./em_run.sh
