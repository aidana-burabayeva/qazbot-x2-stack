#!/bin/bash
# Real localization_trust_node + synthetic D1 publisher (isolated ROS domain 88, inside the dev container).
# usage: ros_node_e2e.sh baseline|candidate  (after stage_and_test.sh staged the tree)
VAR=${1:-candidate}; ST=/tmp/t5_$VAR
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
cd $ST/tests/loc_trust
python3 $ST/ws/src/navigation/localization_trust_node.py --ros-args -p admission_policy:=strict > /tmp/node_$VAR.log 2>&1 &
NP=$!
sleep 2
timeout 100 python3 ros_e2e_synthetic.py > /tmp/e2e_$VAR.log 2>&1; RC=$?
kill -INT $NP; wait $NP; NRC=$?
tail -12 /tmp/e2e_$VAR.log; echo "test_exit=$RC node_exit=$NRC"; grep -ci "traceback\|error" /tmp/node_$VAR.log
