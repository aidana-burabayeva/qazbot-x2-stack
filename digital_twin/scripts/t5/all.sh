#!/bin/bash
# Full T5 check for one variant: all Trust unit/safety tests + real node e2e.
V=$1
/work/scripts/t5/stage_and_test.sh $V > /tmp/unit_$V.log 2>&1; echo "$V unit: $(tail -1 /tmp/unit_$V.log)"
/work/scripts/t5/ros_node_e2e.sh $V > /tmp/e2e_out_$V.log 2>&1
echo "$V e2e: BAD=$(grep -c '^BAD' /tmp/e2e_$V.log) OK=$(grep -c '^OK' /tmp/e2e_$V.log) $(grep test_exit /tmp/e2e_out_$V.log) node_errors=$(tail -1 /tmp/e2e_out_$V.log)"
