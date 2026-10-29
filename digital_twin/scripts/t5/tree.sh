#!/bin/bash
# the control_backend tree itself (no overlays)
/work/scripts/t5/stage_and_test.sh preC3 > /tmp/tree.log 2>&1; tail -2 /tmp/tree.log
/work/scripts/t5/ros_node_e2e.sh preC3 > /tmp/tree_e2e.log 2>&1; echo "e2e BAD=$(grep -c ^BAD /tmp/e2e_preC3.log) OK=$(grep -c ^OK /tmp/e2e_preC3.log) $(grep test_exit /tmp/tree_e2e.log)"
