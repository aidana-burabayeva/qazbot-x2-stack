#!/bin/bash
# Prints MC body/fsm/action/source once (inside container env via ex.sh)
/workspace/control_backend/docker/twin/ex.sh timeout 8 ros2 topic echo --once /aima/mc/common/state 2>&1 | python3 -c "
import sys,re
t=sys.stdin.read()
g=lambda p:(re.search(p,t).group(1) if re.search(p,t) else '?')
print('body_status',g(r'body_status:\s+value: (\d+)'),'| fsm',g(r'fsm_state:\s+current_state: (\d+)'),'| action',g(r'action_desc: \"?(\w+)'),'| source',g(r'input_source:\s+name: \"?([\w\.]*)'))"
