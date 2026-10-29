#!/bin/bash
/work/scripts/t5/stage_and_test.sh candidate > /tmp/u.log 2>&1
grep -E "FAILED|^E " /tmp/u.log | head
