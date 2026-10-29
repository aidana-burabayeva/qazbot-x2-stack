#!/bin/bash
for b in First_RViz_goal_C3_1205 RViz_goal2_1216 RViz_goal3_1220 Static_map_trial_1230 Static_map_retry_1257 S4_takeover_b_1132; do
  cp /evidence/$b/bag/bag_0.db3 /tmp/x.db3
  python3 -c "
import os,struct;f=open('/tmp/x.db3','r+b');f.seek(28);f.write(struct.pack('>I',os.path.getsize('/tmp/x.db3')//4096))"
  python3 /work/scripts/t5/initialpose_times.py /tmp/x.db3 /results/t5/ip_$b.json.ip 2>&1 | tail -1 | sed "s/^x.db3/$b/"
done
