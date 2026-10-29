#!/bin/bash
# Every physical bag: salvage TF (header fixed on a copy when truncated), list map->odom steps old vs robot-displacement metric.
for b in C3 First_RViz_goal_C3_1205 RViz_goal2_1216 RViz_goal3_1220 Static_map_trial_1230 Static_map_retry_1257 S4_takeover_1129 S4_takeover_b_1132 First_RViz_goal_1135; do
  [ -f /evidence/$b/bag/bag_0.db3 ] || continue
  cp /evidence/$b/bag/bag_0.db3 /tmp/x.db3
  python3 - <<'PY'
import os, struct
p = '/tmp/x.db3'; f = open(p, 'r+b'); f.seek(28); f.write(struct.pack('>I', os.path.getsize(p) // 4096)); f.close()
PY
  echo "== $b"; python3 /work/scripts/t5/salvage_tf.py /tmp/x.db3 /results/t5/$b.json 2>&1 | tail -1
  python3 /work/scripts/t5/replay_steps.py /results/t5/$b.json 2>&1 | tail -8
done
