#!/bin/bash
# Truncated bag: copy, fix the page count in the sqlite header to the real file size, then salvage TF rows.
cp /evidence/First_RViz_goal_1135/bag/bag_0.db3 /tmp/g1135.db3
python3 - <<'PY'
import os, struct
p = '/tmp/g1135.db3'; n = os.path.getsize(p) // 4096
f = open(p, 'r+b'); f.seek(28); f.write(struct.pack('>I', n)); f.close(); print('pages ->', n)
PY
python3 /work/scripts/t5/salvage_tf.py /tmp/g1135.db3 /results/t5/g1135_tf.json
