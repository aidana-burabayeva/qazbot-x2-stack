#!/bin/bash
for v in baseline candidate; do /work/scripts/t5/stage_and_test.sh $v test_t5_real_bags.py 2>&1 | tail -3; done
python3 - <<'PY'
import json
b = json.load(open('/results/t5/real_baseline.json')); c = json.load(open('/results/t5/real_candidate.json'))
print('steps', len(b), 'blocked baseline', sum(x['blocked'] for x in b), 'candidate', sum(x['blocked'] for x in c))
for x, y in zip(b, c):
    if x['blocked'] != y['blocked']:
        print('DIFF', x['bag'], round(x['t'], 3), 'baseline', x['blocked'], round(x['jump_xy'], 4), 'candidate', y['blocked'], round(y['jump_xy'], 4))
PY
