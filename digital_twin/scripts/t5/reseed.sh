#!/bin/bash
for v in preC3 baseline candidate; do /work/scripts/t5/stage_and_test.sh $v test_t5_real_reseed.py 2>&1 | tail -1; done
python3 - <<'PY'
import json
for v in ('preC3', 'baseline', 'candidate'):
    for x in json.load(open('/results/t5/reseed_%s.json' % v)):
        print(v, x['bag'], 'seed', x['seed'], 'first post-seed dt=%.2fs' % x['dt_first'], 'motion_ok=%s jump=%s' % (x['first_ok'], x['first_jump']), 'later blocked', x['later_blocked'])
PY
