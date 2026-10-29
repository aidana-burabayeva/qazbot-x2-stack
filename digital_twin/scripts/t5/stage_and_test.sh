#!/bin/bash
# Stage Trust tests + one loc_trust variant into /tmp and run the whole loc_trust suite.
# usage: stage_and_test.sh baseline|candidate [pytest args...]   (runs inside the dev container)
set -e
VAR=${1:-baseline}; shift || true
ST=/tmp/t5_$VAR
rm -rf $ST && mkdir -p $ST/ws/src/navigation $ST/tests
cp -r /backend_tests/loc_trust $ST/tests/
for d in /opt/x2twin/stack/*; do [ "$(basename $d)" = navigation ] || ln -s $d $ST/ws/src/$(basename $d); done
[ "$VAR" = preC3 ] || cp /work/candidates/t5_geometry/baseline_c3/tests/loc_trust/*.py $ST/tests/loc_trust/   # C3 tests of the robot
cp /opt/x2twin/stack/navigation/*.py $ST/ws/src/navigation/ 2>/dev/null || true
if [ "$VAR" != preC3 ]; then cp /work/candidates/t5_geometry/baseline_c3/ws/src/navigation/*.py $ST/ws/src/navigation/; fi   # C3 wins over the older tree
if [ "$VAR" = candidate ]; then cp /work/candidates/t5_geometry/cand/*.py $ST/ws/src/navigation/; fi
cp /work/candidates/t5_geometry/tests/*.py $ST/tests/loc_trust/ 2>/dev/null || true
export T5_VARIANT=$VAR
cd $ST/tests/loc_trust && python3 -m pytest -q -p no:cacheprovider -x --ignore=ros_e2e_synthetic.py --ignore=run_ros_synthetic.py "$@"
