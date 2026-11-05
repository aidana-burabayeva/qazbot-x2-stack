"""Run actual trust wrapper with synthetic D1 data; isolated domain 88 only."""
import os,subprocess,signal,json,sys
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='88':raise RuntimeError('domain 88 required')
r=Path.home()/'control_backend';o=r/'tests/integration_results'/sys.argv[1];o.mkdir()
f=(o/'node.log').open('w');p=subprocess.Popen(['python3',str(r/'ws/src/navigation/localization_trust_node.py')],stdout=f,stderr=subprocess.STDOUT)
try:
 with (o/'test.log').open('w') as out:
  x=subprocess.run(['python3',str(r/'tests/loc_trust/ros_e2e_synthetic.py')],stdout=out,stderr=subprocess.STDOUT,timeout=100)
finally:
 if p.poll() is None:p.send_signal(signal.SIGINT)
 code=p.wait(timeout=10);f.close()
result={'test_exit':x.returncode,'node_exit':code};(o/'result.json').write_text(json.dumps(result));print(result)
if x.returncode or code:raise SystemExit(1)
