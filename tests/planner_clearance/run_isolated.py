"""Reuse the original 20-case exact-clearance harness; no broad pkill or hardware."""
import os,sys,subprocess,signal,time,json
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('isolated domain 87 required')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState
r=Path.home()/'control_backend';o=r/'tests/integration_results'/(sys.argv[1] if len(sys.argv)>1 else 'clearance_smac_bounds');o.mkdir()
# The cases (random endpoints, seed 7) depend on the map, so the identical-cases baseline is per map.
MAP='first_map_nav2_v2';BASE=r/'tests/planner_clearance/results'/f'x2_cell_margin_true_{MAP}.json';ns='qazbot_clearance';procs=[];logs=[]
def start(name,args):
 f=(o/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);procs.append(p);return p
rclpy.init();n=Node('clearance_readiness');s=n.create_client(GetState,f'/{ns}/nav2/planner_server/get_state')
def wait(pred):
 end=time.monotonic()+40
 while not pred():
  if time.monotonic()>end:raise RuntimeError('readiness observation deadline')
  rclpy.spin_once(n,timeout_sec=.05)
try:
 start('model',['python3',str(r/'tests/nav_e2e/model_fixture.py'),ns])
 start('nav2',['ros2','launch',str(r/'ws/src/navigation/nav_full.launch.py'),f'ns:={ns}','robot_radius:=0.65',f'scan_topic:=/{ns}/scan',f'odom_topic:=/{ns}/odom'])
 wait(s.service_is_ready)
 while True:
  f=s.call_async(GetState.Request());wait(f.done)
  if f.result().current_state.id==3:break
 os.environ['QAZBOT_COMPUTE_ACTION']=f'/{ns}/nav2/compute_path_to_pose';os.environ['QAZBOT_COMPUTE_ONLY']='1'
 result=subprocess.run(['python3',str(r/'tests/planner_clearance/harness.py'),str(r/'ws/src/maps'/MAP/'map.yaml'),'0.65',str(o/'clearance.json')],capture_output=True,text=True,timeout=180)
 (o/'harness.log').write_text(result.stdout+result.stderr)
 if result.returncode:raise RuntimeError('clearance harness failed')
 d=json.loads((o/'clearance.json').read_text());base=json.loads(BASE.read_text()) if BASE.exists() else None
 compare={'map':MAP,'new':d['summary'],'baseline':base and base['summary'],'identical_cases':None if base is None else d['cases']==base['cases']}
 (o/'comparison.json').write_text(json.dumps(compare,indent=2));print(json.dumps(compare),flush=True)
 if d['summary']['violations_lt_R'] or compare['identical_cases'] is False:raise RuntimeError('clearance regression changed')
finally:
 for p in reversed(procs):
  if p.poll() is None:p.send_signal(signal.SIGINT)
 codes=[p.wait(timeout=15) for p in reversed(procs)]
 for f in logs:f.close()
 n.destroy_node();rclpy.try_shutdown();(o/'shutdown.json').write_text(json.dumps(codes))
 if any(codes):raise RuntimeError('unclean shutdown')
