"""Deterministic shutdown test: freeze only the isolated test controller during a lifecycle RPC."""
import os,sys,signal,time,subprocess,json
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('domain 87 required')
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import String
r=Path.home()/'control_backend';o=r/'tests/integration_results'/sys.argv[1];o.mkdir();ns='qazbot_rpc_test'
procs=[];logs=[];status={};controller=None;manager=None
rclpy.init();n=Node('rpc_shutdown_probe')
def status_cb(m):
 global status
 status=json.loads(m.data)
n.create_subscription(String,f'/{ns}/goal_coordinator/status',status_cb,1)
def start(name,args):
 f=(o/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);procs.append(p);return p
def wait(pred,limit=40):
 end=time.monotonic()+limit
 while not pred():
  if time.monotonic()>end:raise RuntimeError('diagnostic observation deadline')
  rclpy.spin_once(n,timeout_sec=.02)
def pid_for(fragment):
 found=[int(x.split()[0]) for x in subprocess.check_output(['ps','-eo','pid=,args='],text=True).splitlines() if fragment in x and '__ns:=/'+ns+'/nav2' in x]
 if len(found)!=1:raise RuntimeError('expected exactly one test child: '+str(found))
 return found[0]
try:
 start('model',['python3',str(r/'tests/nav_e2e/model_fixture.py'),ns])
 parent=start('nav2',['ros2','launch',str(r/'ws/src/navigation/nav_full.launch.py'),f'ns:={ns}','robot_radius:=0.454',f'scan_topic:=/{ns}/scan',f'odom_topic:=/{ns}/odom'])
 wait(lambda:status.get('ready',False),60)
 controller=pid_for('/lib/nav2_controller/controller_server ')
 manager=pid_for('/lib/nav2_lifecycle_manager/lifecycle_manager ')
 os.kill(controller,signal.SIGSTOP)
 client=n.create_client(ManageLifecycleNodes,f'/{ns}/nav2/lifecycle_manager_navigation/manage_nodes')
 wait(client.service_is_ready);req=ManageLifecycleNodes.Request();req.command=ManageLifecycleNodes.Request.PAUSE
 future=client.call_async(req)
 wait(lambda:'Deactivating controller_server' in (o/'nav2.log').read_text())
 os.kill(manager,signal.SIGINT)
 wait(lambda:'rcl preshutdown' in (o/'nav2.log').read_text().split('Deactivating controller_server')[-1])
 g=subprocess.run(['gdb','-q','-batch','-p',str(manager),'-ex','thread apply all bt','-ex','detach'],capture_output=True,text=True,timeout=10)
 (o/'manager_backtrace.log').write_text(g.stdout+g.stderr)
 result={'manager_pid':manager,'gdb_exit':g.returncode,'manager_alive_after_shutdown_signal':Path('/proc/'+str(manager)).exists(),'fixed_prefix':os.environ.get('QAZBOT_LIFECYCLE_PREFIX')}
 if os.environ.get('QAZBOT_LIFECYCLE_PREFIX'):
  wait(lambda:not Path('/proc/'+str(manager)).exists(),5)
  result['manager_exited_while_controller_frozen']=True
 (o/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 if controller and Path('/proc/'+str(controller)).exists():os.kill(controller,signal.SIGCONT)
 for p in reversed(procs):
  if p.poll() is None:p.send_signal(signal.SIGINT)
 codes=[p.wait(timeout=25) for p in reversed(procs)]
 for f in logs:f.close()
 n.destroy_node();rclpy.try_shutdown();(o/'shutdown.json').write_text(json.dumps(codes))
