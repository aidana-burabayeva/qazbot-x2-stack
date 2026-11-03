# 2 Hz JSONL log of Trust/GC/adapter status + AMCL pose; CPU/load snapshot every 5 s. Light (one node).
import rclpy,json,time,os,subprocess
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init();n=Node('orch_status_logger');st={}
for k in ['localization_trust','goal_coordinator','motion_adapter']:
    n.create_subscription(String,f'/qazbot_d1_map/{k}/status',lambda m,k=k:st.__setitem__(k,m.data),10)
def amcl(m):
    p=m.pose.pose;c=m.pose.covariance;st['amcl']={'x':p.position.x,'y':p.position.y,'qz':p.orientation.z,'qw':p.orientation.w,'cxx':c[0],'cyy':c[7],'caa':c[35],'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
n.create_subscription(PoseWithCovarianceStamped,'/qazbot_d1_map/amcl_pose',amcl,10)
f=open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'status_log_%s.jsonl'%time.strftime('%H%M%S')),'a')
last_cpu=0;t0=time.time()
while rclpy.ok():
    end=time.time()+0.5
    while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
    rec={'t':time.time()}
    for k in ('localization_trust','goal_coordinator','motion_adapter'):
        try: rec[k]=json.loads(st[k])
        except Exception: pass
    if 'localization_trust' in rec: rec['localization_trust'].pop('input_callbacks',None)
    rec['amcl']=st.get('amcl')
    if time.time()-last_cpu>5:
        last_cpu=time.time();rec['load']=os.getloadavg()
        rec['top']=subprocess.run(['bash','-c','top -b -n1 -o %CPU | sed -n 8,17p'],capture_output=True,text=True).stdout
    f.write(json.dumps(rec)+'\n');f.flush()
