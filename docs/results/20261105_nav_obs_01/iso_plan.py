"""Isolated planner harness: REAL nav2 planner_server (Smac2D, qualified overlay) + global costmap with the production parameters, domain 90,
loopback only, no robot connection. Map = occupied cells of the saved costmap (walls + box). Plans start->goal for several global inflation radii."""
import os,sys,json,math,time,subprocess,yaml,tempfile,numpy as np
os.environ['ROS_DOMAIN_ID']='90';os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']='/home/agi/control_backend/ws/src/navigation/dds_loopback.xml'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav2_msgs.action import ComputePathToPose
from tf2_ros import StaticTransformBroadcaster
Z=np.load('costmap_box.npz');A=Z['global_data'];RES=float(Z['global_res']);ORG=Z['global_org']
box=json.load(open('boxcheck_box.json'));x0,y0,yw=box['start'];gx,gy,_=box['goal']
FOOT='[[0.17, 0.229], [0.008, 0.34], [-0.182, 0.34], [-0.305, 0.229], [-0.305, -0.261], [-0.182, -0.372], [0.008, -0.372], [0.17, -0.261]]'
margin=0.05/math.sqrt(2)
P={'/iso/planner_server':{'ros__parameters':{'use_sim_time':False,'planner_plugins':['GridBased'],'GridBased.plugin':'nav2_smac_planner/SmacPlanner2D',
  'GridBased.allow_unknown':False,'GridBased.tolerance':0.3,'GridBased.cost_travel_multiplier':4.0}},
 '/iso/global_costmap/global_costmap':{'ros__parameters':{'use_sim_time':False,'global_frame':'map','robot_base_frame':'base_footprint',
  'footprint':FOOT,'footprint_padding':0.01+margin,'track_unknown_space':True,'plugins':['static_layer','inflation_layer'],
  'static_layer.plugin':'nav2_costmap_2d::StaticLayer','static_layer.map_topic':'/iso/map','static_layer.map_subscribe_transient_local':True,
  'static_layer.footprint_clearing_enabled':True,'inflation_layer.plugin':'nav2_costmap_2d::InflationLayer',
  'inflation_layer.inflation_radius':0.60,'inflation_layer.cost_scaling_factor':1.5,'inflation_layer.inflate_around_unknown':True}}}
pf=tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=False);yaml.safe_dump(P,pf);pf.close()
env=dict(os.environ)
procs=[]
def sh(cmd):
    return subprocess.Popen(['bash','-c','export QAZBOT_SMAC_PREFIX=/home/agi/control_backend/candidates/nav2_smac_bounds/install/nav2_smac_planner; source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/ws/src/navigation/dds_loopback.xml; '+cmd],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,preexec_fn=os.setsid)
procs.append(sh('ros2 run nav2_planner planner_server --ros-args -r __node:=planner_server -r __ns:=/iso --params-file %s'%pf.name))
procs.append(sh('ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lm -r __ns:=/iso -p autostart:=true -p node_names:="[planner_server]"'))
rclpy.init();n=Node('iso')
m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=RES;m.info.width=A.shape[1];m.info.height=A.shape[0]
m.info.origin.position.x=float(ORG[0]);m.info.origin.position.y=float(ORG[1]);m.info.origin.orientation.w=1.0
if os.environ.get('NOBOX')=='1':
    ci=int((-4.19-ORG[1])/RES);cj=int((-8.62-ORG[0])/RES);A=A.copy();A[ci-6:ci+7,cj-8:cj+9]=np.where(A[ci-6:ci+7,cj-8:cj+9]==254,0,A[ci-6:ci+7,cj-8:cj+9])
grid=np.zeros(A.shape,dtype=np.int8);grid[A==254]=100;grid[A==255]=-1;m.data=grid.flatten().tolist()
pub=n.create_publisher(OccupancyGrid,'/iso/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
sb=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';t.transform.translation.x=x0;t.transform.translation.y=y0
t.transform.rotation.z=math.sin(yw/2);t.transform.rotation.w=math.cos(yw/2);t.header.stamp=n.get_clock().now().to_msg();sb.sendTransform(t)
m.header.stamp=n.get_clock().now().to_msg();pub.publish(m)
cl=ActionClient(n,ComputePathToPose,'/iso/compute_path_to_pose')
end=time.time()+60
while not cl.wait_for_server(timeout_sec=1.0) and time.time()<end: rclpy.spin_once(n,timeout_sec=0.1)
def setp(radius=None,scaling=None):
    c=n.create_client(SetParameters,'/iso/global_costmap/global_costmap/set_parameters');c.wait_for_service(10)
    ps=[]
    for name,v in (('inflation_layer.inflation_radius',radius),('inflation_layer.cost_scaling_factor',scaling)):
        if v is None: continue
        p=Parameter();p.name=name;p.value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(v));ps.append(p)
    r=SetParameters.Request();r.parameters=ps;f=c.call_async(r)
    while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
    return all(x.successful for x in f.result().results)
def ps_(x,y,w):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(w/2);p.pose.orientation.w=math.cos(w/2);return p
def planit():
    g=ComputePathToPose.Goal();g.start=ps_(x0,y0,yw);g.goal=ps_(gx,gy,yw);g.use_start=True;g.planner_id='GridBased'
    f=cl.send_goal_async(g)
    while not f.done(): rclpy.spin_once(n,timeout_sec=0.05)
    h=f.result();rf=h.get_result_async()
    while not rf.done(): rclpy.spin_once(n,timeout_sec=0.05)
    rr=rf.result()
    return [(p.pose.position.x,p.pose.position.y) for p in rr.result.path.poses] if rr.status==4 else None
def spin_for(s):
    e=time.time()+s
    while time.time()<e: rclpy.spin_once(n,timeout_sec=0.05)
results={}
try:
    spin_for(4)
    variants=json.loads(sys.argv[1]) if len(sys.argv)>1 else [[0.6,1.5]]
    for R,cs in variants:
        ok=setp(R,cs);spin_for(3)
        pth=None
        for _ in range(3):
            pth=planit()
            if pth: break
            spin_for(1)
        results['%.2f_%.2f'%(R,cs)]=pth
        print('radius %.2f scaling %.2f set=%s plan %s'%(R,cs,ok,('%d poses'%len(pth)) if pth else None),flush=True)
finally:
    import signal
    for p in procs:
        try: os.killpg(os.getpgid(p.pid),signal.SIGINT)
        except Exception: pass
    json.dump(results,open(os.environ.get('ISO_OUT','iso_plans.json'),'w'))
