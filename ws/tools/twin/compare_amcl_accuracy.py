#!/usr/bin/env python3
"""SIM ONLY: identical truth-derived odometry/front scans for independent real AMCLs.

This isolates localization parameters from navigation trajectory changes. Truth is used
to generate sensor inputs and score outputs, never to correct an AMCL after its seed.
"""
import argparse,copy,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):raise RuntimeError('isolated SIM only')
ROOT=Path(__file__).resolve().parents[3];sys.path[:0]=[str(ROOT/'ws/src/navigation'),str(ROOT/'ws/tools/twin')]
import loc_trust as L
from sensor_model import WedgeRays
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty

ap=argparse.ArgumentParser();ap.add_argument('trace',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--refine',action='store_true');ap.add_argument('--probability-model',action='store_true');args=ap.parse_args()
args.output.mkdir();source=[json.loads(s) for s in args.trace.read_text().splitlines()]
source=[r for r in source if r.get('truth')];ts=np.array([r['t']-source[0]['t'] for r in source]);poses=np.array([r['truth'] for r in source]);poses[:,2]=np.unwrap(poses[:,2])
profile=json.loads((ROOT/'ws/tools/twin/profiles/robot66_observation_v1.json').read_text());rays=WedgeRays(profile);grid=L.OccGrid.from_pgm(str(ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml'))
variants=dict(baseline={},updates=dict(update_min_d=.05,update_min_a=.05),beams60=dict(max_beams=60),reference_sensor=dict(z_hit=.9,laser_likelihood_max_dist=4.))
if args.refine:
    variants=dict(updates=dict(update_min_d=.05,update_min_a=.05),sigma010=dict(update_min_d=.05,update_min_a=.05,sigma_hit=.1),sigma005=dict(update_min_d=.05,update_min_a=.05,sigma_hit=.05),updates_ref=dict(update_min_d=.05,update_min_a=.05,z_hit=.9,laser_likelihood_max_dist=4.))
if args.probability_model:
    variants=dict(updates=dict(update_min_d=.05,update_min_a=.05),prob1440=dict(update_min_d=.05,update_min_a=.05,laser_model_type='likelihood_field_prob'),prob60=dict(update_min_d=.05,update_min_a=.05,laser_model_type='likelihood_field_prob',max_beams=60),prob360=dict(update_min_d=.05,update_min_a=.05,laser_model_type='likelihood_field_prob',max_beams=360))
rclpy.init();n=Node('amcl_accuracy_replay');processes=[];logs=[];publishers={};rows={k:[] for k in variants};stamp_truth={};last_estimate={};start=None
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if any(p.poll() is not None for p in processes):raise RuntimeError('AMCL process exited')
        rclpy.spin_once(n,timeout_sec=.005)
def call(client,request,timeout=40):
    end=time.monotonic()+timeout
    while not client.service_is_ready():
        if time.monotonic()>end:raise RuntimeError('service unavailable '+client.srv_name)
        spin(.05)
    spin(.5)  # Service discovery can precede discovery of the reply reader in Fast DDS.
    while time.monotonic()<end:
        future=client.call_async(request);attempt=time.monotonic()+3
        while not future.done() and time.monotonic()<attempt:spin(.01)
        if future.done():return future.result()
        client.remove_pending_request(future)
    raise RuntimeError('service timed out '+client.srv_name)
def receive(key,m):
    stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    if stamp not in stamp_truth:return
    t,truth,state=stamp_truth[stamp];p=m.pose.pose
    pose=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]
    row=dict(t=t,state=state,truth=truth,pose=pose,xy_error=math.dist(pose[:2],truth[:2]),yaw_error=abs(L.wrap(pose[2]-truth[2])),covariance=[m.pose.covariance[i] for i in (0,7,35)])
    rows[key].append(row);last_estimate[key]=row
def publish(t,scan=True):
    pose=[float(np.interp(t,ts,poses[:,i])) for i in range(3)];idx=min(len(source)-1,int(np.searchsorted(ts,t)))
    stamp=n.get_clock().now().to_msg();stamp_truth[stamp.sec*10**9+stamp.nanosec]=(t,pose,source[idx]['state'])
    transform=TransformStamped();transform.header.stamp=stamp;transform.header.frame_id='odom';transform.child_frame_id='base_footprint';transform.transform.translation.x,transform.transform.translation.y=pose[:2];transform.transform.rotation.z,transform.transform.rotation.w=math.sin(pose[2]/2),math.cos(pose[2]/2)
    msg=None
    if scan:
        msg=LaserScan();msg.header.stamp=stamp;msg.header.frame_id='base_footprint';g=profile['scan'];msg.angle_min=g['angle_min'];msg.angle_increment=g['angle_increment'];msg.angle_max=g['angle_min']+(g['bins']-1)*g['angle_increment'];msg.range_min=g['range_min'];msg.range_max=g['range_max'];msg.ranges=rays.cast(pose,grid.dist_at).astype(np.float32).tolist()
    for pubs in publishers.values():
        pubs['tf'].publish(TFMessage(transforms=[transform]))
        if scan:pubs['scan'].publish(msg)
try:
    for key,params in variants.items():
        ns='accuracy_'+key
        pubs=dict(tf=n.create_publisher(TFMessage,f'/{ns}/tf',20),scan=n.create_publisher(LaserScan,f'/{ns}/scan',10),seed=n.create_publisher(PoseWithCovarianceStamped,f'/{ns}/initialpose',10))
        publishers[key]=pubs;n.create_subscription(PoseWithCovarianceStamped,f'/{ns}/amcl_pose',lambda m,k=key:receive(k,m),10)
        log=(args.output/(key+'.log')).open('w');logs.append(log)
        processes.append(subprocess.Popen(['ros2','launch',str(ROOT/'ws/tools/twin/amcl_sim.launch.py'),f'ns:={ns}',f'scan_topic:=/{ns}/scan',f'map_yaml:={ROOT}/ws/src/maps/first_map_nav2_v3/map.yaml'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        client=n.create_client(GetState,f'/{ns}/amcl/get_state');deadline=time.monotonic()+60
        while call(client,GetState.Request()).current_state.id!=3:
            if time.monotonic()>deadline:raise RuntimeError('not active '+key)
            spin(.1)
        if params:
            result=call(n.create_client(SetParameters,f'/{ns}/amcl/set_parameters'),SetParameters.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in params.items()]))
            if not all(r.successful for r in result.results):raise RuntimeError(str(result))
    for _ in range(25):publish(0);spin(.04)
    seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.header.stamp=n.get_clock().now().to_msg();seed.pose.pose.position.x,seed.pose.pose.position.y=poses[0,:2];seed.pose.pose.orientation.z,seed.pose.pose.orientation.w=math.sin(poses[0,2]/2),math.cos(poses[0,2]/2);seed.pose.covariance[0]=seed.pose.covariance[7]=.0025;seed.pose.covariance[35]=.0004
    for pubs in publishers.values():pubs['seed'].publish(seed)
    for _ in range(25):publish(0);spin(.04)
    for key in variants:call(n.create_client(Empty,f'/accuracy_{key}/request_nomotion_update'),Empty.Request())
    for _ in range(10):publish(0);spin(.04)
    print('REPLAY READY '+str(variants),flush=True)
    start=time.monotonic();next_scan=0.;next_tf=0.
    while time.monotonic()-start<ts[-1]+2:
        t=time.monotonic()-start
        if t>=next_tf:
            scan=t>=next_scan;publish(t,scan=scan);next_tf=t+.02
            if scan:next_scan=t+.2
        spin(.002)
    spin(.3)
    summary={}
    for key,data in rows.items():
        (args.output/(key+'.json')).write_text(json.dumps(data))
        errors=[r['xy_error'] for r in data if r['t']>1]
        endpoint=[r for r in data if r['state'] in ('settling','return') and r['t']>1]
        summary[key]=dict(parameters=variants[key],samples=len(data),xy_p95_m=float(np.quantile(errors,.95)),xy_mean_m=float(np.mean(errors)),outbound_last=[r for r in data if r['state'] in ('outbound','settling')][-1],final=data[-1])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
finally:
    for p in processes:
        if p.poll() is None:p.send_signal(signal.SIGINT)
    for p in processes:
        try:p.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    for log in logs:log.close()
    n.destroy_node();rclpy.try_shutdown()
