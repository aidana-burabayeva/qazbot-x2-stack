#!/usr/bin/env python3
"""Isolated RViz -> real Coordinator/Nav2/Trust/adapter -> SIM -> automatic return.
The optional self-test publishes exactly the same PoseStamped topic as RViz.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

if (os.environ.get('QAZBOT_SIM'),os.environ.get('ROS_DOMAIN_ID'),os.environ.get('ROS_LOCALHOST_ONLY'))!=('1','87','1'):
    raise RuntimeError('isolated SIM only')
ROOT=Path(__file__).resolve().parents[3];NAV=ROOT/'ws/src/navigation'
sys.path[:0]=[str(ROOT/'tests/nav_e2e'),str(NAV)]
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist,PointStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from aimdk_msgs.msg import McLocomotionVelocity
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker,MarkerArray
import loc_trust as L
import pose_store
from loc_model_fixture import LocSim,Toggle

NS='qazbot_sim';START=(-1.275,-4.825,0.)
MAP=ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml'


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def set_pose(p,xyz):
    p.position.x,p.position.y=float(xyz[0]),float(xyz[1]);p.orientation.z,p.orientation.w=math.sin(xyz[2]/2),math.cos(xyz[2]/2)


class RoundTrip(Node):
    def __init__(self,out,backend):
        super().__init__('round_trip',namespace=NS)
        self.out,self.backend=out,backend;self.state='starting';self.home=None;self.home_truth=None
        self.truth=None;self.velocity=None;self.mo=self.odom=None;self.trust={};self.coord={};self.adapter={};self.events=[];self.amcl=None
        self.future=self.goal_handle=None;self.pending_return=False;self.deadline=0.;self.settled=None;self.requested=None;self.outbound_error=None;self.last_command=None
        self.trace=(out/'trace.jsonl').open('w');self.controller_paths={};self.controller_command=None;self.carrot=None
        self.client=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.status=self.create_publisher(String,'~/status',10);self.markers=self.create_publisher(MarkerArray,'twin/markers',10)
        self.seed=self.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        self.goal_pub=self.create_publisher(PoseStamped,'rviz_goal',10)
        self.create_subscription(PoseStamped,'rviz_goal',self.goal,10)
        self.create_subscription(McLocomotionVelocity,'/aima/mc/locomotion/velocity',
            lambda m:setattr(self,'last_command',(m.forward_velocity,m.lateral_velocity,m.angular_velocity)),
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry,'model_truth',self.on_truth,10)
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.on_amcl,10)
        self.create_subscription(TFMessage,'tf',self.on_tf,50)
        for topic in ('nav2/plan','nav2/received_global_plan'):
            self.create_subscription(NavPath,topic,lambda m,k=topic:self.on_path(k,m),10)
        self.create_subscription(Twist,'nav2/cmd_vel_nav',lambda m:setattr(self,'controller_command',(m.linear.x,m.angular.z)),10)
        self.create_subscription(PointStamped,'nav2/lookahead_point',lambda m:setattr(self,'carrot',dict(frame=m.header.frame_id,x=m.point.x,y=m.point.y)),10)
        for topic,key in [('localization_trust/status','trust'),('goal_coordinator/status','coord'),('motion_adapter/status','adapter')]:
            self.create_subscription(String,topic,lambda m,k=key:self.on_status(k,m),10)
        self.stop_client=self.create_client(Trigger,'goal_coordinator/stop')
        self.create_service(Trigger,'~/stop',self.stop)
        self.create_timer(.1,self.tick)
    def on_status(self,key,m):
        value=json.loads(m.data)
        previous=getattr(self,key)
        setattr(self,key,value)
        if key=='trust' and value.get('reasons')!=previous.get('reasons'):
            self.events.append(dict(time=time.time(),event='trust_reasons',reasons=value.get('reasons')))
    def on_truth(self,m):
        p=m.pose.pose;self.truth=(p.position.x,p.position.y,yaw(p.orientation));self.velocity=(m.twist.twist.linear.x,m.twist.twist.angular.z)
    def on_amcl(self,m):
        self.amcl=dict(received=time.time(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                       pose=[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)],
                       covariance=[m.pose.covariance[i] for i in (0,7,35)])
    def on_tf(self,m):
        for t in m.transforms:
            p=t.transform.translation;v=(p.x,p.y,yaw(t.transform.rotation))
            if (t.header.frame_id,t.child_frame_id)==('map','odom'):self.mo=v
            if (t.header.frame_id,t.child_frame_id)==('odom','base_footprint'):self.odom=v
    def pose(self):return None if self.mo is None or self.odom is None else L.compose(self.mo,self.odom)
    def on_path(self,key,m):
        if not m.poses:return
        p=m.poses[-1]
        self.controller_paths[key]=dict(received=time.time(),frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,count=len(m.poses),last=[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)],last_stamp=p.header.stamp.sec+p.header.stamp.nanosec*1e-9)
    def transition(self,state,**detail):
        self.state=state;row=dict(time=time.time(),state=state,**detail);self.events.append(row)
        (self.out/'journey.json').write_text(json.dumps(dict(backend=self.backend,map=str(MAP),home=self.home,home_truth=self.home_truth,events=self.events),indent=2)+'\n')
        self.get_logger().info(json.dumps(row))
    def goal(self,m):
        if self.state not in ('ready','complete','outbound_failed','return_failed','completed_outside_tolerance'):
            self.get_logger().warn('Goal ignored: journey already active or stack not ready');return
        if m.header.frame_id!='map' or not self.coord.get('ready') or self.coord.get('busy') or self.coord.get('latched'):
            self.get_logger().warn('Goal ignored: Coordinator is not ready');return
        self.requested=(m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation))
        self.send(m,'outbound')
    def send(self,m,leg):
        g=NavigateToPose.Goal();g.pose=m
        self.transition(leg,goal=[m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation)],truth=self.truth)
        future=self.client.send_goal_async(g)
        future.add_done_callback(lambda f:self.accepted(f,leg))
    def accepted(self,f,leg):
        if self.state!=leg:return
        try:self.goal_handle=f.result()
        except Exception as e:self.transition(leg+'_failed',error=str(e));return
        if not self.goal_handle.accepted:self.transition(leg+'_failed',status='rejected');return
        self.future=self.goal_handle.get_result_async();self.future.add_done_callback(lambda f:self.finished(f,leg))
    def finished(self,f,leg):
        if self.state!=leg:return
        try:status=f.result().status
        except Exception as e:self.transition(leg+'_failed',error=str(e));return
        if status!=4:self.transition(leg+'_failed',status=status,truth=self.truth);return
        if leg=='outbound':
            self.pending_return=True;self.deadline=time.monotonic()+12.;self.settled=None
            self.transition('settling',truth=self.truth)
        else:
            self.pending_return=True;self.deadline=time.monotonic()+12.;self.settled=None
            self.transition('settling_return',truth=self.truth)
    def stop(self,req,resp):
        self.pending_return=False;self.transition('stopped')
        if self.stop_client.service_is_ready():self.stop_client.call_async(Trigger.Request())
        resp.success=True;resp.message='Journey stopped; no automatic replay';return resp
    def ready(self):
        self.home=tuple(self.truth);self.home_truth=tuple(self.truth);self.transition('ready',home=self.home,home_truth=self.home_truth,estimated_pose=self.pose())
    def tick(self):
        now=time.monotonic()
        if self.pending_return:
            resting=self.velocity is not None and abs(self.velocity[0])<.02 and abs(self.velocity[1])<.03
            if resting and not self.coord.get('busy') and self.coord.get('ready') and self.trust.get('motion_ok'):
                self.settled=now if self.settled is None else self.settled
                if now-self.settled>.6:
                    self.pending_return=False
                    if self.state=='settling_return':
                        p=self.truth;h=self.home_truth
                        xy=math.hypot(p[0]-h[0],p[1]-h[1]);angle=abs(L.wrap(p[2]-h[2]))
                        precise=xy<=.10 and angle<=.10 and max(self.outbound_error)<=.10
                        self.transition('complete' if precise else 'completed_outside_tolerance',truth=p,
                                        xy_error=xy,yaw_error=angle,outbound_error=self.outbound_error)
                    else:
                        p=self.truth;g=self.requested
                        self.outbound_error=(math.hypot(p[0]-g[0],p[1]-g[1]),abs(L.wrap(p[2]-g[2])))
                        self.events.append(dict(time=time.time(),state='outbound_settled',truth=p,xy_error=self.outbound_error[0],yaw_error=self.outbound_error[1]))
                        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg();set_pose(p.pose,self.home);self.send(p,'return')
            else:self.settled=None
            if self.pending_return and now>self.deadline:self.pending_return=False;self.transition('return_failed',error='stack did not settle / become ready')
        self.status.publish(String(data=json.dumps(dict(state=self.state,home=self.home,truth=self.truth,backend=self.backend,ready=self.coord.get('ready')))))
        self.draw()
        self.trace.write(json.dumps(dict(t=time.time(),state=self.state,truth=self.truth,estimate=self.pose(),amcl=self.amcl,velocity=self.velocity,mc_command=self.last_command,controller_command=self.controller_command,controller_paths=self.controller_paths,carrot=self.carrot,coordinator_fault=self.coord.get('fault'),trust_reasons=self.trust.get('reasons'),trust_stamp_rejections=self.trust.get('stamp_rejections'),trust_input_debug={k:self.trust.get(k) for k in ('input_ages','input_callbacks','scan_frame','scan_frame_rejections','scan_points')},plant_scan=getattr(getattr(self,'plant',None),'scan_health',None),collision=getattr(getattr(self,'plant',None),'last_collision',None)))+'\n');self.trace.flush()
    def draw(self):
        marks=[];stamp=self.get_clock().now().to_msg()
        def marker(i,typ,frame):
            m=Marker();m.header.stamp=stamp;m.header.frame_id=frame;m.ns='twin';m.id=i;m.type=typ;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=1.;return m
        body=marker(0,Marker.CUBE,'base_footprint');body.pose.position.z=.2;body.scale.x=.6;body.scale.y=.42;body.scale.z=.4;body.color.g=.65;body.color.b=.9;marks.append(body)
        arrow=marker(1,Marker.ARROW,'base_footprint');arrow.pose.position.z=.45;arrow.scale.x=.65;arrow.scale.y=.1;arrow.scale.z=.1;arrow.color.r=1.;marks.append(arrow)
        text=marker(2,Marker.TEXT_VIEW_FACING,'base_footprint');text.pose.position.z=.8;text.scale.z=.22;text.color.r=text.color.g=text.color.b=1.;labels={'starting':'Запуск','ready':'Готов','outbound':'К цели','settling':'Остановка','return':'Возврат','settling_return':'Остановка на старте','complete':'Вернулся','completed_outside_tolerance':'Вернулся: проверить ошибку','outbound_failed':'Цель не достигнута','return_failed':'Возврат не выполнен','stopped':'Остановлен'}
        text.text=labels.get(self.state,self.state);marks.append(text)
        if self.home is not None:
            home=marker(3,Marker.ARROW,'map');set_pose(home.pose,self.home);home.pose.position.z=.08;home.scale.x=.7;home.scale.y=.13;home.scale.z=.13;home.color.g=1.;marks.append(home)
        envelope=marker(4,Marker.CYLINDER,'base_footprint');envelope.pose.position.z=.02;envelope.scale.x=envelope.scale.y=.908;envelope.scale.z=.025;envelope.color.g=.8;envelope.color.b=1.;envelope.color.a=.25;marks.append(envelope)
        if self.requested is not None:
            target=marker(5,Marker.ARROW,'map');set_pose(target.pose,self.requested);target.pose.position.z=.1;target.scale.x=.6;target.scale.y=.1;target.scale.z=.1;target.color.r=target.color.g=1.;marks.append(target)
        self.markers.publish(MarkerArray(markers=marks))


class FrontScan:
    """Only the measured .66 front wedge/rate, with ideal map rays and pelvis origin.
    No fabricated returns behind the sensor; no simulated raw FAST-LIO or scan dropout calibration.
    """
    def init_scan(self):
        from sensor_model import WedgeRays
        self.scan_config=json.loads((ROOT/'ws/tools/twin/profiles/robot66_observation_v1.json').read_text())
        self.rays=WedgeRays(self.scan_config);self.last_scan=float('-inf')
        self.scan_timer.cancel()
        self.scan_timer=self.create_timer(1/self.scan_config['model']['scan_hz'],self.pub_scan)
    def pub_scan(self):
        now=time.monotonic()
        if not hasattr(self,'rays') or not self.enabled.get('scan',True):return
        if hasattr(self,'p0') and self.p0 is None:return
        stamp=self.sample_stamp() if hasattr(self,'sample_stamp') else self.get_clock().now().to_msg()
        if stamp is None:return
        g=self.scan_config['scan'];m=LaserScan();m.header.frame_id='base_footprint';m.header.stamp=stamp
        m.angle_min=g['angle_min'];m.angle_increment=g['angle_increment'];m.angle_max=m.angle_min+(g['bins']-1)*m.angle_increment
        m.range_min=g['range_min'];m.range_max=g['range_max']
        m.ranges=self.rays.cast((self.x,self.y,self.yaw),self.grid.dist_at).astype(np.float32).tolist()
        self.scan_pub.publish(m)
        finished=time.monotonic()
        self.scan_health=dict(source_stamp=stamp.sec+stamp.nanosec*1e-9,started=now,finished=finished,compute_s=finished-now,interval_s=None if not math.isfinite(self.last_scan) else now-self.last_scan)
        self.last_scan=now


class KinematicPlant(FrontScan,LocSim):
    def __init__(self,grid,motion_grid):
        super().__init__(NS,grid=grid);self.init_scan();self.motion_grid=motion_grid;self.last_collision=None;self.x,self.y,self.yaw=START;self.publish_map_odom=False
        # Stage-like ideal velocity plant: explicitly no MC gain, start threshold or lag.
        self.tau=self.start_vx=self.start_wz=0.
        self.truth=self.create_publisher(Odometry,'model_truth',10);self.odom_pub=Toggle(self.odom_pub,lambda:True,self.truth)
    def create_subscription(self,typ,topic,callback,qos,*args,**kwargs):
        if topic=='cmd_vel':topic='mc_plant_cmd'
        return super().create_subscription(typ,topic,callback,qos,*args,**kwargs)
    def step(self):
        dt=max(0.,time.monotonic()-self.last)
        cv,cw=self.cmd if time.monotonic()-self.cmd_t<=.2 else (0.,0.)
        if not self.moving and abs(cv)<self.start_vx and abs(cw)<self.start_wz:cv=cw=0.
        alpha=1. if self.tau<=0 else 1-math.exp(-dt/self.tau);v=self.v+(cv-self.v)*alpha;w=self.w+(cw-self.w)*alpha
        h=self.yaw+w*dt;x,y=self.x+v*math.cos(h)*dt,self.y+v*math.sin(h)*dt
        g=self.motion_grid
        inside=g.ox<=x<g.ox+g.w*g.res and g.oy<=y<g.oy+g.h*g.res
        clearance=g.dist_at(np.array([[x,y]]))[0] if inside else 0.
        self.last_collision=None
        if clearance<.454:
            self.last_collision=dict(candidate=[x,y],clearance=float(clearance))
            self.cmd=(0.,cw);self.v=0.
        super().step()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--plant',choices=['official','kinematic'],default='kinematic');ap.add_argument('--no-rviz',action='store_true');tests=ap.add_mutually_exclusive_group();tests.add_argument('--test-goal',type=float);tests.add_argument('--test-pose',nargs=3,type=float);tests.add_argument('--test-turn-deg',type=float);ap.add_argument('--name',default=time.strftime('interactive_%Y%m%d_%H%M%S'))
    ap.add_argument('--goal-checker-stateful',choices=['true','false'],default='false')
    # Validated for this ideal front-scan stand; production AMCL remains separate.
    ap.add_argument('--amcl-update-min-d',type=float,default=.05);ap.add_argument('--amcl-update-min-a',type=float,default=.05)
    ap.add_argument('--amcl-motion-noise',type=float);ap.add_argument('--amcl-max-beams',type=int);ap.add_argument('--amcl-z-hit',type=float);ap.add_argument('--amcl-likelihood-max-dist',type=float)
    ap.add_argument('--amcl-transform-tolerance',type=float)
    ap.add_argument('--amcl-sigma-hit',type=float)
    ap.add_argument('--amcl-laser-model',choices=['likelihood_field','likelihood_field_prob'])
    ap.add_argument('--local-frame',choices=['odom','map'],default='odom')
    ap.add_argument('--test-timeout',type=float,default=180.)
    ap.add_argument('--xy-tol',type=float,default=.05);ap.add_argument('--yaw-tol',type=float,default=.06)
    args=ap.parse_args()
    if not math.isfinite(args.test_timeout) or args.test_timeout<=0:ap.error('test timeout must be positive')
    overrides={k:v for k,v in dict(update_min_d=args.amcl_update_min_d,update_min_a=args.amcl_update_min_a,max_beams=args.amcl_max_beams,z_hit=args.amcl_z_hit,laser_likelihood_max_dist=args.amcl_likelihood_max_dist,transform_tolerance=args.amcl_transform_tolerance,sigma_hit=args.amcl_sigma_hit,laser_model_type=args.amcl_laser_model).items() if v is not None}
    if any(not math.isfinite(v) or v<=0 for v in overrides.values() if not isinstance(v,str)):ap.error('AMCL numeric parameters must be positive')
    if args.amcl_motion_noise is not None:
        if not math.isfinite(args.amcl_motion_noise) or args.amcl_motion_noise<0:ap.error('motion noise must be finite and nonnegative')
        overrides.update({f'alpha{i}':args.amcl_motion_noise for i in range(1,6)})
    out=ROOT/'tests/integration_results'/args.name;out.mkdir();procs=[];logs=[];stop=[False];rc=1
    def start(name,cmd):
        f=(out/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,p));return p
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,lambda *a:stop.__setitem__(0,True));signal.signal(signal.SIGTERM,lambda *a:stop.__setitem__(0,True))
    ex=SingleThreadedExecutor();n=RoundTrip(out,args.plant);ex.add_node(n);plant=None
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if stop[0]:raise KeyboardInterrupt()
            dead=[(name,p.returncode) for name,p in procs if p.poll() is not None]
            if dead:raise RuntimeError('child exited: '+str(dead))
            ex.spin_once(timeout_sec=.02)
    def wait(pred,seconds=60):
        end=time.monotonic()+seconds
        while not pred():
            if time.monotonic()>end:raise RuntimeError('deadline '+json.dumps(dict(state=n.state,trust=n.trust.get('reasons'),coord=n.coord,adapter=n.adapter)))
            spin(.05)
    def call(client):
        wait(client.service_is_ready);f=client.call_async(Trigger.Request());wait(f.done);return f.result()
    try:
        if args.plant=='official':
            start('frames',['ros2','bag','record','-o',str(out/'frames'),'/tf','/tf_static',
                '/aima/hal/odom/state','/aima/mc/common/state','/aima/mc/locomotion/velocity',
                f'/{NS}/tf',f'/{NS}/model_truth',f'/{NS}/odom',f'/{NS}/amcl_pose',
                f'/{NS}/scan',f'/{NS}/localization_trust/status',f'/{NS}/cmd_vel',f'/{NS}/cmd_vel_authorized',f'/{NS}/nav2/cmd_vel_nav',
                f'/{NS}/rviz_goal',f'/{NS}/round_trip/status'])
        grid=L.OccGrid.from_pgm(str(MAP))
        if args.plant=='kinematic':
            import cv2,yaml
            y=yaml.safe_load(MAP.read_text());a=np.flipud(cv2.imread(str(MAP.parent/y['image']),0));blocked=(255-a.astype(float))/255>=y['free_thresh']
            # Unknown is a planning constraint, not a fictitious physical wall.
            motion=L.OccGrid.from_pgm(str(MAP));motion.dist[[0,-1],:]=0;motion.dist[:,[0,-1]]=0
            plant=KinematicPlant(grid,motion)
            start('virtual_mc',['python3',str(ROOT/'ws/tools/twin/virtual_mc.py'),f'/{NS}/mc_plant_cmd'])
        else:
            from mc_plant_bridge import HalPlant
            manifest=json.loads((ROOT/'runtime_x86/rviz_map_v3/configuration/manifest.json').read_text())
            if manifest['image_sha256']!=pose_store.map_image_sha256(str(MAP)) or manifest['start_map']!=list(START):raise RuntimeError('map/world transform mismatch')
            expected=os.path.normpath(str(ROOT/'runtime_x86/rviz_map_v3/configuration/robot/lx2501_3_t2d5/simulator/default.yaml'))
            running=[]
            for cmdfile in Path('/proc').glob('[0-9]*/cmdline'):
                try:argv=cmdfile.read_bytes().decode().split('\0')
                except (OSError,UnicodeError):continue
                if argv and Path(argv[0]).name=='aima-sim-app':
                    running.extend(os.path.normpath(a.split('=',1)[1]) for a in argv if a.startswith('--cfg_file_path='))
            if running!=[expected]:raise RuntimeError('sim process uses wrong world configuration: '+str(running))
            class OfficialFrontPlant(FrontScan,HalPlant):pass
            plant=OfficialFrontPlant(NS,grid=grid,start=START,fixed_world=True);plant.init_scan()
        n.plant=plant;ex.add_node(plant);wait(lambda:n.truth is not None)
        start('amcl',['ros2','launch',str(ROOT/'ws/tools/twin/amcl_sim.launch.py'),f'ns:={NS}',f'scan_topic:=/{NS}/scan',f'map_yaml:={MAP}'])
        start('trust',['python3',str(NAV/'localization_trust_node.py'),'--ros-args','-r',f'__ns:=/{NS}',
            '-r',f'/qazbot_d1_map/tf:=/{NS}/tf','-r',f'/qazbot_lio/odom_level:=/{NS}/odom','-r',f'/qazbot_lio/scan:=/{NS}/scan',
            '-p',f'map_yaml:={MAP}','-p',f'expected_map_sha256:={pose_store.map_image_sha256(str(MAP))}'])
        start('adapter',['python3',str(NAV/'x2_motion_adapter.py'),'--ros-args','-r',f'__ns:=/{NS}',
            '-p','live:=true','-p',f'lock_path:={out}/adapter.lock','-p',f'cmd_vel_topic:=/{NS}/cmd_vel_authorized',
            '-p',f'tf_topic:=/{NS}/tf','-p',f'trust_topic:=/{NS}/localization_trust/trusted'])
        wait(lambda:n.trust.get('map_ok')=='' and n.adapter)
        from lifecycle_msgs.srv import GetState
        active=n.create_client(GetState,'amcl/get_state');wait(active.service_is_ready)
        while True:
            f=active.call_async(GetState.Request());wait(f.done)
            if f.result().current_state.id==3:break
            spin(.1)
        if overrides:
            from rcl_interfaces.srv import SetParameters
            from rclpy.parameter import Parameter
            client=n.create_client(SetParameters,'amcl/set_parameters');wait(client.service_is_ready)
            req=SetParameters.Request();req.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in overrides.items()]
            f=client.call_async(req);wait(f.done)
            if not all(r.successful for r in f.result().results):raise RuntimeError('AMCL experiment rejected '+str(f.result()))
        (out/'experiment.json').write_text(json.dumps(dict(dds_udp_only=os.environ.get('QAZBOT_SIM_DDS_UDP_ONLY')=='1',mc_linear_decel=os.environ.get('QAZBOT_SIM_MC_LINEAR_DECEL'),mc_walk_candidate=os.environ.get('QAZBOT_SIM_MC_WALK')=='1',mc_walk_lead=float(os.environ.get('QAZBOT_SIM_MC_WALK_LEAD','.45')),exact_goal_candidate=os.environ.get('QAZBOT_SIM_EXACT_GOAL')=='1',pose_refine_candidate=os.environ.get('QAZBOT_SIM_POSE_REFINE')=='1',mc_turn_candidate=os.environ.get('QAZBOT_SIM_MC_TURN')=='1',mc_turn_command_cap=.8 if os.environ.get('QAZBOT_SIM_MC_TURN')=='1' else .5,local_frame=args.local_frame,goal_checker_stateful=False if os.environ.get('QAZBOT_SIM_MC_TURN')=='1' else args.goal_checker_stateful=='true',xy_tolerance=args.xy_tol,yaw_tolerance=args.yaw_tol,amcl_overrides=overrides),indent=2))
        # Known virtual spawn only: seed exactly the simulated truth, never guess hardware pose.
        seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.header.stamp=n.get_clock().now().to_msg();set_pose(seed.pose.pose,n.truth)
        # The simulator spawn is known exactly; this is not a physical localization hypothesis.
        seed.pose.covariance[0]=seed.pose.covariance[7]=.0025;seed.pose.covariance[35]=.0004;n.seed.publish(seed)
        wait(lambda:n.trust.get('motion_ok') is True,15)
        start('nav2',['ros2','launch',str(NAV/'nav_full.launch.py'),f'ns:={NS}',f'map_yaml:={MAP}','robot_radius:=0.454',
            'max_angular:=0.5','autostart:=false',f'local_frame:={args.local_frame}',f'xy_tol:={args.xy_tol}',f'yaw_tol:={args.yaw_tol}',f'goal_checker_stateful:={args.goal_checker_stateful}',f'scan_topic:=/{NS}/scan',f'odom_topic:=/{NS}/odom','allow_dry_run_driver:=false'])
        # Let lifecycle service reply readers be discovered before requesting
        # configure/activate; otherwise Fast DDS may lose the very first response.
        from nav2_msgs.srv import ManageLifecycleNodes
        manager=n.create_client(ManageLifecycleNodes,'nav2/lifecycle_manager_navigation/manage_nodes')
        wait(manager.service_is_ready);spin(1.)
        request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.STARTUP
        future=manager.call_async(request);wait(future.done,30)
        if not future.result().success:raise RuntimeError('Nav2 lifecycle startup failed')
        response=call(n.create_client(Trigger,'motion_adapter/register'))
        if not response.success:raise RuntimeError(response.message)
        wait(lambda:n.coord.get('ready') and n.client.server_is_ready() and n.pose() is not None,120);spin(1);n.ready()
        if not args.no_rviz:start('rviz',['rviz2','-d',str(ROOT/'ws/tools/twin/interactive.rviz'),'--ros-args','-r',f'/tf:=/{NS}/tf','-r',f'/tf_static:=/{NS}/tf_static'])
        print('READY: RViz 2D Goal Pose -> outbound -> automatic return; SIM only; plant='+args.plant,flush=True)
        if args.test_goal is not None or args.test_pose is not None or args.test_turn_deg is not None:
            target=(n.home[0],n.home[1],L.wrap(n.home[2]+math.radians(args.test_turn_deg))) if args.test_turn_deg is not None else (args.test_pose if args.test_pose is not None else (n.home[0]+args.test_goal,n.home[1],n.home[2]))
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();set_pose(p.pose,target);n.goal_pub.publish(p)
            wait(lambda:n.state in ('complete','outbound_failed','return_failed','completed_outside_tolerance'),args.test_timeout)
            rc=0 if n.state in ('complete','completed_outside_tolerance') else 1;spin(1)
        else:
            while not stop[0]:spin(.1)
            rc=0
    except KeyboardInterrupt:n.transition('stopped');rc=0
    finally:
        if n.stop_client.service_is_ready():
            future=n.stop_client.call_async(Trigger.Request());end=time.monotonic()+1
            while not future.done() and time.monotonic()<end:ex.spin_once(timeout_sec=.02)
        codes={}
        for _,p in procs:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for name,p in reversed(procs):
            try:codes[name]=p.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);codes[name]=p.wait(timeout=5)
        for f in logs:f.close()
        from process_exit_audit import audit_process_exits
        process_audit=audit_process_exits(out,codes)
        if not process_audit['clean_shutdown']:rc=1
        (out/'session.json').write_text(json.dumps(dict(state=n.state,workflow_succeeded=n.state in ('complete','completed_outside_tolerance'),within_ground_truth_tolerance=n.state=='complete',backend=args.plant,clean_shutdown=process_audit['clean_shutdown'],native_process_failures=process_audit['native_process_failures'],scan_geometry=dict(fov_deg=103.,front_only=True,bins=1440,range_max_m=8.,origin='pelvis',ideal_rays=True),map_sha256=pose_store.map_image_sha256(str(MAP)),shutdown=codes,home=n.home,home_truth=n.home_truth,truth=n.truth),indent=2))
        ex.shutdown();n.trace.close();n.destroy_node()
        if plant is not None:plant.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__=='__main__':sys.exit(main())
