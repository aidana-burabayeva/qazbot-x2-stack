#!/usr/bin/env python3
"""Existing MC model + exact D1 grid + ideal localization/health fixture.

Does NOT qualify AMCL or LocalizationTrust. Source loss scenarios test action
coordination here; the actual trust algorithm has its own independent tests.
No vendor topics/services or SDK types; used only in isolated domain 87.
"""
import os,sys,time,json,signal
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller_dry'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from e2e_map import MAP_YAML,START
from sim_robot import Sim
import cv2,numpy as np,yaml,rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger

class Fixture(Node):
    def __init__(self,ns):
        super().__init__('ideal_localization_fixture',namespace=ns)
        p=MAP_YAML
        y=yaml.safe_load(p.read_text());a=np.flipud(cv2.imread(str(p.parent/y['image']),0))
        prob=(255-a.astype(float))/255
        data=np.full(a.shape,-1,dtype=np.int8);data[prob<y['free_thresh']]=0;data[prob>y['occupied_thresh']]=100
        m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=float(y['resolution'])
        m.info.width=a.shape[1];m.info.height=a.shape[0];m.info.origin.position.x=float(y['origin'][0]);m.info.origin.position.y=float(y['origin'][1]);m.info.origin.orientation.w=1.;m.data=data.ravel().tolist()
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map=self.create_publisher(OccupancyGrid,'map',q);self.map.publish(m)
        self.ok=self.create_publisher(Bool,'localization_trust/trusted',1)
        self.driver=self.create_publisher(String,'motion_adapter/status',1)
        self.driver_allow=True
        self.create_subscription(Bool,'fixture_driver_ready',lambda m:setattr(self,'driver_allow',m.data),1)
        self.create_service(Trigger,'motion_adapter/arm_motion',self.driver_arm)
        self.last_scan=self.last_odom=float('-inf');self.allow=True
        self.create_subscription(Odometry,'odom',lambda m:setattr(self,'last_odom',time.monotonic()),1)
        self.create_subscription(LaserScan,'scan',lambda m:setattr(self,'last_scan',time.monotonic()),1)
        self.create_subscription(Bool,'fixture_trust',lambda m:setattr(self,'allow',m.data),1)
        self.create_service(Trigger,'localization_trust/arm_check',self.arm)
        self.create_timer(.2,self.tick)
    def arm(self,q,r):
        r.success=self.allow;r.message='ideal localization fixture, not a hardware qualification';return r
    def driver_arm(self,q,r):
        r.success=self.driver_allow;r.message='model driver only, no hardware registration';return r
    def tick(self):
        self.driver.publish(String(data=json.dumps(dict(live=True,registration='registered',
                            ready=self.driver_allow,state='idle',reason='model_fixture'))))
        self.ok.publish(Bool(data=self.allow and time.monotonic()-min(self.last_scan,self.last_odom)<.5))

class TogglePublisher:
    def __init__(self,publisher):self.publisher=publisher;self.enabled=True
    def publish(self,msg):
        if self.enabled:self.publisher.publish(msg)

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='87':raise RuntimeError('model fixture requires isolated domain 87')
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped=[False]
    def terminate(signum,frame):stopped[0]=True
    signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
    ns=sys.argv[1];sim=Sim(ns);sim.x,sim.y,sim.yaw=START;sim.wall_y=START[1]
    sim.odom_pub=TogglePublisher(sim.odom_pub);sim.scan_pub=TogglePublisher(sim.scan_pub)
    sim.create_subscription(Bool,'fixture_odom_enabled',lambda m:setattr(sim.odom_pub,'enabled',m.data),1)
    sim.create_subscription(Bool,'fixture_scan_enabled',lambda m:setattr(sim.scan_pub,'enabled',m.data),1)
    fixture=Fixture(ns);ex=SingleThreadedExecutor();ex.add_node(sim);ex.add_node(fixture)
    try:
        while not stopped[0] and rclpy.ok():ex.spin_once(timeout_sec=.05)
    finally:sim.destroy_node();fixture.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
