#!/usr/bin/env python3
"""Real Trust restart stress, private synthetic inputs; no adapter, Nav2 or MC commands."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
if any(os.environ.get(k)!=v for k,v in dict(QAZBOT_SIM='1',ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1').items()):raise RuntimeError('isolated SIM only')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'ws/src/navigation'
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--source',type=Path,default=NAV/'localization_trust_node.py');p.add_argument('--cycles',type=int,default=20);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
rclpy.init();node=rclpy.create_node('trust_shutdown_probe',namespace='shutdown_probe');pubs=[node.create_publisher(TFMessage,'tf',10),node.create_publisher(Odometry,'odom',qos_profile_sensor_data),node.create_publisher(LaserScan,'scan',qos_profile_sensor_data)];rows=[]
def feed():
    stamp=node.get_clock().now().to_msg();trs=[]
    for parent,child in [('map','odom'),('odom','base_footprint')]:
        tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id=parent;tr.child_frame_id=child;tr.transform.rotation.w=1.;trs.append(tr)
    pubs[0].publish(TFMessage(transforms=trs));o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.orientation.w=1.;pubs[1].publish(o)
    s=LaserScan();s.header.stamp=stamp;s.header.frame_id='base_footprint';s.angle_min=-.5;s.angle_max=.49;s.angle_increment=.01;s.range_min=.05;s.range_max=8.;s.ranges=[2.]*100;pubs[2].publish(s);rclpy.spin_once(node,timeout_sec=0.)
try:
    for i in range(a.cycles):
        log=a.output/f'cycle_{i:02d}.log';env=dict(os.environ,PYTHONPATH=str(NAV)+':'+os.environ.get('PYTHONPATH',''))
        args=['python3',str(a.source),'--ros-args','-r','__ns:=/shutdown_probe','-r','/qazbot_d1_map/tf:=/shutdown_probe/tf','-r','/qazbot_lio/scan:=/shutdown_probe/scan','-r','/qazbot_lio/odom_level:=/shutdown_probe/odom','-p','map_yaml:='+str(ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml')]
        with log.open('w') as stream:
            child=subprocess.Popen(args,env=env,stdout=stream,stderr=subprocess.STDOUT);deadline=time.monotonic()+10
            while 'localization_trust started' not in log.read_text() and child.poll() is None and time.monotonic()<deadline:feed();time.sleep(.01)
            ready='localization_trust started' in log.read_text();end=time.monotonic()+.5+(i%4)*.07
            while ready and child.poll() is None and time.monotonic()<end:feed();time.sleep(.005)
            if child.poll() is None:child.send_signal(signal.SIGINT if i%2==0 else signal.SIGTERM)
            end=time.monotonic()+8
            while child.poll() is None and time.monotonic()<end:feed();time.sleep(.005)
            forced=child.poll() is None
            if forced:child.kill()
            code=child.wait();rows.append(dict(cycle=i,ready=ready,signal='SIGINT' if i%2==0 else 'SIGTERM',exit_code=code,forced=forced));print(json.dumps(rows[-1]),flush=True)
    (a.output/'summary.json').write_text(json.dumps(dict(source=str(a.source),cycles=rows,passed=sum(r['ready'] and r['exit_code']==0 and not r['forced'] for r in rows)),indent=2)+'\n')
finally:node.destroy_node();rclpy.try_shutdown()
