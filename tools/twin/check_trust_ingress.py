#!/usr/bin/env python3
"""ROS probe: diagnostic lock must not block raw inputs; absent inputs still expire.
Publishes only isolated probe TF/odometry/scan, never motion commands.
"""
import argparse,json,math,os,sys,threading,time
from pathlib import Path
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):raise RuntimeError('isolated SIM only')
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'ws/src/navigation'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from localization_trust_node import TrustNode
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);a=ap.parse_args();a.output.mkdir()
rclpy.init(args=['--ros-args','-r','__ns:=/ingress_probe','-r','/qazbot_d1_map/tf:=/ingress_probe/tf','-r','/qazbot_lio/scan:=/ingress_probe/scan','-r','/qazbot_lio/odom_level:=/ingress_probe/odom'])
rx=TrustNode();tx=Node('probe_source');executor=SingleThreadedExecutor();executor.add_node(rx.input_node)
def receive():
    try:executor.spin()
    except ExternalShutdownException:pass
thread=threading.Thread(target=receive,daemon=True);thread.start()
tf_pub=tx.create_publisher(TFMessage,'tf',50);odom_pub=tx.create_publisher(Odometry,'odom',10);scan_pub=tx.create_publisher(LaserScan,'scan',10)
count=0
def publish_for(seconds):
    global count
    end=time.monotonic()+seconds;next_tick=time.monotonic()
    while time.monotonic()<end:
        stamp=tx.get_clock().now().to_msg();body=TransformStamped();body.header.stamp=stamp;body.header.frame_id='odom';body.child_frame_id='base_footprint';body.transform.rotation.w=1.;transforms=[body]
        if count%10==0:
            m=TransformStamped();m.header.stamp=stamp;m.header.frame_id='map';m.child_frame_id='odom';m.transform.rotation.w=1.;transforms.append(m)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint';scan.angle_min=-.1;scan.angle_max=.1;scan.angle_increment=.1;scan.range_min=.45;scan.range_max=8.;scan.ranges=[1.,2.,3.];scan_pub.publish(scan)
        tf_pub.publish(TFMessage(transforms=transforms))
        if count%5==0:
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_footprint';odom.pose.pose.orientation.w=1.;odom_pub.publish(odom)
        count+=1;next_tick+=.02;time.sleep(max(0.,next_tick-time.monotonic()))
try:
    deadline=time.monotonic()+10
    while not all(p.get_subscription_count() for p in (tf_pub,odom_pub,scan_pub)):
        if time.monotonic()>deadline:raise RuntimeError('probe subscriptions not discovered')
        time.sleep(.05)
    publish_for(.6)
    with rx.lock:
        publish_for(1.)
        now=time.monotonic();ages={k:now-v for k,v in dict(odom=rx.odom_t,odom_msg=rx.odom_msg_t,scan=rx.scan_t,map=rx.mo_t).items()}
        queued=len(rx.pending_map_steps)
    receiving=all(math.isfinite(x) and 0<=x<.4 for x in ages.values()) and queued>=4
    time.sleep(.8)
    inp=rx._inputs()
    with rx.input_lock:steps=list(rx.pending_map_steps);rx.pending_map_steps.clear()
    with rx.lock:
        for pose in steps:rx.ev.observe_map_odom(pose)
        verdict=rx.ev.evaluate(inp)
    expired={'T1_odom_stale','T1_odom_message_stale','T1_scan_stale'}<=set(verdict['reasons'])
    result=dict(passed=receiving and expired,receive_while_lock_held=receiving,held_lock_s=1.,ages_while_locked=ages,preserved_map_steps=queued,real_input_loss_detected=expired,loss_reasons=verdict['reasons'],no_motion_publisher=True)
    (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)
finally:
    executor.shutdown();thread.join(timeout=2);rx.input_node.destroy_node();rx.destroy_node();tx.destroy_node();rclpy.try_shutdown()
