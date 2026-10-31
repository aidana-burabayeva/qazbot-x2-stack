#!/usr/bin/env python3
"""Extract the motion/localization story from a closed core bag into small CSVs (read-only analysis on the laptop; pure-python mcap, no ROS needed).
usage: /tmp/mcapenv/bin/python extract_motion.py <bag_dir> <out_dir>   (pip install mcap mcap-ros2-support)"""
import csv, math, os, sys, collections, glob
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
bag, out = sys.argv[1], sys.argv[2]
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
WANT = {'/qazbot_d1_map/cmd_vel_authorized', '/aima/mc/locomotion/velocity', '/qazbot_native/odom', '/aima/mc/leg_odometry', '/qazbot_d1_map/amcl_pose',
        '/qazbot_d1_map/tf', '/qazbot_d1_map/initialpose', '/qazbot_d1_map/goal_pose', '/aima/mc/body_pose'}
rows = collections.defaultdict(list); n_leg = 0
for f in sorted(glob.glob(os.path.join(bag, '*.mcap'))):
    with open(f, 'rb') as fh:
        rd = make_reader(fh, decoder_factories=[DecoderFactory()])
        for schema, ch, msg, m in rd.iter_decoded_messages(topics=sorted(WANT)):
            t = msg.log_time * 1e-9; tp = ch.topic
            if tp == '/qazbot_d1_map/cmd_vel_authorized': rows['cmd_authorized'].append((t, m.linear.x, m.linear.y, m.angular.z))
            elif tp == '/aima/mc/locomotion/velocity': rows['mc_command'].append((t, m.forward_velocity, m.lateral_velocity, m.angular_velocity))
            elif tp == '/qazbot_native/odom': p = m.pose.pose; rows['native_odom'].append((t, p.position.x, p.position.y, yaw(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == '/aima/mc/leg_odometry':
                n_leg += 1
                if n_leg % 5 == 0: p = m.pose.pose; rows['leg_odom'].append((t, p.position.x, p.position.y, yaw(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == '/qazbot_d1_map/amcl_pose': p = m.pose.pose; rows['amcl_pose'].append((t, p.position.x, p.position.y, yaw(p.orientation), m.pose.covariance[0], m.pose.covariance[35]))
            elif tp == '/qazbot_d1_map/initialpose': p = m.pose.pose; rows['initialpose'].append((t, p.position.x, p.position.y, yaw(p.orientation)))
            elif tp == '/qazbot_d1_map/goal_pose': p = m.pose; rows['goal_pose'].append((t, p.position.x, p.position.y, yaw(p.orientation)))
            elif tp == '/qazbot_d1_map/tf':
                for tr in m.transforms:
                    if (tr.header.frame_id, tr.child_frame_id) == ('map', 'odom'):
                        x = tr.transform; rows['map_to_odom'].append((t, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, x.translation.x, x.translation.y, yaw(x.rotation)))
hdr = {'cmd_authorized': 't,vx,vy,wz', 'mc_command': 't,vx,vy,wz', 'native_odom': 't,x,y,yaw,vx,wz', 'leg_odom': 't,x,y,yaw,vx,wz', 'amcl_pose': 't,x,y,yaw,cov_xx,cov_yawyaw',
       'initialpose': 't,x,y,yaw', 'goal_pose': 't,x,y,yaw', 'map_to_odom': 't_rx,t_stamp,x,y,yaw'}
os.makedirs(out, exist_ok=True)
for k, v in rows.items():
    with open(os.path.join(out, k + '.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(hdr[k].split(',')); w.writerows([[f'{c:.6f}' for c in row] for row in v])
    print(k, len(v))
