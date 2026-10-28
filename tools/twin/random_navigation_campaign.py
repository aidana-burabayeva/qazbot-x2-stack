#!/usr/bin/env python3
"""Random known-free goals through the running RViz interface; SIM only.

Uses ordinary navigation feedback only for sequencing. Ground truth is read only
for scoring. Does not reseed, teleport, resume a fault, bypass stops or retry goals.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
import cv2
import numpy as np
from scipy import ndimage
import yaml

if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):
    raise RuntimeError('isolated SIM only')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[2]
ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('output',type=Path)
ap.add_argument('--count',type=int,default=8);ap.add_argument('--seed',type=int,default=20261026)
ap.add_argument('--timeout',type=float,default=240.)
ap.add_argument('--targets',type=Path);ap.add_argument('--indices',type=int,nargs='+');a=ap.parse_args()
if a.count<1 or a.timeout<=0:ap.error('positive count/timeout required')
a.output.mkdir();map_yaml=ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml';config=yaml.safe_load(map_yaml.read_text())
image=map_yaml.parent/config['image'];pixels=np.flipud(cv2.imread(str(image),0));res=config['resolution'];ox,oy,theta=config['origin']
if theta!=0:raise RuntimeError('this map sampler requires zero map yaw')
prob=pixels/255. if config.get('negate',0) else (255-pixels)/255.
free=prob<config['free_thresh'];clearance=ndimage.distance_transform_edt(free)*res
occupied=prob>config['occupied_thresh'];physical_clearance=ndimage.distance_transform_edt(~occupied)*res
safe=free & (clearance>=.52)
labels,_=ndimage.label(safe)
journey=json.loads((a.session/'journey.json').read_text());home=journey['home_truth']
hi,hj=int((home[1]-oy)/res),int((home[0]-ox)/res);component=labels[hi,hj]
if not component:raise RuntimeError('home not in safe free-space component')
rng=np.random.default_rng(a.seed);chosen=[]
for i in range(a.count):
    # Alternate tighter and more open targets; sample across the entire connected map.
    near=i%2==0;mask=(labels==component)&((clearance<.85) if near else (clearance>=.85))
    candidates=np.argwhere(mask);rng.shuffle(candidates)
    for row,col in candidates:
        x,y=ox+(int(col)+.5)*res,oy+(int(row)+.5)*res
        if math.hypot(x-home[0],y-home[1])<1.:continue
        if any(math.hypot(x-p['goal'][0],y-p['goal'][1])<1. for p in chosen):continue
        chosen.append(dict(index=i+1,goal=[x,y,float(rng.uniform(-math.pi,math.pi))],
                           category='near_boundary' if near else 'open',
                           known_free_clearance_m=float(clearance[row,col]),occupied_clearance_m=float(physical_clearance[row,col])))
        break
    else:raise RuntimeError('not enough separated safe goals')
if a.targets:
    saved=json.loads(a.targets.read_text())
    if saved['image_sha256']!=hashlib.sha256(image.read_bytes()).hexdigest():raise RuntimeError('saved target map differs')
    chosen=[p for p in saved['targets'] if a.indices is None or p['index'] in a.indices]
    if not chosen or (a.indices is not None and {p['index'] for p in chosen}!=set(a.indices)):raise RuntimeError('requested targets absent')
    for target in chosen:
        x,y,h=target['goal'];row,col=int((y-oy)/res),int((x-ox)/res)
        if not all(math.isfinite(v) for v in (x,y,h)) or not (0<=row<labels.shape[0] and 0<=col<labels.shape[1]) or labels[row,col]!=component:
            raise RuntimeError('saved target is not in connected known-free space')
    a.count=len(chosen);a.seed=saved['seed']
manifest=dict(seed=a.seed,count=a.count,map_sha256=hashlib.sha256(map_yaml.read_bytes()).hexdigest(),image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),session=str(a.session),home=home,targets=chosen)
(a.output/'targets.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest),flush=True)
rclpy.init();n=Node('random_navigation_campaign',namespace='qazbot_sim');status={};coord={};results=[]
def on_status(m):
    global status
    status=json.loads(m.data)
def on_coord(m):
    global coord
    coord=json.loads(m.data)
n.create_subscription(String,'round_trip/status',on_status,10)
n.create_subscription(String,'goal_coordinator/status',on_coord,10)
pub=n.create_publisher(PoseStamped,'rviz_goal',10);stop=n.create_client(Trigger,'round_trip/stop')
def spin():rclpy.spin_once(n,timeout_sec=.05)
def save():
    summary=dict(targets=manifest,results=results,
                 passed=sum(r.get('state')=='complete' for r in results),
                 completed=len(results),remaining=a.count-len(results))
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
def halt():
    if stop.service_is_ready():
        f=stop.call_async(Trigger.Request());end=time.monotonic()+3
        while not f.done() and time.monotonic()<end:spin()
active=False
try:
    end=time.monotonic()+15
    while not status or not coord or pub.get_subscription_count()==0:
        if time.monotonic()>end:raise RuntimeError('running interactive stack not found')
        spin()
    if status.get('backend')!='kinematic':raise RuntimeError('random campaign requires kinematic stand')
    for target in chosen:
        deadline=time.monotonic()+15
        while not (coord.get('ready') and not coord.get('busy') and status.get('state') in ('ready','complete','completed_outside_tolerance','outbound_failed','return_failed')):
            if coord.get('latched'):raise RuntimeError('fault latched; no automatic resume '+str(coord))
            if time.monotonic()>deadline:raise RuntimeError('stack not ready for new goal')
            spin()
        before=json.loads((a.session/'journey.json').read_text());old=len(before['events'])
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();x,y,h=target['goal'];p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2)
        print('START '+json.dumps(target),flush=True);pub.publish(p);started=time.time();active=False
        while True:
            spin();active=active or status.get('state')=='outbound'
            if active and status.get('state') in ('complete','completed_outside_tolerance','outbound_failed','return_failed'):
                current=json.loads((a.session/'journey.json').read_text());events=current['events'][old:]
                terminal=next(e for e in reversed(events) if 'state' in e)
                result=dict(**target,started=started,duration_s=time.time()-started,**{k:v for k,v in terminal.items() if k not in ('time','goal')},events=events)
                results.append(result);save();print('RESULT '+json.dumps(result),flush=True);break
            if time.time()-started>a.timeout:
                halt();results.append(dict(**target,state='timeout',duration_s=time.time()-started));save()
                raise RuntimeError('trial timeout: stopped campaign, no automatic replay')
            if not active and time.time()-started>8:raise RuntimeError('goal not accepted')
        end=time.monotonic()+1
        while time.monotonic()<end:spin()
except BaseException as exc:
    if active:halt()
    (a.output/'error.txt').write_text(str(exc)+'\n');save();raise
finally:
    n.destroy_node();rclpy.try_shutdown()
