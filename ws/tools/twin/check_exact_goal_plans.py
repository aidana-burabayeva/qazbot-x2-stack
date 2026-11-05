#!/usr/bin/env python3
"""Read-only ComputePathToPose checks: exact sub-cell goals and preserved bounds failures."""
import argparse,json,math,os,time
from pathlib import Path
import yaml
if any(os.environ.get(k)!=v for k,v in {'QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87','ROS_LOCALHOST_ONLY':'1'}.items()):raise RuntimeError('isolated SIM only')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose
ap=argparse.ArgumentParser();ap.add_argument('output',type=Path);ap.add_argument('--baseline',action='store_true');a=ap.parse_args();a.output.mkdir()
ROOT=Path(__file__).resolve().parents[3];m=yaml.safe_load((ROOT/'ws/src/maps/first_map_nav2_v3/map.yaml').read_text());res=m['resolution'];ox,oy,_=m['origin'];cx=ox+math.floor((-1.275-ox)/res)*res;cy=oy+math.floor((-4.825-oy)/res)*res
rclpy.init();n=Node('exact_goal_plan_probe',namespace='qazbot_sim');client=ActionClient(n,ComputePathToPose,'nav2/compute_path_to_pose');motions=[];truth=[];rows=[]
n.create_subscription(Twist,'cmd_vel_authorized',lambda v:motions.append((v.linear.x,v.linear.y,v.angular.z)),10)
n.create_subscription(Odometry,'model_truth',lambda v:truth.append((v.pose.pose.position.x,v.pose.pose.position.y)),10)
def spin(seconds=.02):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def wait(pred,seconds=30):
    end=time.monotonic()+seconds
    while not pred():
        if time.monotonic()>end:raise RuntimeError('probe deadline')
        spin()
def pose(x,y,h):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);return p
def request(name,start,goal,valid=True):
    q=ComputePathToPose.Goal();q.start=pose(*start);q.goal=pose(*goal);q.use_start=True;q.planner_id='GridBased'
    f=client.send_goal_async(q);wait(f.done);handle=f.result()
    if not handle.accepted:row=dict(name=name,status='rejected',valid=valid)
    else:
        f=handle.get_result_async();wait(f.done);result=f.result();path=result.result.path
        row=dict(name=name,status=result.status,valid=valid,poses=len(path.poses))
        if path.poses:
            p=path.poses[-1].pose;h=2*math.atan2(p.orientation.z,p.orientation.w)
            row.update(last=[p.position.x,p.position.y,h],requested=list(goal),xy_error=math.hypot(p.position.x-goal[0],p.position.y-goal[1]),yaw_error=abs(math.atan2(math.sin(h-goal[2]),math.cos(h-goal[2]))))
    rows.append(row);print(json.dumps(row),flush=True)
try:
    wait(client.server_is_ready);spin(1)
    offsets=[(.005,.005,.045,.045,-math.pi+.01),(.045,.008,.008,.040,.8),(.025,.025,.042,.011,-1.2),(.012,.038,.041,.016,math.pi-.01)]
    for i,(sx,sy,gx,gy,h) in enumerate(offsets):request('same_cell_'+str(i),(cx+sx,cy+sy,0.),(cx+gx,cy+gy,h))
    request('neighbor_noncenter',(cx+.01,cy+.04,0.),(cx+res+.01,cy+.012,.5))
    request('goal_outside',(cx+.01,cy+.01,0.),(ox-1.,cy,0.),False)
    request('start_outside',(ox-1.,cy,0.),(cx+.04,cy+.04,.2),False)
    request('valid_after_bounds_failures',(cx+.015,cy+.01,0.),(cx+.045,cy+.04,.7))
    spin(.3)
    bounds_ok=all(r.get('status')!=4 for r in rows if not r['valid'])
    valid_ok=all(r.get('status')==4 and r.get('yaw_error',1.)<1e-6 for r in rows if r['valid'])
    defects=sum(r.get('xy_error',0.)>1e-6 for r in rows if r['valid'])
    nonzero=sum(any(abs(x)>1e-9 for x in v) for v in motions)
    displacement=0. if not truth else max(math.dist(p,truth[0]) for p in truth)
    passed=bounds_ok and valid_ok and nonzero==0 and displacement<1e-6 and (defects>0 if a.baseline else defects==0)
    summary=dict(passed=passed,baseline=a.baseline,endpoint_defects=defects,bounds_ok=bounds_ok,nonzero_authorized_commands=nonzero,truth_displacement_m=displacement,checks=rows)
    (a.output/'results.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)
    if not passed:raise SystemExit(1)
finally:n.destroy_node();rclpy.try_shutdown()
