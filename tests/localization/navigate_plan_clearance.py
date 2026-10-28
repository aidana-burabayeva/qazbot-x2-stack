import sys, math, json, numpy as np, yaml, rclpy
from PIL import Image
from scipy.ndimage import distance_transform_edt
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import os
D=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','..','ws','src','maps','first_map_nav2_v1')+'/'
meta=yaml.safe_load(open(D+'map.yaml')); res=meta['resolution']; ox,oy=meta['origin'][:2]
free=np.flipud((255-np.asarray(Image.open(D+meta['image'])))/255.0<meta['free_thresh']); K=10
fd=distance_transform_edt(np.kron(free,np.ones((K,K),bool)))*(res/K)
def clear(p):
    pts=[p[0]]
    for a,b in zip(p,p[1:]): pts.extend(np.linspace(a,b,max(2,math.ceil(math.dist(a,b)/(res/K))+1))[1:])
    xy=np.asarray(pts); c=np.floor((xy[:,0]-ox)/(res/K)).astype(int); r=np.floor((xy[:,1]-oy)/(res/K)).astype(int)
    return float(fd[r,c].min())
rclpy.init(); n=Node('nav_clear'); cl=ActionClient(n,NavigateToPose,'/qazbot_d1_map/navigate_to_pose'); plans=[]
n.create_subscription(Path,'/qazbot_d1_map/plan',plans.append,10); assert cl.wait_for_server(timeout_sec=30)
for gx,gy in [(0.775,-2.925),(-0.425,-5.675),(3.0,-1.0)]:
    plans.clear(); g=NavigateToPose.Goal(); g.pose.header.frame_id='map'; g.pose.pose.position.x=gx; g.pose.pose.position.y=gy; g.pose.pose.orientation.w=1.0
    f=cl.send_goal_async(g); rclpy.spin_until_future_complete(n,f,timeout_sec=20); h=f.result(); r=h.get_result_async(); rclpy.spin_until_future_complete(n,r,timeout_sec=40)
    for _ in range(20): rclpy.spin_once(n,timeout_sec=0.1)
    if plans:
        p=[(q.pose.position.x,q.pose.position.y) for q in plans[-1].poses]
        print(f'goal ({gx},{gy}): status {r.result().status}  path {len(p)} poses, {sum(math.dist(a,b) for a,b in zip(p,p[1:])):.2f} m, start ({p[0][0]:.2f},{p[0][1]:.2f}), exact min clearance {clear(p):.3f} m')
    else: print(f'goal ({gx},{gy}): status {r.result().status} (4=SUCCEEDED), no path')
